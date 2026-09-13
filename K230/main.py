'''
钢球识别 + 串口球心坐标输出 - K230一号板[控制眼](无无线/无图传)
用途: 识别钢球, 直接把球心像素坐标输出给 MSPM0G3507 核心板
协议: ASCII每帧一行 "cx\n"
      cx = 球心在显示画面(800x480)中的X像素坐标, 左=0, 右=799
      无球时 cx=-1
      (2026-08-01 简化: 球在管子上滚只有X方向变化, 不再发Y坐标)
接线: K230排针18脚(BANK0_GPIO3, UART1_TXD) -> MSPM0的UART_RX; G -> GND(必须共地)
      备选: 排针17脚(BANK0_GPIO5, UART2_TXD)。注意排针13脚(IO4)只支持UART1_RXD不能当TX
屏幕显示: 左上 FPS/球心坐标, 红框+球心十字

说明(2026-07-31 精简版):
  - 已移除管子/蓝点标记识别、放球标定、距离计算、轨道 ROI 过滤,
    只保留"检测钢球 -> 卡尔曼平滑球心 -> 串口输出坐标"核心链路
  - 传感器采集 320x240(带宽减半, 模型有效输入 224x168 不变), 失败自动回退 640x480
  - 后处理 top-1 快速路径: 不转置、不排序, 直接 argmax
  - GC 每 120 帧一次; SHOW_EVERY_N 显示跳帧; STAGE_TIMING 分段耗时统计
'''
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os, sys, gc, time
from machine import UART, FPIOA
from media.media import *
from media.sensor import Sensor
import nncase_runtime as nn
import ulab.numpy as np
import image


def ALIGN_UP(x, align):
    return (x + align - 1) // align * align


class KalmanCV1D:
    """匀速模型卡尔曼(位置+速度), 一维, 纯Python标量运算。
    比一阶滤波多一个速度状态: 球滚动时用速度外推, 几乎消除跟踪滞后;
    球静止时速度估计收敛到0, 退化为平滑滤波, 不引入抖动。"""
    def __init__(self, q_pos=1.0, q_vel=1000.0, r=4.0):
        self.q_pos = q_pos      # 位置过程噪声
        self.q_vel = q_vel      # 速度过程噪声(大=允许急加速, 更跟手)
        self.r = r              # 观测噪声(检测框中心的抖动, ~±2px 取4)
        self.x = 0.0            # 位置
        self.v = 0.0            # 速度(px/s)
        self.p00 = 100.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 100.0
        self.init = False

    def reset(self, z):
        self.x = float(z)
        self.v = 0.0
        self.p00 = 100.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 100.0
        self.init = True

    def update(self, z, dt):
        if not self.init:
            self.reset(z)
            return self.x
        dt = max(0.001, min(0.1, dt))   # 钳帧间隔, 防卡帧后外推飞掉
        # ---- 预测: x += v*dt, P = F*P*F' + Q ----
        self.x += self.v * dt
        p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11 + self.q_pos
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11 + self.q_vel
        # ---- 更新(观测只有位置): K = P*H' / (H*P*H' + R) ----
        s = p00 + self.r
        k0 = p00 / s
        k1 = p10 / s
        y = z - self.x
        self.x += k0 * y
        self.v += k1 * y
        self.p00 = p00 - k0 * p00
        self.p01 = p01 - k0 * p01
        self.p10 = p10 - k1 * p00
        self.p11 = p11 - k1 * p01
        return self.x


class SteelBallDetApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size,
                 confidence_threshold=0.5, nms_threshold=0.5,
                 use_nms=False,
                 rgb888p_size=[320, 240], display_size=[800, 480], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.use_nms = use_nms
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        # letterbox 参数(在 config_preprocess 中计算)
        self.pad_top = 0
        self.pad_left = 0
        self.scale = 1.0
        self.x_ratio = 1.0
        self.y_ratio = 1.0

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            sw, sh = ai2d_input_size
            mw, mh = self.model_input_size
            s = min(float(mw) / sw, float(mh) / sh)
            cw, ch = sw * s, sh * s
            pad_left_m = (mw - cw) / 2.0
            pad_top_m = (mh - ch) / 2.0
            self.pad_top = pad_top_m
            self.pad_left = pad_left_m
            self.scale = s
            pt = int(round(pad_top_m / s))
            pb = int(round((mh - ch - pad_top_m) / s))
            pl_ = int(round(pad_left_m / s))
            pr = int(round((mw - cw - pad_left_m) / s))
            self.ai2d.pad([0, 0, 0, 0, pt, pb, pl_, pr], 0, [114, 114, 114])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, sh, sw], [1, 3, mh, mw])
            # 模型坐标 -> 显示坐标 的比例, 预计算避免每帧重复
            self.x_ratio = float(self.display_size[0]) / mw
            self.y_ratio = float(self.display_size[1]) / (mh - 2.0 * pad_top_m)

    def _make_det(self, data, idx, score):
        """模型坐标 -> 显示坐标, 返回 [x, y, w, h, score]"""
        cx = float(data[0, idx]); cy = float(data[1, idx])
        w = float(data[2, idx]);  h = float(data[3, idx])
        x1 = (cx - w / 2.0 - self.pad_left) * self.x_ratio
        y1 = (cy - h / 2.0 - self.pad_top) * self.y_ratio
        return [int(x1), int(y1), int(w * self.x_ratio), int(h * self.y_ratio), score]

    # 后处理: 默认 top-1 快速路径(不转置/不排序, 直接 argmax);
    # use_nms=True 时走通用路径。兼容 YOLOv8-det [1,5,N] 和 YOLOv8-seg [1,37,N]
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            data = results[0][0]            # (C, N): 4坐标 + nc类分数 + (可选32掩码系数)
            total_ch = data.shape[0]
            if total_ch >= 37:
                nc = total_ch - 36          # 分割模型, 后面 32 是 mask 系数
            else:
                nc = total_ch - 4           # 纯检测模型
            if nc <= 0:
                nc = 1
            if nc == 1:
                max_scores = data[4]        # (N,) 单行, 无需转置/最大值归并
            else:
                max_scores = np.max(data[4:4 + nc], axis=0)

            if not self.use_nms:
                # ---- top-1 快速路径 ----
                best = int(np.argmax(max_scores))
                s = float(max_scores[best])
                if s < self.confidence_threshold:
                    return []
                return [self._make_det(data, best, s)]

            # ---- NMS 通用路径(默认不走) ----
            data_t = data.transpose()       # (N, C)
            order = np.argsort(-max_scores, axis=0)[:50]
            boxes, scores, ids = [], [], []
            for i in range(len(order)):
                idx = int(order[i])
                s = float(max_scores[idx])
                if s < self.confidence_threshold:
                    break
                boxes.append([float(data_t[idx, 0]), float(data_t[idx, 1]),
                              float(data_t[idx, 2]), float(data_t[idx, 3])])
                scores.append(s)
                ids.append(idx)
            keep = self._nms(boxes, scores, self.nms_threshold)
            return [self._make_det(data, ids[i], scores[i]) for i in keep]

    def _nms(self, boxes, scores, thresh):
        if len(boxes) == 0:
            return []
        order = sorted(range(len(scores)), key=lambda i: -scores[i])
        keep = []
        while order:
            i = order.pop(0)
            keep.append(i)
            rest = []
            for j in order:
                iou = self._iou(boxes[i], boxes[j])
                if iou < thresh:
                    rest.append(j)
            order = rest
        return keep

    def _iou(self, a, b):
        ax1, ay1, ax2, ay2 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
        bx1, by1, bx2, by2 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2-ix1), max(0.0, iy2-iy1)
        inter = iw * ih
        ua = a[2]*a[3] + b[2]*b[3] - inter
        return inter / ua if ua > 0 else 0.0

    def draw_result(self, pl, dets, fps=None, ball_xy=None):
        """左上角 FPS+球心坐标, 钢球红框, 球心黄色十字"""
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pl.osd_img.clear()
            if fps is not None:
                txt = "FPS:%.1f" % fps
                if ball_xy is not None:
                    txt += " | (%d,%d)" % (ball_xy[0], ball_xy[1])
                else:
                    txt += " | NO BALL"
                pl.osd_img.draw_string_advanced(
                    10, 10, 32, txt, color=(255, 0, 255, 0))  # (A,R,G,B) 不透明绿
            for d in dets:
                x1, y1, w, h, score = d
                x1 = max(0, x1); y1 = max(0, y1)
                if w <= 0 or h <= 0:
                    continue
                pl.osd_img.draw_rectangle(x1, y1, w, h, color=(255, 255, 0, 0), thickness=3)
                pl.osd_img.draw_string_advanced(
                    x1, max(0, y1 - 40), 28,
                    " %s %.2f" % (self.labels[0], score), color=(255, 255, 0, 0))
            # 球心十字(黄)
            if ball_xy is not None:
                bx, by = ball_xy
                pl.osd_img.draw_line(bx - 14, by, bx + 14, by, color=(255, 255, 255, 0), thickness=2)
                pl.osd_img.draw_line(bx, by - 14, bx, by + 14, color=(255, 255, 255, 0), thickness=2)


# ============== 日志 ==============
LOG_FILE = "/sdcard/a_run_error.log"


def log_msg(msg):
    """写日志到 TF 卡，方便排查黑屏/重启问题"""
    try:
        t = time.localtime()
        ts = "%04d%02d%02d_%02d%02d%02d" % (t[0], t[1], t[2], t[3], t[4], t[5])
        with open(LOG_FILE, "a") as f:
            f.write("[%s] %s\n" % (ts, msg))
    except Exception:
        pass
    print(msg)


if __name__ == "__main__":
    # 启动前清空旧日志，便于只看本次运行的错误
    try:
        with open(LOG_FILE, "w") as f:
            f.write("")
    except Exception:
        pass

    # ===================== 可修改参数 =====================
    display_mode = "lcd"           # 3.5寸屏; 接显示器改 "hdmi"
    if display_mode == "hdmi":
        display_size = [1920, 1080]
    else:
        display_size = [800, 480]

    # 模型配置: YOLOv8n-det 检测模型, 224x224 输入
    kmodel_path = "/sdcard/steel_ball_det_224.kmodel"
    labels = ["steel_ball"]
    confidence_threshold = 0.5     # 漏检调低(如0.3), 误检调高(如0.6)
    nms_threshold = 0.5
    use_nms = False                # 单钢珠场景必须 False, top-1 快速路径
    MODEL_SIZE = 224               # 检测模型输入尺寸

    # 传感器采集分辨率: 320x240 带宽减半, 模型有效输入不变(224x168), 精度不降
    # 若屏幕预览异常或创建失败, 会自动回退 640x480
    SENSOR_SIZE = [320, 240]

    SHOW_EVERY_N = 1               # 每N帧刷新一次屏幕; 推理/串口不受影响。瓶颈在显示时改 2 或 3
    STAGE_TIMING = True            # 每60帧打印 采集/推理/绘制 平均耗时, 定位瓶颈后可关
    UART_BAUD = 115200              # 与MSPM0端波特率一致
    SENSOR_ID = 1                  # 摄像头接口: 0=CSI0, 1=CSI1, 2=CSI2

    # 匀速卡尔曼参数(位置+速度):
    # KF_QV 大 = 允许急加速更跟手但噪声大; KF_R 小 = 更信任观测(更跟手)
    KF_QV = 1000.0
    KF_R = 4.0
    # ======================================================

    # 串口发送脚候选, 按顺序尝试, 第一个成功的生效:
    #   IO3 =UART1_TXD: 40Pin排针第18脚(BANK0_GPIO3) <- 推荐, TX接这里
    #   IO5 =UART2_TXD: 40Pin排针第17脚(BANK0_GPIO5) <- 备选
    #   IO11=UART2_TXD: 未引到40Pin排针(GH1.25座子), 保底
    # 注意1: 排针13脚=IO4 只支持 UART1_RXD 只能收不能发(B板用它当RX)
    # 注意2: 本固件构造UART前必须同时配好TX和RX脚(实测报错 rx not configured),
    #        RX脚只需软件配置, 不用接线
    UART_TX_CANDIDATES = [
        (3,  "UART1_TXD", 4,  "UART1_RXD", UART.UART1),  # TX=排针18, RX=排针13(空接)
        (5,  "UART2_TXD", 6,  "UART2_RXD", UART.UART2),  # TX=排针17, RX=排针20(空接)
        (11, "UART2_TXD", 12, "UART2_RXD", UART.UART2),  # TX=GH1.25座子(空接RX)
    ]
    uart = None
    log_msg("A board startup, SENSOR_ID=%d" % SENSOR_ID)
    fpioa = FPIOA()
    for tx_pin, tx_func, rx_pin, rx_func, uid in UART_TX_CANDIDATES:
        try:
            fpioa.set_function(tx_pin, getattr(FPIOA, tx_func))
            fpioa.set_function(rx_pin, getattr(FPIOA, rx_func))
            uart = UART(uid, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
            log_msg("uart ready: tx=pin%d(%s) rx=pin%d, baud %d"
                    % (tx_pin, tx_func, rx_pin, UART_BAUD))
            break
        except Exception as e:
            log_msg("uart init fail on pin %d (%s): %s" % (tx_pin, tx_func, str(e)))
            uart = None
    if uart is None:
        log_msg("uart init fail: no usable TX pin!")

    pl = None
    seg = None
    pl_created = False
    rgb888p_size = list(SENSOR_SIZE)
    frame_count = 0
    last_tick = time.ticks_ms()
    fps = 0.0
    try:
        # 创建管线, 320x240 失败时自动回退 640x480
        for try_size in (rgb888p_size, [640, 480]):
            try:
                rgb888p_size = try_size
                pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size,
                              display_mode=display_mode)
                # 使用指定摄像头接口(CSI2), 并做画面镜像修正
                sensor = Sensor(id=SENSOR_ID)
                try:
                    pl.create(sensor=sensor, hmirror=True, vflip=True)
                    log_msg("pipeline created, sensor id=%d, size=%s"
                            % (SENSOR_ID, str(rgb888p_size)))
                except TypeError:
                    # 旧版 PipeLine 不支持 sensor 参数, 回退到默认接口
                    log_msg("fallback pipeline without sensor param")
                    pl.create(hmirror=True, vflip=True)
                    try:
                        pl.sensor.stop()
                        pl.sensor.set_hmirror(True)
                        pl.sensor.set_vflip(True)
                        pl.sensor.run()
                    except Exception as e:
                        log_msg("flip fail: %s" % str(e))
                pl_created = True
                break
            except Exception as e:
                log_msg("pipeline create fail with size %s: %s" % (str(try_size), str(e)))
                pl = None
                pl_created = False
                if try_size[0] == 640:
                    raise

        # 模型加载失败显示在屏幕上
        try:
            seg = SteelBallDetApp(kmodel_path, labels=labels,
                                  model_input_size=[MODEL_SIZE, MODEL_SIZE],
                                  confidence_threshold=confidence_threshold,
                                  nms_threshold=nms_threshold,
                                  use_nms=use_nms,
                                  rgb888p_size=rgb888p_size,
                                  display_size=display_size, debug_mode=0)
            seg.config_preprocess()
            log_msg("model loaded ok")
        except Exception as e:
            log_msg("model init fail: %s" % str(e))
            try:
                sys.print_exception(e)
            except Exception:
                pass
            fail_cnt = 0
            while True:
                pl.osd_img.clear()
                pl.osd_img.draw_string_advanced(10, 180, 40, "MODEL FAIL!", color=(255, 255, 0, 0))
                pl.osd_img.draw_string_advanced(10, 240, 24, "check /sdcard kmodel file", color=(255, 255, 255, 0))
                pl.show_image()
                fail_cnt += 1
                if fail_cnt % 30 == 0:
                    gc.collect()
                time.sleep_ms(30)
        log_msg("钢球识别已启动(检测模型 %dx%d, 采集 %dx%d)"
                % (MODEL_SIZE, MODEL_SIZE, rgb888p_size[0], rgb888p_size[1]))

        # 卡尔曼滤波器初始化(匀速模型, x/y 各一个)
        kf_x = KalmanCV1D(q_vel=KF_QV, r=KF_R)
        kf_y = KalmanCV1D(q_vel=KF_QV, r=KF_R)
        kf_initialized = False

        # 分段耗时统计(us)
        t_get = 0
        t_run = 0
        t_show = 0
        had_ball = False

        while True:
            try:
                s0 = time.ticks_us()
                img = pl.get_frame()
                s1 = time.ticks_us()
                dets = seg.run(img)
                s2 = time.ticks_us()

                now_tick = time.ticks_ms()
                dt = time.ticks_diff(now_tick, last_tick)
                last_tick = now_tick
                if dt > 0:
                    fps = fps * 0.9 + 0.1 * (1000.0 / dt)

                # 球心 + 卡尔曼滤波
                ball_xy = None
                if dets:
                    raw_x = dets[0][0] + dets[0][2] // 2
                    raw_y = dets[0][1] + dets[0][3] // 2
                    dt_sec = dt / 1000.0 if dt > 0 else 0.016
                    if not kf_initialized:
                        kf_x.reset(raw_x)
                        kf_y.reset(raw_y)
                        kf_initialized = True
                    elif abs(raw_x - kf_x.x) > 80 or abs(raw_y - kf_y.x) > 80:
                        # 观测跳变过大, 重置滤波器(球被拿起/放下等)
                        kf_x.reset(raw_x)
                        kf_y.reset(raw_y)
                    ball_x = int(round(kf_x.update(raw_x, dt_sec)))
                    ball_y = int(round(kf_y.update(raw_y, dt_sec)))
                    # 钳到画面范围内
                    ball_x = max(0, min(display_size[0] - 1, ball_x))
                    ball_y = max(0, min(display_size[1] - 1, ball_y))
                    ball_xy = (ball_x, ball_y)
                    if not had_ball:
                        had_ball = True
                        log_msg("ball found, conf=%.2f" % dets[0][4])
                else:
                    if had_ball:
                        had_ball = False
                        log_msg("ball lost")

                # 串口发送球心X坐标(每帧都发, 不受显示跳帧影响)
                if uart is not None:
                    try:
                        if ball_xy is not None:
                            uart.write("%d\n" % ball_xy[0])
                        else:
                            # 无球时发送特殊值
                            uart.write("-1\n")
                    except Exception as e:
                        log_msg("uart write fail: %s" % str(e))

                # 显示(可跳帧): FPS + 坐标 + 红框 + 球心十字
                if frame_count % SHOW_EVERY_N == 0:
                    seg.draw_result(pl, dets, fps=fps, ball_xy=ball_xy)
                    pl.show_image()
                s3 = time.ticks_us()

                # 分段耗时统计
                if STAGE_TIMING:
                    t_get += time.ticks_diff(s1, s0)
                    t_run += time.ticks_diff(s2, s1)
                    t_show += time.ticks_diff(s3, s2)
                    if frame_count % 60 == 59:
                        log_msg("stage ms: get=%.1f infer=%.1f draw=%.1f fps=%.1f"
                                % (t_get / 60000.0, t_run / 60000.0, t_show / 60000.0, fps))
                        t_get = 0
                        t_run = 0
                        t_show = 0
            except Exception as e:
                log_msg("main loop error: %s" % str(e))
                try:
                    import traceback
                    traceback.print_exc()
                except Exception:
                    pass
            frame_count += 1
            if frame_count % 120 == 0:
                gc.collect()
    except Exception as e:
        log_msg("run fatal error: %s" % str(e))
        try:
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        try:
            if uart is not None:
                uart.deinit()
        except Exception:
            pass
        try:
            if seg is not None:
                seg.deinit()
        except Exception:
            pass
        try:
            if pl is not None and pl_created:
                pl.destroy()
        except Exception:
            pass
