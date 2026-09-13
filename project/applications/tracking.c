/*
 * 循迹小车控制
 * 10ms 控制线程（轻环）：编码器采样 → 四向按键扫描 → 任务命令 → 差速 → 速度环 → PWM
 * 灰度线程（低优先级）：采样 → 循迹误差/丢线 → 转向速度
 * 任务启停由菜单 UI 经 Task_PostCmd 投递 (PTZ Modbus 事务不跨线程)
 * 行驶+控球 (T45/T6): 循迹行驶同时稳球, 行驶中动态调度控球 Kv/缓启停前馈
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include "tracking.h"
#include "gray_module.h"
#include "dir.h"
#include "motor.h"
#include "ptz.h"
#include "ball.h"
#include "comm.h"
#include "key.h"
#include <drivers/pulse_encoder.h>

#define DBG_TAG "track"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* 编码器设备：pulse4=右轮 */
#define ENC_R_DEV   "pulse4"
#define ENC_L_DEV   "pulse3"
#define ENC_L_SIGN  (-1)
#define ENC_R_SIGN  (+1)
/* 单拍计数差上限: 正常≤~40(速度22+刹车瞬态), 超过判为毛刺丢弃
 * (来源: 驱动回绕判向竞态 / PA6 PA7 受相邻PA8电机PWM耦合), 防速度环满占空比抽动 */
#define ENC_DELTA_MAX  100

#define CTRL_PERIOD_MS  10
#define TARGET_SPEED    25
#define SOFT_START_INIT 1    /* 缓启动初始速度 */
#define SOFT_STEP_TICKS 30u  /* 每档 300ms */
#define GRAY_PERIOD_MS  10   /* 灰度线程采样周期; 整轮扫描本身另耗约 1-2ms */

/* 行驶任务参数: 行驶+控球任务降速降Kp保球稳 */
#define TRACK_SPD_FAST  TARGET_SPEED             /* 纯循迹 */
#define TRACK_SPD_SLOW  18                       /* 行驶+控球 */
#define TRACK_KP_FAST   TRACK_KP_BASE            /* 纯循迹转向 Kp (dir.h 共享) */
#define TRACK_KP_SLOW   (0.20f * 2.0f / 3.0f)

/* 1.Track: lap 到 17s 开始预减速到 TRACK_PRE_DECEL_SPD 巡航; 压停车线直接断电 */
#define TRACK_PRE_DECEL_MS   17000u
#define TRACK_PRE_DECEL_SPD  12
#define TRACK_DECEL_STEP_TICKS 10u /* 预减速每档时长 (×10ms拍); 要更狠就调小 */

volatile int Target_Speed = TARGET_SPEED;

/* 模式：STOP=停止, RUN=完整循迹 */
#define MODE_STOP  0
#define MODE_RUN   1

static volatile int track_mode = MODE_STOP;

volatile int speed_L = 0, speed_R = 0;   /* 供屏幕 UI 显示 */
static volatile int PWMA = 0, PWMB = 0;
static int soft_speed = 0;
static unsigned soft_tick = 0;
static uint8_t soft_stop = 0;    /* 停车线触发缓停进行中 (仅行驶+控球任务) */
static uint8_t pre_decel = 0;    /* 1.Track 预减速进行中 (17s 后降巡航) */
/* 起步后1s内不判停车线: 无论车压在起始线上还是线后起步, 都不会误触发 */
#define STOPLINE_ARM_DELAY 100u  /* ×10ms拍 */
static uint8_t stopline_arm_ms = 0; /* 停车线武装倒计时 (10ms拍) */

static rt_device_t enc_l_dev = RT_NULL;
static rt_device_t enc_r_dev = RT_NULL;
static volatile uint8_t ball_en = 0;   /* 控球 PID 总开关, 默认关 */
volatile int32_t ptz_cur_angle = PTZ_ANGLE_MID;   /* PTZ 当前角度缓存(0.01°), 供 UI */
static uint8_t ptz_fail = 0;           /* 读角度连续失败计数 */
static uint8_t ptz_offline = 0;        /* 1=驱动器离线, 停止轮询 */

static void track_motor_stop(void)
{
    soft_speed = 0;
    soft_tick = 0;
    Turn_Speed = 0;
    PWMA = 0;
    PWMB = 0;
    Set_PWM(0, 0);
    PID_Reset();
}

/* ---- 行驶+控球: 行驶中动态调度控球阻尼 ----
 * 题4/5: 缓启/出弯 Kv×1.3 → 缓停/入弯 Kv×1.2 → 平稳×1.0, 边沿触发
 * 题6正向: 满增益压稳 400ms 后直接回 1.0 (不做中间斜坡);
 *          缓启/缓停本体叠杆角前馈(-1°/+0.5°), 结束后由 ball.c 再保持 400ms */
#define TURN_KV_TH      2      /* |Δturn| 滤波阈值, 过小易抖 */

static uint8_t drive_ball_on;  /* 本任务为行驶+控球 (T45/T6) */
static uint8_t soft_kv_phase;  /* 题4/5: 0无 1加速 2减速 (边沿) */
static uint8_t turn_kv_inited;
static int turn_abs_prev;
static int turn_d_filt;        /* |Turn| 差分一阶滤波: <=-TH 出弯, >=+TH 入弯 */
static uint8_t t6_kv_st;       /* 题6正向: 0空闲 1缓加 2缓加后压稳 3缓减 4缓减后压稳 */
static uint16_t t6_kv_hold_ms;
static float t6_kv_want_sent;
static int8_t t6_soft_ff_sent;

static void DriveBall_Reset(void)
{
    drive_ball_on = 0;
    soft_kv_phase = 0;
    turn_kv_inited = 0;
    turn_d_filt = 0;
    t6_kv_st = 0;
    t6_kv_hold_ms = 0;
    t6_kv_want_sent = 1.0f;
    t6_soft_ff_sent = 0;
    soft_stop = 0;
}

static void DriveBall_Begin(void)
{
    DriveBall_Reset();
    drive_ball_on = 1;
}

/* 行驶结束: 控球阻尼/前馈恢复正常, 球继续稳住 */
static void DriveBall_Restore(void)
{
    Ball_SetKvScale(1.0f);
    Ball_SetSoftFf(0);
}

/* 10ms 拍: 行驶中按缓启/缓停/转弯动态调度控球 Kv, 仅 drive_ball_on 且 MODE_RUN 时调用 */
static void DriveBall_KvTick(uint8_t soft_starting, uint8_t soft_stopping)
{
    int turn_abs = Turn_Speed;

    if (turn_abs < 0)
    {
        turn_abs = -turn_abs;
    }
    if (!turn_kv_inited)
    {
        turn_abs_prev = turn_abs;
        turn_d_filt = 0;
        turn_kv_inited = 1;
    }
    else
    {
        int d = turn_abs - turn_abs_prev;

        turn_abs_prev = turn_abs;
        turn_d_filt = (turn_d_filt * 3 + d) / 4;
    }

    if (Ball_IsT6Pos())
    {
        float want = 1.0f;
        int8_t want_ff = 0;

        if (soft_starting)
        {
            t6_kv_st = 1;
            t6_kv_hold_ms = 0;
            want = BALL_T6_KV_ACCEL;
        }
        else if (soft_stopping)
        {
            t6_kv_st = 3;            /* 路程末端缓停: 减速增益+缓停前馈, 随 mode_stop 复位 */
            t6_kv_hold_ms = 0;
            want = BALL_T6_KV_DECEL;
        }
        else if (t6_kv_st == 1u)
        {
            t6_kv_st = 2;            /* 刚退出缓加 → 满增益压稳 */
            t6_kv_hold_ms = 0;
            want = BALL_T6_KV_ACCEL;
        }
        else if (turn_d_filt <= -TURN_KV_TH)
        {
            t6_kv_st = 0;            /* 出弯≈加速 */
            t6_kv_hold_ms = 0;
            want = BALL_T6_KV_ACCEL;
        }
        else if (turn_d_filt >= TURN_KV_TH)
        {
            t6_kv_st = 0;            /* 入弯≈减速 */
            t6_kv_hold_ms = 0;
            want = BALL_T6_KV_DECEL;
        }
        else if (t6_kv_st == 2u || t6_kv_st == 4u)
        {
            t6_kv_hold_ms = (uint16_t)(t6_kv_hold_ms + CTRL_PERIOD_MS);
            if (t6_kv_hold_ms >= BALL_T6_SOFT_FF_HOLD_MS)
            {
                t6_kv_st = 0;
                t6_kv_hold_ms = 0;
            }
            else
            {
                want = (t6_kv_st == 2u) ? BALL_T6_KV_ACCEL : BALL_T6_KV_DECEL;
            }
        }
        if (t6_kv_st == 1u)
        {
            want_ff = -1;
        }
        else if (t6_kv_st == 3u)
        {
            want_ff = 1;
        }
        if (want_ff != t6_soft_ff_sent)
        {
            Ball_SetSoftFf(want_ff);
            t6_soft_ff_sent = want_ff;
        }
        if (want != t6_kv_want_sent)
        {
            Ball_SetKvScale(want);
            t6_kv_want_sent = want;
        }
    }
    else
    {
        uint8_t phase = 0u;

        if (soft_starting)
        {
            phase = 1u;
        }
        else if (soft_stopping)
        {
            phase = 2u;              /* 路程末端缓停: W 减速阻尼 */
        }
        else if (turn_d_filt <= -TURN_KV_TH)
        {
            phase = 1u;              /* 出弯≈加速 */
        }
        else if (turn_d_filt >= TURN_KV_TH)
        {
            phase = 2u;              /* 入弯≈减速 */
        }

        if (phase != soft_kv_phase)
        {
            if (phase == 1u)
            {
                Ball_SetKvScale(BALL_T6_KV_ACCEL);
            }
            else if (phase == 2u)
            {
                Ball_SetKvScale(BALL_T6_KV_DECEL);
            }
            else
            {
                Ball_SetKvScale(1.0f);
            }
            soft_kv_phase = phase;
        }
    }
}

/* 计时: 任务启动开始, 停止/丢线冻结; tick=1ms, 32位减法回绕安全 */
static rt_tick_t lap_t0;
static volatile uint8_t lap_timing;
static volatile uint32_t lap_ms_frozen;

static void Lap_Start(void)
{
    lap_t0 = rt_tick_get();
    lap_timing = 1;
}

static void Lap_Freeze(void)
{
    if (lap_timing)
    {
        lap_ms_frozen = (uint32_t)(rt_tick_get() - lap_t0);
        lap_timing = 0;
    }
}

uint32_t Lap_TimeMs(void)
{
    return lap_timing ? (uint32_t)(rt_tick_get() - lap_t0) : lap_ms_frozen;
}

static void mode_stop(void)
{
    track_mode = MODE_STOP;
    track_motor_stop();
    if (drive_ball_on)
    {
        DriveBall_Restore();   /* 行驶结束: 控球阻尼/前馈恢复正常 */
        DriveBall_Reset();
    }
    Lap_Freeze();
    LOG_I("motor stop");
}

static void track_start(void)
{
    Tracking_Reset();
    Tracking_ClearLineLost();
    track_motor_stop();
    soft_stop = 0;
    pre_decel = 0;
    stopline_arm_ms = STOPLINE_ARM_DELAY;
    track_mode = MODE_RUN;
    Lap_Start();
    LOG_I("tracking start");
}

/* 菜单任务命令: 单槽邮箱, UI 线程写/控制线程取, 关中断保护防丢命令 */
static volatile TaskCmd_t task_cmd = TASK_CMD_NONE;

void Task_PostCmd(TaskCmd_t cmd)
{
    rt_base_t level = rt_hw_interrupt_disable();

    task_cmd = cmd;
    rt_hw_interrupt_enable(level);
}

static TaskCmd_t Task_TakeCmd(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    TaskCmd_t cmd = task_cmd;

    task_cmd = TASK_CMD_NONE;
    rt_hw_interrupt_enable(level);
    return cmd;
}

/* 在控制线程内执行, 与 PTZ_ReadAngle/Ball_Control 的 Modbus 事务天然互斥 */
static void Task_CmdExec(TaskCmd_t cmd)
{
    switch (cmd)
    {
    case TASK_CMD_TRACK:
        ball_en = 0;
        Ball_Stop();   /* PTZ 回中 */
        Target_Speed = TRACK_SPD_FAST;
        Track_PID.Kp = TRACK_KP_FAST;
        track_start();
        soft_speed = Target_Speed;   /* 1.Track 不缓启, 起步即全速 */
        break;

    case TASK_CMD_BALL_T3:
        Target_Speed = TRACK_SPD_SLOW;
        Track_PID.Kp = TRACK_KP_SLOW;
        mode_stop();
        ball_en = 1;
        Ball_Start();
        Lap_Start();
        break;

    case TASK_CMD_BALL_T45:
        Target_Speed = TRACK_SPD_SLOW;
        Track_PID.Kp = TRACK_KP_SLOW;
        mode_stop();
        ball_en = 1;
        Ball_StartHold();
        track_start();       /* 循迹行驶同时稳中点 */
        DriveBall_Begin();
        break;

    case TASK_CMD_BALL_T6:
    {
        int16_t px = K230_GetPos();

        if (px <= 0 || px > 999)
        {
            px = (int16_t)PID_SETPOINT;   /* 无有效视觉位置则稳中点 */
        }
        Target_Speed = TRACK_SPD_SLOW;
        Track_PID.Kp = TRACK_KP_SLOW;
        mode_stop();
        ball_en = 1;
        Ball_StartHoldAt(px);
        track_start();       /* 循迹行驶同时稳指定位置 */
        DriveBall_Begin();
        break;
    }

    case TASK_CMD_STOP_ALL:
        mode_stop();
        ball_en = 0;
        Ball_Stop();
        break;

    default:
        break;
    }
}

/* 灰度线程：独占采样与循迹误差计算，控制环只消费单字原子标志
 * (由 main.c 创建启动) */
void Gray_ThreadEntry(void *parameter)
{
    while (1)
    {
        Gray_Value_Acquire();
        StopLine_Update();   /* 停车线判定 (≥4路黑+防抖), 行驶+控球缓停触发用 */
        Tracking_Get_Error();
        Tracking_PID_Compute();
        rt_thread_mdelay(GRAY_PERIOD_MS);
    }
}

void Track_CtrlEntry(void *parameter)
{
    rt_int32_t last_l = 0, last_r = 0;
    rt_tick_t next_wake = rt_tick_get();

    while (1)
    {
        rt_int32_t cl = 0, cr = 0;
        int base, turn, turn_lim;
        int target_L, target_R;
        int dl, dr;
        uint8_t soft_starting = 0;
        static uint8_t ang_tick = 0;

        /* 编码器增量采样; 假尖峰丢弃并沿用上拍速度, 参考值照常更新保证后续差分正确 */
        rt_device_read(enc_l_dev, 0, &cl, 1);
        rt_device_read(enc_r_dev, 0, &cr, 1);
        dl = ENC_L_SIGN * (int)(cl - last_l);
        dr = ENC_R_SIGN * (int)(cr - last_r);
        last_l = cl;
        last_r = cr;
        if (dl <= ENC_DELTA_MAX && dl >= -ENC_DELTA_MAX) speed_L = dl;
        if (dr <= ENC_DELTA_MAX && dr >= -ENC_DELTA_MAX) speed_R = dr;

        Key_Scan_10ms();
        Task_CmdExec(Task_TakeCmd());

        /* 控球 PID: K230 新帧驱动, PTZ 帧内已限幅钳制; ball_on/ball_off 开关
         * 行驶+控球时按缓启/转弯动态调度控球 Kv */
        if (ball_en)
        {
            Ball_Control();
            Ball_SoftFfTick(CTRL_PERIOD_MS);
            if (drive_ball_on && track_mode == MODE_RUN)
            {
                DriveBall_KvTick((!soft_stop && soft_speed < Target_Speed) ? 1u : 0u,
                                 soft_stop);
            }
        }

        /* 每 100ms 读一次 PTZ 实际角度, 缓存给 UI (Modbus 事务在本线程内, 无并发)
         * 连续 3 次读失败判离线, 停止轮询防止 100ms 超时拖垮控制环 */
        if (!ptz_offline && ++ang_tick >= 10)
        {
            int32_t a;
            ang_tick = 0;
            if (PTZ_ReadAngle(&a))
            {
                ptz_cur_angle = a;
                ptz_fail = 0;
            }
            else if (++ptz_fail >= 3)
            {
                ptz_offline = 1;
                LOG_W("ptz offline, stop polling angle");
            }
        }

        if (track_mode == MODE_RUN)
        {
            if (soft_stop)
            {
                /* 路程末端缓停: 每300ms降1档到0, 降完停车收尾; 丢线不再判 */
                if (soft_speed > 0)
                {
                    soft_tick++;
                    if (soft_tick >= SOFT_STEP_TICKS)
                    {
                        soft_tick = 0;
                        soft_speed--;
                    }
                }
                if (soft_speed <= 0)
                {
                    soft_speed = 0;
                    LOG_I("soft stop done");
                    mode_stop();
                    continue;
                }
            }
            else
            {
                /* 缓启动 */
                if (soft_speed < Target_Speed)
                {
                    soft_starting = 1;
                    if (soft_speed < SOFT_START_INIT)
                    {
                        soft_speed = SOFT_START_INIT;
                        soft_tick = 0;
                    }
                    else
                    {
                        soft_tick++;
                        if (soft_tick >= SOFT_STEP_TICKS)
                        {
                            soft_tick = 0;
                            soft_speed++;
                            if (soft_speed > Target_Speed) soft_speed = Target_Speed;
                        }
                    }
                }
                else if (soft_speed > Target_Speed)
                {
                    /* 1.Track 预减速: 逐档降到预减速巡航速, 节奏独立于缓启 */
                    soft_tick++;
                    if (soft_tick >= TRACK_DECEL_STEP_TICKS)
                    {
                        soft_tick = 0;
                        soft_speed--;
                    }
                }
                else
                {
                    soft_speed = Target_Speed;
                }

                /* 1.Track 预减速触发: lap 到 18s 后目标降档巡航 */
                if (!drive_ball_on && !pre_decel &&
                    Lap_TimeMs() >= TRACK_PRE_DECEL_MS)
                {
                    pre_decel = 1;
                    Target_Speed = TRACK_PRE_DECEL_SPD;
                    LOG_I("pre decel, target=%d", (int)Target_Speed);
                }

                /* 停车线 (起步1s内不判): 行驶+控球 → 缓停; 1.Track → 目标归零刹停 */
                if (stopline_arm_ms > 0u)
                {
                    stopline_arm_ms--;
                }
                else if (StopLine_IsPresent())
                {
                    if (drive_ball_on)
                    {
                        Lap_Freeze();        /* 成绩止于停车线 */
                        soft_stop = 1;       /* 下拍起从当前速度降档 */
                        soft_tick = 0;
                        LOG_I("stop line, soft stop");
                    }
                    else
                    {
                        LOG_I("stop line, stop");
                        mode_stop();         /* 直接断电, 计时同步冻结 */
                        continue;
                    }
                }

                /* 丢线停车（缓启动期间不判） */
                if (!soft_starting && Tracking_LineLost)
                {
                    LOG_W("line lost, stop");
                    mode_stop();
                    continue;
                }
            }

            base = soft_speed;
            if (soft_starting)
            {
                Turn_Speed = 0;
                turn = 0;
            }
            else
            {
                turn = Turn_Speed;
                turn_lim = (int)(base * TURN_LIMIT_RATIO);
                if (turn_lim < 0) turn_lim = 0;
                if (turn > turn_lim) turn = turn_lim;
                else if (turn < -turn_lim) turn = -turn_lim;
            }

            target_L = base;
            target_R = base;
            if (turn > 0) target_R = base - turn;
            else if (turn < 0) target_L = base + turn;

            /* 轮速PI增益: 纯循迹/行驶+控球各用一套标称值 (motor.h 独立整定);
             * 缓启/缓停段(基速<目标)再按基速比例衰减——低速目标只有1~3个
             * 脉冲, 反馈量化在0/1间跳, 标称Kp下每拍±数百导致车轮极限环抖动 */
            {
                int kp_nom = drive_ball_on ? VEL_KP_BALL : VEL_KP_NOM;
                int ki_nom = drive_ball_on ? VEL_KI_BALL : VEL_KI_NOM;

                if (base < Target_Speed)
                {
                    int s = (base > 0) ? base : 1;

                    PID_A.Kp = kp_nom * s / Target_Speed;
                    PID_A.Ki = ki_nom * s / Target_Speed;
                }
                else
                {
                    PID_A.Kp = kp_nom;
                    PID_A.Ki = ki_nom;
                }
                PID_B.Kp = PID_A.Kp;
                PID_B.Ki = PID_A.Ki;
            }

            PWMB = Velocity_B(target_L, speed_L);
            PWMA = Velocity_A(target_R, speed_R);
            Set_PWM(PWMA, PWMB);
        }
        else
        {
            track_motor_stop();
        }

        /* 严格 10ms 周期：补偿本拍环内耗时，超期则重新对齐 */
        next_wake += CTRL_PERIOD_MS;
        if (next_wake > rt_tick_get())
            rt_thread_mdelay(next_wake - rt_tick_get());
        else
            next_wake = rt_tick_get();
    }
}

/* 外设初始化: 编码器/电机/灰度/云台/控球; 线程由 main.c 创建启动 */
int Track_Init(void)
{
    enc_l_dev = rt_device_find(ENC_L_DEV);
    enc_r_dev = rt_device_find(ENC_R_DEV);
    if (enc_l_dev == RT_NULL || enc_r_dev == RT_NULL)
    {
        LOG_E("find %s/%s failed", ENC_L_DEV, ENC_R_DEV);
        return -RT_ERROR;
    }
    if (rt_device_open(enc_l_dev, RT_DEVICE_OFLAG_RDONLY) != RT_EOK ||
        rt_device_open(enc_r_dev, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        LOG_E("encoder open failed");
        return -RT_ERROR;
    }

    Motor_Init();
    Gray_Init();

    /* PTZ 云台: 打开 uart3 + 仅进闭环 */
    if (PTZ_Init() == RT_EOK)
    {
        PTZ_CloseLoop();
    }
    Ball_Init();

    return RT_EOK;
}

/* ---- msh 测试命令 ---- */
static void track_start_cmd(int argc, char **argv)
{
    track_start();
}
MSH_CMD_EXPORT(track_start_cmd, start tracking);

static void track_stop_cmd(int argc, char **argv)
{
    mode_stop();
}
MSH_CMD_EXPORT(track_stop_cmd, stop motor);

static void track_stat_cmd(int argc, char **argv)
{
    int i;

    rt_kprintf("mode:%d ball_en:%d spd L/R:%d/%d PWM A/B:%d/%d\n",
               track_mode, ball_en, speed_L, speed_R, PWMA, PWMB);
    rt_kprintf("err:%d turn:%d lost:%d stopline:%d\n",
               Tracking_Error, Turn_Speed, Tracking_LineLost, StopLine_IsPresent());
    rt_kprintf("ball pos:%d set:%d err:%d tgt:%d\n",
               Ball_GetPos(), Ball_GetSetpoint(), Ball_GetErr(), (int)Ball_GetTarget());
    for (i = 0; i < 8; i++)
    {
        rt_kprintf("G%d:%3d ", i, Gray_Value[i]);
    }
    rt_kprintf("\n");
}
MSH_CMD_EXPORT(track_stat_cmd, show tracking status);

static void ball_on_cmd(int argc, char **argv)
{
    if (argc >= 2)
    {
        Ball_StartHoldAt((int16_t)atoi(argv[1]));   /* 指定像素目标 */
    }
    else
    {
        Ball_StartHold();   /* 默认稳中点 426 */
    }
    ball_en = 1;
    LOG_I("ball pid on");
}
MSH_CMD_EXPORT(ball_on_cmd, ball PID on [setpoint_px]);

static void ball_off_cmd(int argc, char **argv)
{
    ball_en = 0;
    Ball_Stop();   /* 回中位 212.50° */
    LOG_I("ball pid off");
}
MSH_CMD_EXPORT(ball_off_cmd, ball PID off and PTZ to mid);

static void enc_test_cmd(int argc, char **argv)
{
    rt_int32_t cl = 0, cr = 0;

    rt_device_read(enc_l_dev, 0, &cl, 1);
    rt_device_read(enc_r_dev, 0, &cr, 1);
    rt_kprintf("enc L(pulse3):%d R(pulse4):%d\n", cl, cr);
}
MSH_CMD_EXPORT(enc_test_cmd, show encoder raw count);
