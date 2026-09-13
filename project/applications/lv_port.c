/*
 * LVGL port for onboard 240x240 LCD (RGB565)
 */
#include <rtthread.h>
#include <lvgl.h>
#include "drv_lcd.h"
#include "gray_module.h"
#include "tracking.h"
#include "dir.h"
#include "ball.h"
#include "menu.h"

#define DISP_HOR_RES 240
#define DISP_VER_RES 240

/* Flush LVGL draw buffer to LCD */
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    lcd_fill_array(area->x1, area->y1, area->x2, area->y2, color_p);
    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf[DISP_HOR_RES * 10];           /* 10 行绘图缓冲 */
    static lv_disp_drv_t disp_drv;

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, DISP_HOR_RES * 10);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
}

/* LVGL 移植线程 (lv_rt_thread_port.c) 固定调用, 不可删; 本板无触摸屏, 留空 */
void lv_port_indev_init(void)
{
}

/* 用户 GUI：左上=目标/编码器速度，右上=灰度数值(超阈值变绿)+阈值，
 * 中=球S-P-e/PTZ，底部=任务选择+计时（menu.c: 上/下切换，左启动，右全停） */
#define GRAY_CH_NUM   8
#define UI_REFRESH_MS 50

static lv_obj_t *spd_target_label;
static lv_obj_t *spd_l_label;
static lv_obj_t *spd_r_label;
static lv_obj_t *gray_labels[GRAY_CH_NUM];
static lv_obj_t *th_label;
static lv_obj_t *ball_label;
static lv_obj_t *ptz_label;
static lv_obj_t *time_label;

/* 运行在 LVGL 线程的定时器回调里，读取采集线程写入的 Gray_Value */
static void ui_refresh_cb(lv_timer_t *timer)
{
    lv_label_set_text_fmt(spd_target_label, "T:%d", Target_Speed);
    lv_label_set_text_fmt(spd_l_label, "L:%d", speed_L);
    lv_label_set_text_fmt(spd_r_label, "R:%d", speed_R);

    /* S=设定目标 P=几何校正后位置 e=误差 */
    lv_label_set_text_fmt(ball_label, "S:%d P:%d e:%d",
                          (int)Ball_GetSetpoint(), (int)Ball_GetPos(),
                          (int)Ball_GetErr());
    {
        int32_t t = Ball_GetTarget();
        int32_t c = (int32_t)ptz_cur_angle;
        lv_label_set_text_fmt(ptz_label, "PTZ %d.%02d/%d.%02d",
                              (int)(t / 100), (int)(t % 100),
                              (int)(c / 100), (int)(c % 100));
    }

    /* 计时: 走动中实时, 停止后冻结 */
    {
        uint32_t ms = Lap_TimeMs();
        lv_label_set_text_fmt(time_label, "T:%lu.%02lus",
                              (unsigned long)(ms / 1000u),
                              (unsigned long)((ms % 1000u) / 10u));
    }

    for (int i = 0; i < GRAY_CH_NUM; i++)
    {
        lv_label_set_text_fmt(gray_labels[i], "%3d", Gray_Value[i]);
        if (Gray_Value[i] > GRAY_THRESHOLD)
            lv_obj_set_style_text_color(gray_labels[i], lv_color_hex(0x00FF00), 0);
        else
            lv_obj_set_style_text_color(gray_labels[i], lv_color_white(), 0);
    }
}

/* 分区容器：透明底 + 边框 */
#define UI_PANEL_W 120
#define UI_PANEL_H 64

static lv_obj_t *ui_panel_create(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                                 lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x606060), 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_radius(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    return panel;
}

void lv_user_gui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    /* 左上 1/4：速度（目标 / 编码器 L R），三行；右边框=中缝、下边框=与K230面板分隔 */
    lv_obj_t *panel_l = ui_panel_create(scr, 0, 0, UI_PANEL_W, UI_PANEL_H);
    lv_obj_set_style_border_side(panel_l,
        LV_BORDER_SIDE_RIGHT | LV_BORDER_SIDE_BOTTOM, 0);

    spd_target_label = lv_label_create(panel_l);
    lv_label_set_text(spd_target_label, "T:0");
    lv_obj_set_style_text_color(spd_target_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_pos(spd_target_label, 4, 2);

    spd_l_label = lv_label_create(panel_l);
    lv_label_set_text(spd_l_label, "L:0");
    lv_obj_set_style_text_color(spd_l_label, lv_color_white(), 0);
    lv_obj_set_pos(spd_l_label, 4, 24);

    spd_r_label = lv_label_create(panel_l);
    lv_label_set_text(spd_r_label, "R:0");
    lv_obj_set_style_text_color(spd_r_label, lv_color_white(), 0);
    lv_obj_set_pos(spd_r_label, 4, 46);

    /* 右上：8 路灰度数值 4x2，第三行 TH */
    lv_obj_t *panel_r = ui_panel_create(scr, UI_PANEL_W, 0, UI_PANEL_W, UI_PANEL_H);
    lv_obj_set_style_border_side(panel_r, LV_BORDER_SIDE_BOTTOM, 0);

    for (int i = 0; i < GRAY_CH_NUM; i++)
    {
        int col = i % 4;
        int row = i / 4;

        gray_labels[i] = lv_label_create(panel_r);
        lv_label_set_text_fmt(gray_labels[i], "%3d", 0);
        lv_obj_set_style_text_color(gray_labels[i], lv_color_white(), 0);
        lv_obj_set_pos(gray_labels[i], 2 + col * 29, 2 + row * 22);
    }

    th_label = lv_label_create(panel_r);
    lv_label_set_text_fmt(th_label, "TH:%d", GRAY_THRESHOLD);
    lv_obj_set_style_text_color(th_label, lv_color_hex(0x808080), 0);
    lv_obj_set_pos(th_label, 2, 46);

    // 中部：球 S/P/e + PTZ 目标/当前角度，y=64 与上面板对齐，行距 22 同顶栏；

    lv_obj_t *panel_k = ui_panel_create(scr, 0, 64, 240, 46);
    lv_obj_set_style_border_side(panel_k, LV_BORDER_SIDE_BOTTOM, 0);

    ball_label = lv_label_create(panel_k);
    lv_label_set_text(ball_label, "S:0 P:0 e:0");
    lv_obj_set_style_text_color(ball_label, lv_color_hex(0x80FF80), 0);
    lv_obj_set_pos(ball_label, 4, 2);

    ptz_label = lv_label_create(panel_k);
    lv_label_set_text(ptz_label, "PTZ 0.00/0.00");
    lv_obj_set_style_text_color(ptz_label, lv_color_hex(0xFF80FF), 0);
    lv_obj_set_pos(ptz_label, 4, 24);

    /* 计时行: 任务列表下方 (y=198 行文字到 214, 218 起) */
    time_label = lv_label_create(scr);
    lv_label_set_text(time_label, "T:0.00s");
    lv_obj_set_style_text_color(time_label, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_pos(time_label, 4, 218);

    lv_timer_create(ui_refresh_cb, UI_REFRESH_MS, NULL);

    /* 下方空白区: 任务选择 (y=120 起, 4 行 x 26px, 至约 y=214) */
    Menu_Create(scr, 4, 120);
}
