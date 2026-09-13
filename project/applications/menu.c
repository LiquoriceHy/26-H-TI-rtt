/*
 * 任务选择区: 嵌在主界面下方空白, 上/下切换任务, 左键启动, 右键全停
 * 按键在 10ms 控制线程扫描, 本模块在 LVGL 定时器里取事件
 * 启动/停止经 Task_PostCmd 投递给控制线程执行 (PTZ Modbus 事务不跨线程)
 */
#include <rtthread.h>
#include <lvgl.h>
#include "key.h"
#include "menu.h"
#include "tracking.h"

#define MENU_PERIOD_MS 50

typedef struct {
    const char *name;
    TaskCmd_t   cmd;
} MenuItem_t;

static const MenuItem_t s_items[MENU_N] = {
    { "1.Track",    TASK_CMD_TRACK },
    { "2.Bal +-5",  TASK_CMD_BALL_T3 },
    { "3.Drv Mid",  TASK_CMD_BALL_T45 },
    { "4.Drv Here", TASK_CMD_BALL_T6 },
};

static lv_obj_t *item_labels[MENU_N];
static lv_obj_t *hint_label;
static uint8_t sel;

static void menu_refresh(void)
{
    uint8_t i;

    for (i = 0; i < MENU_N; i++)
    {
        lv_label_set_text_fmt(item_labels[i], "%c%s",
                              (i == sel) ? '>' : ' ', s_items[i].name);
        lv_obj_set_style_text_color(item_labels[i],
            (i == sel) ? lv_color_hex(0xFFFF00) : lv_color_white(), 0);
    }
}

static void menu_key_handle(uint8_t ev)
{
    if (ev & KEY_BIT_UP)
    {
        sel = (uint8_t)((sel + MENU_N - 1) % MENU_N);
        menu_refresh();
    }
    if (ev & KEY_BIT_DOWN)
    {
        sel = (uint8_t)((sel + 1) % MENU_N);
        menu_refresh();
    }
    if (ev & KEY_BIT_LEFT)
    {
        Task_PostCmd(s_items[sel].cmd);
    }
    if (ev & KEY_BIT_RIGHT)
    {
        Task_PostCmd(TASK_CMD_STOP_ALL);
    }
}

static void menu_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    menu_key_handle(Key_TakeEvents());
}

void Menu_Create(lv_obj_t *parent, lv_coord_t x, lv_coord_t y)
{
    uint8_t i;

    for (i = 0; i < MENU_N; i++)
    {
        item_labels[i] = lv_label_create(parent);
        lv_obj_set_pos(item_labels[i], x, y + i * MENU_ROW_H);
    }

    hint_label = lv_label_create(parent);
    lv_label_set_text(hint_label, "L:go  R:stop");
    lv_obj_set_style_text_color(hint_label, lv_color_hex(0x808080), 0);
    lv_obj_set_pos(hint_label, x + 96, y + (MENU_N - 1) * MENU_ROW_H);

    menu_refresh();
    lv_timer_create(menu_timer_cb, MENU_PERIOD_MS, NULL);
}
