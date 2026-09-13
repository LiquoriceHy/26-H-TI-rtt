/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-5-10      ShiHao       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "tracking.h"

#define DBG_TAG "main"
#define DBG_LVL         DBG_LOG
#include <rtdbg.h>

/* 板载 WS2812B 灯珠阵列电源使能脚：拉低使能，拉高关断 */
#define LED_MATRIX_EN_PIN      GET_PIN(F, 2)

/*
 * 业务线程总览 (创建/启动集中在此, 便于通览):
 *   track  优先级10  10ms控制环: 编码器/按键/任务/混控/轮速PI/控球联动
 *   gray   优先级15  灰度采样+循迹误差+转向PID+停车线
 *   LVGL   优先级21  刷屏+UI, 由 LVGL 包自建 (packages/.../lv_rt_thread_port.c)
 */
int main(void)
{
    rt_thread_t tid;

    rt_pin_mode(LED_MATRIX_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_MATRIX_EN_PIN, PIN_HIGH);

    if (Track_Init() != RT_EOK)
    {
        LOG_E("track init failed");
        return -1;
    }

    tid = rt_thread_create("track", Track_CtrlEntry, RT_NULL,
                           2048, 10, 10);
    if (tid == RT_NULL)
    {
        LOG_E("create track thread failed");
        return -1;
    }
    rt_thread_startup(tid);

    tid = rt_thread_create("gray", Gray_ThreadEntry, RT_NULL,
                           1024, 15, 10);
    if (tid == RT_NULL)
    {
        LOG_E("create gray thread failed");
        return -1;
    }
    rt_thread_startup(tid);

    return 0;
}

