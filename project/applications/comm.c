/*
 * K230 视觉数据接收
 * RT-Thread 串口设备框架：rx_indicate 回调里逐字节读，行解析后存坐标
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "comm.h"

#define DBG_TAG "k230"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define K230_DEV_NAME   "uart2"
#define K230_LINE_MAX   16

static rt_device_t k230_dev = RT_NULL;
static char s_line[K230_LINE_MAX];
static rt_uint8_t s_line_len;
static volatile int16_t s_pos;
static volatile uint8_t s_ready;

/* 行解析：ASCII 数值行，'\n'/'\r' */
static void k230_parse_byte(char c)
{
    if (c == '\n' || c == '\r')
    {
        if (s_line_len > 0)
        {
            char tmp[K230_LINE_MAX];

            rt_memcpy(tmp, (const void *)s_line, s_line_len);
            tmp[s_line_len] = '\0';
            s_pos = (int16_t)atoi(tmp);
            s_ready = 1;
            s_line_len = 0;
        }
    }
    else if (s_line_len < K230_LINE_MAX - 1)
    {
        s_line[s_line_len++] = c;
    }
    else
    {
        s_line_len = 0;   /* 行超长，丢弃重同步 */
    }
}

/* 串口接收回调（中断上下文）：读走本次到达的字节并解析 */
static rt_err_t k230_rx_cb(rt_device_t dev, rt_size_t size)
{
    char ch;

    while (size-- && rt_device_read(dev, 0, &ch, 1) == 1)
    {
        k230_parse_byte(ch);
    }
    return RT_EOK;
}

int16_t K230_GetPos(void)
{
    return s_pos;
}

uint8_t K230_GetReady(void)
{
    uint8_t r = s_ready;

    s_ready = 0;
    return r;
}

int K230_Init(void)
{
    k230_dev = rt_device_find(K230_DEV_NAME);
    if (k230_dev == RT_NULL)
    {
        LOG_E("find %s failed", K230_DEV_NAME);
        return -RT_ERROR;
    }
    if (rt_device_open(k230_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open %s failed", K230_DEV_NAME);
        return -RT_ERROR;
    }
    rt_device_set_rx_indicate(k230_dev, k230_rx_cb);
    LOG_I("k230 ready on %s", K230_DEV_NAME);

    return RT_EOK;
}
INIT_APP_EXPORT(K230_Init);

/* msh: 查看最新坐标（不清新帧标志） */
static void k230_stat_cmd(int argc, char **argv)
{
    rt_kprintf("pos:%d line_len:%d\n", s_pos, s_line_len);
}
MSH_CMD_EXPORT(k230_stat_cmd, show K230 position);
