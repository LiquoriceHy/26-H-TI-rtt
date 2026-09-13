#include "gray_module.h"
#include <drivers/adc.h>

#define GRAY_ADC_DEV_NAME  "adc1"
#define GRAY_ADC_CHANNEL   1     /* PA1 = ADC1_IN1 */

unsigned char Gray_Value[8] = {0};
static rt_adc_device_t gray_adc_dev = RT_NULL;

void Gray_Init(void)
{
    if (gray_adc_dev != RT_NULL)   /* 已初始化，避免线程与 msh 命令重复打开 */
    {
        return;
    }

    rt_pin_mode(GRAY_S0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(GRAY_S1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(GRAY_S2_PIN, PIN_MODE_OUTPUT);

    gray_adc_dev = (rt_adc_device_t)rt_device_find(GRAY_ADC_DEV_NAME);
    if (gray_adc_dev == RT_NULL)
    {
        rt_kprintf("gray: find %s failed!\n", GRAY_ADC_DEV_NAME);
        return;
    }
    rt_adc_enable(gray_adc_dev, GRAY_ADC_CHANNEL);
}

static unsigned int gray_adc_once(void)
{
    if (gray_adc_dev == RT_NULL)
    {
        return 0;
    }
    return rt_adc_read(gray_adc_dev, GRAY_ADC_CHANNEL);
}

void Gray_Value_Acquire(void)
{
    if (gray_adc_dev == RT_NULL)
    {
        return;
    }

    for (unsigned char ch = 0; ch < 8; ch++)
    {
        if (ch & 0x04) GRAY_S2_HIGH; else GRAY_S2_LOW;
        if (ch & 0x02) GRAY_S1_HIGH; else GRAY_S1_LOW;
        if (ch & 0x01) GRAY_S0_HIGH; else GRAY_S0_LOW;

        /* 模拟开关切换稳定延时 */
        for (volatile int d = 0; d < 10; d++) {}

        unsigned int sum = 0;
        for (unsigned char i = 0; i < 5; i++)
        {
            unsigned int raw = gray_adc_once();
            if (i >= 3) sum += raw;   /* 抛弃前 3 次，累加后 2 次 */
        }
        unsigned int scaled = sum >> 5;   /* 2*4095>>5 ≈ 255 */
        Gray_Value[ch] = (scaled > 255) ? 255 : (unsigned char)scaled;
    }
}

/* msh 测试：打印 8 路灰度值 */
static void gray_test(int argc, char **argv)
{
    Gray_Init();
    Gray_Value_Acquire();
    for (int i = 0; i < 8; i++)
    {
        rt_kprintf("CH%d: %3d\n", i, Gray_Value[i]);
    }
}
MSH_CMD_EXPORT(gray_test, gray module test: print 8 channel values);
