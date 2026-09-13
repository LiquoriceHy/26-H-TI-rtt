/*
 * 四向按键扫描: PC0=左 PC4=右 PC5=上 PC1=下, 低电平有效
 * 控制线程 10ms 节拍消抖, 按下沿置事件位, 由 UI 线程取走
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "key.h"

#define KEY_PIN_L   GET_PIN(C, 0)
#define KEY_PIN_R   GET_PIN(C, 4)
#define KEY_PIN_U   GET_PIN(C, 5)
#define KEY_PIN_D   GET_PIN(C, 1)

#define KEY_DEBOUNCE 2u

static const rt_base_t key_pins[4] = {KEY_PIN_L, KEY_PIN_R, KEY_PIN_U, KEY_PIN_D};
static const uint8_t key_bits[4]  = {KEY_BIT_LEFT, KEY_BIT_RIGHT, KEY_BIT_UP, KEY_BIT_DOWN};

static volatile uint8_t s_events;
static uint8_t steady[4];   /* 1=释放(上拉常态) */
static uint8_t cnt[4];

int Key_Init(void)
{
    int i;

    for (i = 0; i < 4; i++)
    {
        rt_pin_mode(key_pins[i], PIN_MODE_INPUT_PULLUP);
        steady[i] = 1;
        cnt[i] = 0;
    }
    return RT_EOK;
}
INIT_APP_EXPORT(Key_Init);

void Key_Scan_10ms(void)
{
    int i;

    for (i = 0; i < 4; i++)
    {
        uint8_t down = (rt_pin_read(key_pins[i]) == PIN_LOW) ? 1u : 0u;

        if (down != steady[i])
        {
            if (++cnt[i] >= KEY_DEBOUNCE)
            {
                if (down)
                {
                    s_events |= key_bits[i];   /* 按下沿产生事件 */
                }
                steady[i] = down;
                cnt[i] = 0;
            }
        }
        else
        {
            cnt[i] = 0;
        }
    }
}

uint8_t Key_TakeEvents(void)
{
    rt_base_t level;
    uint8_t e;

    level = rt_hw_interrupt_disable();
    e = s_events;
    s_events = 0;
    rt_hw_interrupt_enable(level);
    return e;
}
