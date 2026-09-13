#ifndef _GRAY_MODULE_H_
#define _GRAY_MODULE_H_
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* 灰度模拟开关地址选通引脚（分配表） */
#define GRAY_S0_PIN   GET_PIN(G, 0)
#define GRAY_S1_PIN   GET_PIN(G, 2)
#define GRAY_S2_PIN   GET_PIN(G, 4)

#define GRAY_S0_HIGH  rt_pin_write(GRAY_S0_PIN, PIN_HIGH)
#define GRAY_S0_LOW   rt_pin_write(GRAY_S0_PIN, PIN_LOW)
#define GRAY_S1_HIGH  rt_pin_write(GRAY_S1_PIN, PIN_HIGH)
#define GRAY_S1_LOW   rt_pin_write(GRAY_S1_PIN, PIN_LOW)
#define GRAY_S2_HIGH  rt_pin_write(GRAY_S2_PIN, PIN_HIGH)
#define GRAY_S2_LOW   rt_pin_write(GRAY_S2_PIN, PIN_LOW)

/* 8 路灰度值 (0~255) */
extern unsigned char Gray_Value[8];

void Gray_Init(void);
void Gray_Value_Acquire(void);

#endif /* _GRAY_MODULE_H_ */
