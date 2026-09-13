#ifndef _KEY_H_
#define _KEY_H_
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* 四向按键: PC0=左 PC4=右 PC5=上 PC1=下, 全部低电平有效 */

typedef enum {
    KEY_BIT_LEFT  = 1u << 0,
    KEY_BIT_RIGHT = 1u << 1,
    KEY_BIT_UP    = 1u << 2,
    KEY_BIT_DOWN  = 1u << 3,
} KeyBit_t;

/* 引脚初始化  */
int Key_Init(void);

/* 每 10ms 控制节拍调用一次 */
void Key_Scan_10ms(void);

/* 取走挂起的按键事件位图（按下沿产生），读后清零 */
uint8_t Key_TakeEvents(void);

#endif /* _KEY_H_ */
