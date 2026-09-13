#ifndef _COMM_H_
#define _COMM_H_
#include <rtthread.h>

/* K230 视觉模块（USART2: PA2=TX, PA3=RX, 115200）
 * 协议：ASCII 文本行，每行一个坐标数值，'\n' 结尾 */

/* 打开 uart2 并启动行解析（INIT_APP_EXPORT 自动调用） */
int K230_Init(void);

/* 最新解析出的坐标 */
int16_t K230_GetPos(void);

/* 新帧标志，读后清零；返回 1 表示有新数据 */
uint8_t K230_GetReady(void);

#endif /* _COMM_H_ */
