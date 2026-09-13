#ifndef _PTZ_H_
#define _PTZ_H_
#include <rtthread.h>

/* JC 系列驱动器 Modbus RTU 电机 ID */
#define PTZ_MOTOR_ID        0x01

/* 位置单位: 0.01° (角度放大 100 倍, 36000 = 360°) */
#define PTZ_DEG(x)          ((int32_t)((x) * 100))

/* 机械限位 (0.01°, 超出会损坏机械, 实测值) —— 任何下发角度都必须经过 PTZ_ClampAngle */
#define PTZ_ANGLE_MIN       19950   /* 下限 199.50° */
#define PTZ_ANGLE_MAX       22550   /* 上限 225.50° */
#define PTZ_ANGLE_MID       21250   /* 中位 212.50° = 真正水平 */

/* PTZ 串口: USART3 (PD8=TX, PD9=RX), 115200, Modbus RTU */

/* 打开 uart3 (需先在 RT-Thread Settings 使能 UART3), 成功返回 RT_EOK */
int PTZ_Init(void);

/* 进入闭环 (写 0x00A2=1), 上电后必须先调用一次 */
void PTZ_CloseLoop(void);

/* 限位钳制: ang 任意 int32, 返回 [PTZ_ANGLE_MIN, PTZ_ANGLE_MAX] */
int32_t PTZ_ClampAngle(int32_t ang);

/* 绝对位置: ang 单位 0.01° (用 PTZ_DEG 换算), 自动钳制到机械限位 */
void PTZ_SetAbsAngle(int32_t ang);

/* 读取当前角度: *angle 单位 0.01°, 成功返回1, 失败(超时/CRC)返回0 */
uint8_t PTZ_ReadAngle(int32_t *angle);

#endif  /* _PTZ_H_ */
