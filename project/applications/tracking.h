#ifndef _TRACKING_H_
#define _TRACKING_H_
#include <rtthread.h>

extern volatile int Target_Speed;
extern volatile int speed_L;
extern volatile int speed_R;
extern volatile int32_t ptz_cur_angle;   /* PTZ 当前角度缓存 (0.01°) */

/* 菜单任务命令: UI 线程投递, 控制线程内执行（PTZ Modbus 事务不跨线程并发） */
typedef enum {
    TASK_CMD_NONE = 0,
    TASK_CMD_TRACK,      /* 循迹 (纯循迹, 控球关) */
    TASK_CMD_BALL_T3,    /* 控球: 题3 ±5 往返 (车不动) */
    TASK_CMD_BALL_T45,   /* 行驶+控球: 循迹行驶同时稳中点 426 */
    TASK_CMD_BALL_T6,    /* 行驶+控球: 循迹行驶同时稳当前视觉位置 */
    TASK_CMD_STOP_ALL,   /* 全停: 电机停 + 控球关 + PTZ 回中 */
} TaskCmd_t;

void Task_PostCmd(TaskCmd_t cmd);

/* 业务线程: 外设初始化与两个线程入口, 由 main.c 统一创建启动 */
int Track_Init(void);
void Track_CtrlEntry(void *parameter);   /* 10ms 控制环 */
void Gray_ThreadEntry(void *parameter);  /* 灰度采样+循迹误差 */

/* 计时: 任务启动开始, 停止/丢线冻结; tick=1ms */
uint32_t Lap_TimeMs(void);   /* 计时中返回已走时间, 停止后返回冻结成绩 */

#endif /* _TRACKING_H_ */
