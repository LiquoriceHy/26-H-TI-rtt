#ifndef _MENU_H_
#define _MENU_H_
#include <lvgl.h>

/* 任务选择区行数与行高 (lv_port 布局用) */
#define MENU_N      4
#define MENU_ROW_H  26

/* 在主屏 (x,y) 处创建任务选择区: 上/下切换, 左键启动, 右键全停 */
void Menu_Create(lv_obj_t *parent, lv_coord_t x, lv_coord_t y);

#endif /* _MENU_H_ */
