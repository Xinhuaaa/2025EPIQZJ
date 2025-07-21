/**
  ******************************************************************************
  * @file    Robot_task.h
  * @brief   机器人主控模块头文件
  ******************************************************************************
  * @attention
  *
  * 描述：机器人系统初始化函数声明
  *
  ******************************************************************************
  */

#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

#include "main.h"
#include "chassis.h"
// 抓取位置宏定义
#define MoveToLeft() Chassis_MoveToPosition_Blocking(1.60, -0.48, 0, 0)
#define MoveToCenter() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 0)
#define MoveToRight() Chassis_MoveToPosition_Blocking(1.60, 0.50, 0, 0)
/* 位置移动宏定义 */
// 放置点位置宏定义 A-F
#define MoveToA() Chassis_MoveToPosition_Blocking(-0.71, -0.3, 90, 0)
#define MoveToB() Chassis_MoveToPosition_Blocking(-0.74,-0.67, 0, 0)
#define MoveToC() Chassis_MoveToPosition_Blocking(-0.74,-0.22, 0, 0)
#define MoveToD() Chassis_MoveToPosition_Blocking(-0.74,0.22, 0, 0)
#define MoveToE() Chassis_MoveToPosition_Blocking(-0.74,0.66, 0, 0)
#define MoveToF() Chassis_MoveToPosition_Blocking(-0.71, 0.3, 90, 0)




/* 函数声明 */

/**
 * @brief  机器人系统初始化函数
 * @param  无
 * @retval 初始化结果：0成功，-1失败
 */
int Robot_Init(void);



/**
 * @brief  机器人任务函数，用作RTOS任务
 * @param  argument: 任务参数（未使用）
 * @retval 无
 */
void Robot_task(void *argument);

#endif /* __ROBOT_TASK_H */