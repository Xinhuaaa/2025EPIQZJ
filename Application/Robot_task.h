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

// 位置误差（偏置/在线修正量），通过 CDC 指令动态更新：x+.../y.../yaw...
extern float ErrX;   // 单位与底盘接口一致（米）
extern float ErrY;   // 单位与底盘接口一致（米）
extern float ErrYaw; // 单位：度（与现有 Chassis_MoveToPosition_Blocking 第三个参数一致）

// 抓取位置宏定义
#define MoveToLeft()       Chassis_MoveToPosition_Blocking((1.60f + ErrX), (0.50f + ErrY), (0.0f + ErrYaw), 7000)
#define MoveToCenter()     Chassis_MoveToPosition_Blocking((1.60f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 7000)
#define MoveToRight()      Chassis_MoveToPosition_Blocking((1.60f + ErrX), (-0.50f + ErrY), (0.0f + ErrYaw), 7000)
#define STARTMoveToLeft()  Chassis_MoveToPosition_Blocking((1.60f + ErrX), (0.50f + ErrY), (0.0f + ErrYaw), 0)
#define STARTMoveToCenter() Chassis_MoveToPosition_Blocking((1.60f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 0)
#define STARTMoveToRight() Chassis_MoveToPosition_Blocking((1.60f + ErrX), (-0.50f + ErrY), (0.0f + ErrYaw), 0)
// -----------------------------------------------
// #define MoveToLeft() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 6000)
// #define MoveToCenter() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 6000)
// #define MoveToRight() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 6000)
/* 位置移动宏定义 */
// 放置点位置宏定义 A-F
// 路线1 bcdefa
#define MoveToA1() Chassis_MoveToPosition_Blocking((-0.685f + ErrX), (-0.205f + ErrY), (-90.0f + ErrYaw), 0)
#define MoveToB1() Chassis_MoveToPosition_Blocking((-0.705f + ErrX), (-0.650f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToC1() Chassis_MoveToPosition_Blocking((-0.700f + ErrX), (-0.203f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToD1() Chassis_MoveToPosition_Blocking((-0.695f + ErrX), (0.239f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToE1() Chassis_MoveToPosition_Blocking((-0.690f + ErrX), (0.690f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToF1() Chassis_MoveToPosition_Blocking((-0.675f + ErrX), (0.280f + ErrY), (-90.0f + ErrYaw), 0)
// #define MoveToA1() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.0f + ErrY), (0.0f + ErrYaw), 0)
// #define MoveToB1() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.0f + ErrY), (0.0f + ErrYaw), 0)
// #define MoveToC1() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.0f + ErrY), (0.0f + ErrYaw), 0)
// #define MoveToD1() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.0f + ErrY), (0.0f + ErrYaw), 0)
// #define MoveToE1() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.0f + ErrY), (0.0f + ErrYaw), 0)
// #define MoveToF1() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.0f + ErrY), (0.0f + ErrYaw), 0)
// -----------------------------------------------
// #define MoveToA1() Chassis_MoveToPosition_Blocking(-0.685, 0, 0, 0)
// #define MoveToB1() Chassis_MoveToPosition_Blocking(-0.695,0, 0, 0)
// #define MoveToC1() Chassis_MoveToPosition_Blocking(-0.705,0, 0, 0)
// #define MoveToD1() Chassis_MoveToPosition_Blocking(-0.705,0, 0, 0)
// #define MoveToE1() Chassis_MoveToPosition_Blocking(-0.70,0, 0, 0)
// #define MoveToF1() Chassis_MoveToPosition_Blocking(-0.685,0, 0, 0)
// 路线0 edcba
#define MoveToA0() Chassis_MoveToPosition_Blocking((-0.675f + ErrX), (-0.188f + ErrY), (-90.0f + ErrYaw), 0)
#define MoveToB0() Chassis_MoveToPosition_Blocking((-0.700f + ErrX), (-0.687f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToC0() Chassis_MoveToPosition_Blocking((-0.700f + ErrX), (-0.200f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToD0() Chassis_MoveToPosition_Blocking((-0.700f + ErrX), (0.235f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToE0() Chassis_MoveToPosition_Blocking((-0.700f + ErrX), (0.695f + ErrY), (0.0f + ErrYaw), 0)
#define MoveToF0() Chassis_MoveToPosition_Blocking((-0.690f + ErrX), (0.300f + ErrY), (-90.0f + ErrYaw), 0)
// 抓取位置宏定义
// #define MoveToLeft() Chassis_MoveToPosition_Blocking(0, 0, 0, 0)
// #define MoveToCenter() Chassis_MoveToPosition_Blocking(0, 0, 0, 0)
// #define MoveToRight() Chassis_MoveToPosition_Blocking(0, 0, 0, 0)
// /* 位置移动宏定义 */
// // 放置点位置宏定义 A-F
// #define MoveToA() Chassis_MoveToPosition_Blocking(0.0, 0, 0, 0)
// #define MoveToB() Chassis_MoveToPosition_Blocking(0.0,0, 0, 0)
// #define MoveToC() Chassis_MoveToPosition_Blocking(0.0,0, 0, 0)
// #define MoveToD() Chassis_MoveToPosition_Blocking(0.0,0, 0, 0)
// #define MoveToE() Chassis_MoveToPosition_Blocking(0.0,0, 0, 0)
// #define MoveToF() Chassis_MoveToPosition_Blocking(0.0, 0, 0, 0)




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