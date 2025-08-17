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

/*
 * 方案1：基准轨迹 + 偏移
 * 这里的宏只提供基准坐标，ErrX/ErrY/ErrYaw 不再直接叠加在坐标参数里，
 * 由校准流程通过 Chassis_AddOffset 累加到偏移。
 * 若需要应用全局误差（ErrX 等），先调用 APPLY_ERR_OFFSET() 刷新到底盘偏移。
 */
#define APPLY_ERR_OFFSET() do { \
  Chassis_ResetOffset();      \
  Chassis_AddOffset(ErrX, ErrY, ErrYaw); \
} while(0)

// 抓取位置基准宏
#define MoveToLeft()       Chassis_MoveToPosition_Blocking(1.64f,  0.50f,  0.0f, 5200)
#define MoveToCenter()     Chassis_MoveToPosition_Blocking(1.64f,  0.00f,  0.0f, 5200)
#define MoveToRight()      Chassis_MoveToPosition_Blocking(1.64f, -0.50f,  0.0f, 5200)
#define STARTMoveToLeft()  Chassis_MoveToPosition_Blocking(1.64f,  0.50f,  0.0f, 0)
#define STARTMoveToCenter() Chassis_MoveToPosition_Blocking(1.64f, 0.00f,  0.0f, 0)
#define STARTMoveToRight() Chassis_MoveToPosition_Blocking(1.64f, -0.50f,  0.0f, 0)
// #define MoveToLeft()       Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 6200)
// #define MoveToCenter()     Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 6200)
// #define MoveToRight()      Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 6200)
// #define STARTMoveToLeft()  Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 0)
// #define STARTMoveToCenter() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 0)
// #define STARTMoveToRight() Chassis_MoveToPosition_Blocking((0.0f + ErrX), (0.00f + ErrY), (0.0f + ErrYaw), 0)
// -----------------------------------------------
// #define MoveToLeft() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 6000)
// #define MoveToCenter() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 6000)
// #define MoveToRight() Chassis_MoveToPosition_Blocking(1.60, 0, 0, 6000)
/* 位置移动宏定义 */
// 放置点位置宏定义 A-F
// 路线1 bcdefa
#define MoveToA1() Chassis_MoveToPosition_Blocking(-0.675f, -0.205f, -90.0f, 100000)
#define MoveToB1() Chassis_MoveToPosition_Blocking(-0.700f, -0.650f,   0.0f, 100000)
#define MoveToC1() Chassis_MoveToPosition_Blocking(-0.700f, -0.198f,   0.0f, 100000)
#define MoveToD1() Chassis_MoveToPosition_Blocking(-0.700f,  0.249f,   0.0f, 100000)
#define MoveToE1() Chassis_MoveToPosition_Blocking(-0.700f,  0.710f,   0.0f, 100000)
#define MoveToF1() Chassis_MoveToPosition_Blocking(-0.685f,  0.295f, -90.0f, 100000)
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
#define MoveToA0() Chassis_MoveToPosition_Blocking(-0.665f, -0.203f, -90.0f, 100000)
#define MoveToB0() Chassis_MoveToPosition_Blocking(-0.700f, -0.650f,   0.0f, 100000)
#define MoveToC0() Chassis_MoveToPosition_Blocking(-0.700f, -0.198f,   0.0f, 100000)
#define MoveToD0() Chassis_MoveToPosition_Blocking(-0.700f,  0.249f,   0.0f, 100000)
#define MoveToE0() Chassis_MoveToPosition_Blocking(-0.700f,  0.710f,   0.0f, 100000)
#define MoveToF0() Chassis_MoveToPosition_Blocking(-0.685f,  0.295f, -90.0f, 100000)
// #define MoveToA0() Chassis_MoveToPosition_Blocking(-0.675f, -0.188f, -90.0f, 100000)
// #define MoveToB0() Chassis_MoveToPosition_Blocking(-0.700f, -0.687f,   0.0f, 100000)
// #define MoveToC0() Chassis_MoveToPosition_Blocking(-0.700f, -0.200f,   0.0f, 100000)
// #define MoveToD0() Chassis_MoveToPosition_Blocking(-0.700f,  0.235f,   0.0f, 100000)
// #define MoveToE0() Chassis_MoveToPosition_Blocking(-0.700f,  0.695f,   0.0f, 100000)
// #define MoveToF0() Chassis_MoveToPosition_Blocking(-0.690f,  0.300f, -90.0f, 100000)
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