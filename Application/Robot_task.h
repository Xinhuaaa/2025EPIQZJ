/**
  ******************************************************************************
  * @file    Robot_task.h
  * @brief   机器人任务头文件
  ******************************************************************************
  * @attention
  *
  * 描述：机器人主控任务声明，包括底盘导航控制
  *
  ******************************************************************************
  */

#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* 任务句柄 */
extern osThreadId_t chassisTaskHandle;
extern osThreadId_t navigationTaskHandle;
extern osThreadId_t canTestTaskHandle; // CAN测试任务句柄

/* 函数声明 */
void Robot_TaskCreate(void);
void Robot_StartLiftTest(void);
void Chassis_TestNavigation(void);
void Lift_TestControl(void);

/**
 * @brief CAN诊断函数，定期检查并打印CAN状态
 * 
 * @param interval_ms 诊断间隔，单位毫秒，0表示立即执行一次
 * @param times 诊断次数，0表示无限次
 */
void Robot_CANDiagnostic(uint32_t interval_ms, uint32_t times);

/**
 * @brief 启动周期性CAN测试帧发送任务
 * 
 * @param interval_ms 发送间隔，单位毫秒
 * @param motor_id 目标电机ID（1-8）
 * @param test_value 测试控制值，小值用于轻微运动测试
 * @return uint8_t 0表示成功创建任务，非0表示失败
 */
uint8_t Robot_StartCANTestTask(uint32_t interval_ms, uint8_t motor_id, int16_t test_value);

/**
 * @brief 停止周期性CAN测试帧发送任务
 * 
 * @return uint8_t 0表示成功停止，非0表示失败
 */
uint8_t Robot_StopCANTestTask(void);

#endif /* __ROBOT_TASK_H */