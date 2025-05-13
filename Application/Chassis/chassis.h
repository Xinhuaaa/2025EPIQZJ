/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘运动控制头文件
  ******************************************************************************
  * @attention
  *
  * 描述：底盘运动控制接口定义，提供坐标闭环控制功能
  *
  ******************************************************************************
  */

#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#define HWT101_ANGLE_RANGE     360.0f  // HWT101角度传感器范围

/**
  * @brief  底盘初始化
  * @note   初始化底盘控制所需的各项参数和组件，并启动底盘控制任务
  * @retval 无
  */
void Chassis_Init(void);

/**
  * @brief  设置底盘目标位置
  * @param  x   目标X坐标，单位m
  * @param  y   目标Y坐标，单位m
  * @param  yaw 目标偏航角，单位度，范围(-180, 180]
  */
void Chassis_SetTargetPosition(float x, float y, float yaw);

/**
  * @brief  底盘控制循环，需要定期调用(建议10-20ms周期)
  * @return 底盘是否已到达目标位置
  */
bool Chassis_Control_Loop(void);

/**
  * @brief  获取底盘当前位置
  * @param  x   指向存储X坐标的变量指针，单位m
  * @param  y   指向存储Y坐标的变量指针，单位m
  * @param  yaw 指向存储偏航角的变量指针，单位度，范围(-180, 180]
  */
void Chassis_GetCurrentPosition(float *x, float *y, float *yaw);

/**
  * @brief  紧急停止底盘
  * @note   在紧急情况下调用此函数立即停止底盘运动
  * @retval 无
  */
void Chassis_EmergencyStop(void);

/**
  * @brief  重置底盘位置为原点
  * @note   重置当前位置为坐标原点(0,0,0)
  * @retval 无
  */
void Chassis_ResetPosition(void);

/**
  * @brief  设置底盘PID控制参数
  * @param  kp_xy XY方向PID比例系数
  * @param  ki_xy XY方向PID积分系数
  * @param  kd_xy XY方向PID微分系数
  * @param  kp_yaw 偏航角PID比例系数
  * @param  ki_yaw 偏航角PID积分系数
  * @param  kd_yaw 偏航角PID微分系数
  * @retval 无
  */
void Chassis_SetPIDParams(float kp_xy, float ki_xy, float kd_xy, 
                         float kp_yaw, float ki_yaw, float kd_yaw);

/* 函数声明别名 - 用于兼容性 */
#define EMM_V5_Init           Emm_V5_Init
#define EMM_V5_EnableMotor    Emm_V5_En_Control
#define EMM_V5_SetControlMode Emm_V5_Modify_Ctrl_Mode
#define EMM_V5_ResetEncoder   Emm_V5_Reset_CurPos_To_Zero
#define EMM_V5_GetAllEncoders Emm_V5_Get_All_Encoders
#define EMM_V5_SetSpeed       Emm_V5_Vel_Control
#define EMM_V5_Stop           Emm_V5_Stop_Now

#endif /* __CHASSIS_H */