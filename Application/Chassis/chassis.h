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
#include "ADRC.h" // 添加ADRC控制器头文件

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
  * @brief  阻塞式移动底盘到指定位置
  * @param  x   目标X坐标，单位m
  * @param  y   目标Y坐标，单位m
  * @param  yaw 目标偏航角，单位度，范围(-180, 180]
  * @param  timeout_ms 超时时间，单位ms，设为0表示无超时限制
  * @return 是否成功到达目标位置（超时返回false）
  */
bool Chassis_MoveToPosition_Blocking(float x, float y, float yaw, uint32_t timeout_ms);

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
  * @brief  设置X轴PID控制参数
  * @param  kp X方向比例系数
  * @param  ki X方向积分系数
  * @param  kd X方向微分系数
  */
void Chassis_SetXPIDParams(float kp, float ki, float kd);

/**
  * @brief  设置Y轴PID控制参数
  * @param  kp Y方向比例系数
  * @param  ki Y方向积分系数
  * @param  kd Y方向微分系数
  */
void Chassis_SetYPIDParams(float kp, float ki, float kd);

/**
  * @brief  设置偏航角PID控制参数
  * @param  kp 偏航角比例系数
  * @param  ki 偏航角积分系数
  * @param  kd 偏航角微分系数
  */
void Chassis_SetYawPIDParams(float kp, float ki, float kd);

/**
  * @brief  设置底盘所有PID控制参数(兼容旧接口)
  * @param  kp_x X方向比例系数
  * @param  ki_x X方向积分系数
  * @param  kd_x X方向微分系数
  * @param  kp_y Y方向比例系数
  * @param  ki_y Y方向积分系数
  * @param  kd_y Y方向微分系数
  * @param  kp_yaw 偏航角比例系数
  * @param  ki_yaw 偏航角积分系数
  * @param  kd_yaw 偏航角微分系数
  */
void Chassis_SetPIDParams(float kp_x, float ki_x, float kd_x, 
                         float kp_y, float ki_y, float kd_y,
                         float kp_yaw, float ki_yaw, float kd_yaw);
                         
/**
  * @brief  设置底盘ADRC控制器基本参数
  * @param  r_xy XY方向跟踪速度因子
  * @param  b0_xy XY方向系统增益
  * @param  r_yaw 偏航角跟踪速度因子
  * @param  b0_yaw 偏航角系统增益
  * @retval 无
  */
void Chassis_SetADRCParams(float r_xy, float b0_xy, float r_yaw, float b0_yaw);

/**
  * @brief  设置底盘ADRC观测器参数
  * @param  beta01_xy XY方向ESO反馈增益1
  * @param  beta02_xy XY方向ESO反馈增益2
  * @param  beta03_xy XY方向ESO反馈增益3
  * @param  beta01_yaw 偏航角ESO反馈增益1
  * @param  beta02_yaw 偏航角ESO反馈增益2
  * @param  beta03_yaw 偏航角ESO反馈增益3
  * @retval 无
  */
void Chassis_SetADRCESOParams(float beta01_xy, float beta02_xy, float beta03_xy,
                             float beta01_yaw, float beta02_yaw, float beta03_yaw);

/**
  * @brief  重置底盘控制器状态
  * @note   清空控制器内部状态，包括积分项、观测器状态等
  * @retval 无
  */
void Chassis_ResetController(void);
bool Chassis_MoveToX_Blocking(float x, uint32_t timeout_ms);

/* 函数声明别名 - 用于兼容性 */
#define EMM_V5_Init           Emm_V5_Init
#define EMM_V5_EnableMotor    Emm_V5_En_Control
#define EMM_V5_SetControlMode Emm_V5_Modify_Ctrl_Mode
#define EMM_V5_ResetEncoder   Emm_V5_Reset_CurPos_To_Zero
#define EMM_V5_GetAllEncoders Emm_V5_Get_All_Encoders
#define EMM_V5_SetSpeed       Emm_V5_Vel_Control
#define EMM_V5_Stop           Emm_V5_Stop_Now

#endif /* __CHASSIS_H */