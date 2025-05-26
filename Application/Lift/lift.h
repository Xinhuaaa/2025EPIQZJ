/**
 ******************************************************************************
 * @file    lift.h
 * @author  TKX Team
 * @version V1.0.0
 * @date    2024-01-01
 * @brief   升降系统控制模块头文件 - 基于4个DJI M2006电机
 ******************************************************************************
 * @attention
 * 
 * 该模块控制车辆升降结构，使用4个M2006电机：
 * - 左侧电机组：ID 1, 2
 * - 右侧电机组：ID 3, 4
 * 支持同步左右升降和独立左右控制
 *
 ******************************************************************************
 */

#ifndef __LIFT_H
#define __LIFT_H

#include "main.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include "stdbool.h"

/* 升降位置枚举 */
typedef enum {
    LIFT_BOTTOM = 0,    // 底部位置 (0度)
    LIFT_MIDDLE = 1,    // 中间位置 (90度)  
    LIFT_TOP = 2        // 顶部位置 (180度)
} Lift_Position_e;

/* 升降状态枚举 */
typedef enum {
    LIFT_IDLE = 0,      // 空闲状态
    LIFT_MOVING = 1,    // 移动中
    LIFT_REACHED = 2    // 到达目标位置
} Lift_State_e;

/* 升降侧面枚举 */
typedef enum {
    LIFT_SIDE_LEFT = 0,     // 左侧
    LIFT_SIDE_RIGHT = 1,    // 右侧
    LIFT_SIDE_BOTH = 2      // 双侧同步
} Lift_Side_e;

/* 升降控制模式枚举 */
typedef enum {
    LIFT_MODE_POSITION = 0, // 位置控制模式
    LIFT_MODE_MANUAL = 1    // 手动控制模式
} Lift_Mode_e;

/* 升降配置结构体 */
typedef struct {
    float position_tolerance;   // 位置允许误差 (度)
    float max_speed;           // 最大运行速度 (度/秒)
    uint32_t task_period;      // 任务周期 (ms)
    bool enable_sync;          // 是否启用同步控制
} Lift_Config_t;

/* 升降状态结构体 */
typedef struct {
    Lift_State_e left_state;      // 左侧状态
    Lift_State_e right_state;     // 右侧状态
    Lift_Position_e left_position;  // 左侧目标位置
    Lift_Position_e right_position; // 右侧目标位置
    float left_current_angle;     // 左侧当前角度
    float right_current_angle;    // 右侧当前角度
    float left_target_angle;      // 左侧目标角度
    float right_target_angle;     // 右侧目标角度
    Lift_Mode_e mode;            // 控制模式
    bool left_enabled;           // 左侧电机使能
    bool right_enabled;          // 右侧电机使能
} Lift_Status_t;

/* 电机ID定义 */
#define LIFT_MOTOR_LEFT_1_ID     1  // 左侧电机1
#define LIFT_MOTOR_LEFT_2_ID     2  // 左侧电机2  
#define LIFT_MOTOR_RIGHT_1_ID    3  // 右侧电机1
#define LIFT_MOTOR_RIGHT_2_ID    4  // 右侧电机2

/* 位置角度定义 (度) */
#define LIFT_ANGLE_BOTTOM        0.0f   // 底部位置角度
#define LIFT_ANGLE_MIDDLE        90.0f  // 中间位置角度
#define LIFT_ANGLE_TOP           180.0f // 顶部位置角度

/* 默认配置参数 */
#define LIFT_DEFAULT_TOLERANCE   2.0f   // 默认位置容差 (度)
#define LIFT_DEFAULT_MAX_SPEED   60.0f  // 默认最大速度 (度/秒)
#define LIFT_TASK_PERIOD         20     // 任务周期 (ms)

/* 任务参数 */
#define LIFT_TASK_STACK_SIZE     512    // 任务堆栈大小
#define LIFT_TASK_PRIORITY       5      // 任务优先级

/* 全局变量声明 */
extern DJIMotorInstance *lift_motor_left_1;
extern DJIMotorInstance *lift_motor_left_2;
extern DJIMotorInstance *lift_motor_right_1;
extern DJIMotorInstance *lift_motor_right_2;
extern Lift_Status_t lift_status;
extern Lift_Config_t lift_config;
extern osThreadId_t liftTaskHandle;

/* 函数声明 */

/**
 * @brief 升降系统初始化
 * @param config 配置参数指针，传入NULL使用默认配置
 * @retval 0: 成功, -1: 失败
 */
int Lift_Init(Lift_Config_t *config);

/**
 * @brief 创建升降任务
 * @retval 0: 成功, -1: 失败
 */
int Lift_CreateTask(void);

/**
 * @brief 升降控制任务函数
 * @param argument 任务参数
 */
void LiftTask(void *argument);

/**
 * @brief 设置升降位置
 * @param side 控制侧面
 * @param position 目标位置
 * @retval 0: 成功, -1: 失败
 */
int Lift_SetPosition(Lift_Side_e side, Lift_Position_e position);

/**
 * @brief 设置升降角度
 * @param side 控制侧面
 * @param angle 目标角度 (度)
 * @retval 0: 成功, -1: 失败
 */
int Lift_SetAngle(Lift_Side_e side, float angle);

/**
 * @brief 手动控制升降速度
 * @param side 控制侧面
 * @param speed 控制速度 (度/秒), 正值上升，负值下降
 * @retval 0: 成功, -1: 失败
 */
int Lift_ManualControl(Lift_Side_e side, float speed);

/**
 * @brief 停止升降运动
 * @param side 控制侧面
 * @retval 0: 成功, -1: 失败
 */
int Lift_Stop(Lift_Side_e side);

/**
 * @brief 使能/禁用升降电机
 * @param side 控制侧面
 * @param enable true: 使能, false: 禁用
 * @retval 0: 成功, -1: 失败
 */
int Lift_Enable(Lift_Side_e side, bool enable);

/**
 * @brief 获取升降状态
 * @return 升降状态结构体指针
 */
Lift_Status_t* Lift_GetStatus(void);

/**
 * @brief 获取当前位置
 * @param side 查询侧面
 * @return 当前角度 (度)
 */
float Lift_GetCurrentAngle(Lift_Side_e side);

/**
 * @brief 检查是否到达目标位置
 * @param side 检查侧面
 * @return true: 已到达, false: 未到达
 */
bool Lift_IsReached(Lift_Side_e side);

/**
 * @brief 升降系统复位到底部位置
 * @retval 0: 成功, -1: 失败
 */
int Lift_Reset(void);

#endif /* __LIFT_H */