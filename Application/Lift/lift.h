/**
 ******************************************************************************
 * @file    lift.h
 * @author  TKX Team
 * @version V3.0.0
 * @date    2024-01-01
 * @brief   升降系统控制模块头文件 - 极简版本
 ******************************************************************************
 * @attention
 * 
 * 极简化的升降系统，使用4个M2006电机：
 * - 只保留基本的上升/下降运动功能
 * - 控制单位：位移（厘米），负值表示向下运动
 * - 机械参数：从动轮直径12cm，36:1减速比
 *
 ******************************************************************************
 */

#ifndef __LIFT_H
#define __LIFT_H

#include "main.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include "stdbool.h"

/* 升降状态结构体 */
typedef struct {
    float current_displacement;  // 当前位移 (cm)
    float target_displacement;   // 目标位移 (cm)
    float speed;                // 当前速度 (cm/s) - 仅用于状态监控
    bool enabled;               // 系统使能状态
    bool is_moving;             // 是否正在运动
} Lift_Status_t;

/* 电机ID定义 */
#define LIFT_MOTOR_1_ID          1  // 电机1
#define LIFT_MOTOR_2_ID          2  // 电机2  
#define LIFT_MOTOR_3_ID          3  // 电机3
#define LIFT_MOTOR_4_ID          4  // 电机4

/* 机械参数定义 */
#define LIFT_WHEEL_DIAMETER      0.12f    // 主动轮直径 (m)
#define LIFT_WHEEL_RADIUS        0.06f    // 主动轮半径 (m)
#define LIFT_WHEEL_CIRCUMFERENCE 0.377f   // 主动轮周长 (m) = π * 0.12
#define LIFT_GEAR_RATIO          36.0f    // 减速比 36:1

/* 运动参数定义 */
#define LIFT_DEFAULT_SPEED       5.0f   // 默认上升/下降速度 (cm/s)
#define LIFT_MAX_SPEED          10.0f   // 最大速度 (cm/s)
#define LIFT_MAX_DISPLACEMENT   50.0f   // 最大向上位移 (cm)
#define LIFT_MIN_DISPLACEMENT  -50.0f   // 最大向下位移 (cm)
#define LIFT_TASK_PERIOD        20      // 任务周期 (ms)

/* 任务参数 */
#define LIFT_TASK_STACK_SIZE     128    // 任务堆栈大小
#define LIFT_TASK_PRIORITY       5      // 任务优先级

/* 全局变量声明 */
extern DJIMotorInstance *lift_motors[4];
extern Lift_Status_t lift_status;
extern osThreadId_t liftTaskHandle;

/* 函数声明 */

/**
 * @brief 升降系统初始化
 * @retval 0: 成功, -1: 失败
 */
int Lift_Init(void);

/**
 * @brief 升降控制任务函数
 * @param argument 任务参数
 */
void LiftTask(void *argument);

/**
 * @brief 升降向上移动
 * @param displacement 目标位移 (cm)，相对于当前位置的增量
 * @retval 0: 成功, -1: 失败
 */
int Lift_Up(float displacement);

/**
 * @brief 升降向下移动  
 * @param displacement 目标位移 (cm)，相对于当前位置的增量（正值）
 * @retval 0: 成功, -1: 失败
 */
int Lift_Down(float displacement);

/**
 * @brief 移动到指定位置
 * @param target_displacement 目标绝对位移 (cm)，负值表示向下位移
 * @retval 0: 成功, -1: 失败
 */
int Lift_MoveTo(float target_displacement);

/**
 * @brief 停止升降运动
 * @retval 0: 成功, -1: 失败
 */
int Lift_Stop(void);

/**
 * @brief 获取升降状态
 * @return 升降状态结构体指针
 */
Lift_Status_t* Lift_GetStatus(void);

/**
 * @brief 获取当前位移
 * @return 当前位移 (cm)，负值表示向下位移
 */
float Lift_GetCurrentDisplacement(void);

#endif /* __LIFT_H */