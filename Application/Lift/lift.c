/**
 ******************************************************************************
 * @file    lift.c
 * @author  TKX Team
 * @version V1.0.0
 * @date    2024-01-01
 * @brief   升降系统控制模块实现 - 基于4个DJI M2006电机
 ******************************************************************************
 * @attention
 * 
 * 该模块控制车辆升降结构，使用4个M2006电机：
 * - 左侧电机组：ID 1, 2 (同步控制)
 * - 右侧电机组：ID 3, 4 (同步控制)
 * 支持位置控制和手动控制两种模式
 *
 ******************************************************************************
 */

#include "lift.h"
#include "bsp_can.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "can.h"

/* 全局变量定义 */
DJIMotorInstance *lift_motor_left_1 = NULL;
DJIMotorInstance *lift_motor_left_2 = NULL;
DJIMotorInstance *lift_motor_right_1 = NULL;
DJIMotorInstance *lift_motor_right_2 = NULL;

Lift_Status_t lift_status;
Lift_Config_t lift_config;
osThreadId_t liftTaskHandle = NULL;

/* 私有函数声明 */
static float Lift_PositionToAngle(Lift_Position_e position);
static Lift_Position_e Lift_AngleToPosition(float angle);
static void Lift_UpdateCurrentAngles(void);
static void Lift_UpdateStates(void);
static void Lift_ControlMotors(void);
static int Lift_InitMotors(void);

/**
 * @brief 升降系统初始化
 * @param config 配置参数指针，传入NULL使用默认配置
 * @retval 0: 成功, -1: 失败
 */
int Lift_Init(Lift_Config_t *config)
{
    // 配置参数初始化
    if (config != NULL) {
        lift_config = *config;
    } else {
        // 使用默认配置
        lift_config.position_tolerance = LIFT_DEFAULT_TOLERANCE;
        lift_config.max_speed = LIFT_DEFAULT_MAX_SPEED;
        lift_config.task_period = LIFT_TASK_PERIOD;
        lift_config.enable_sync = true;
    }
    
    // 状态初始化
    memset(&lift_status, 0, sizeof(Lift_Status_t));
    lift_status.left_state = LIFT_IDLE;
    lift_status.right_state = LIFT_IDLE;
    lift_status.left_position = LIFT_BOTTOM;
    lift_status.right_position = LIFT_BOTTOM;
    lift_status.mode = LIFT_MODE_POSITION;
    lift_status.left_enabled = true;
    lift_status.right_enabled = true;
    
    // 初始化电机
    if (Lift_InitMotors() != 0) {
        printf("升降系统电机初始化失败!\r\n");
        return -1;
    }
    
    printf("升降系统初始化完成!\r\n");
    return 0;
}

/**
 * @brief 初始化升降电机
 * @retval 0: 成功, -1: 失败
 */
static int Lift_InitMotors(void)
{
    Motor_Init_Config_s m2006_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1
        },
        .motor_type = M2006,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f, .Ki = 0.0f, .Kd = 0.5f, .MaxOut = 60.0f,
                .DeadBand = 1.0f, .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 10.0f, .Derivative_LPF_RC = 0.01f
            },
            .speed_PID = {
                .Kp = 15.0f, .Ki = 0.2f, .Kd = 0.0f, .MaxOut = 8000.0f,
                .DeadBand = 0.5f, .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000.0f
            },
            .current_PID = {
                .Kp = 1.5f, .Ki = 0.1f, .Kd = 0.0f, .MaxOut = 10000.0f,
                .DeadBand = 0.0f, .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f
            }
        },
        .controller_setting_init_config = {
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            .outer_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = FEEDFORWARD_NONE
        }
    };
    
    // 初始化左侧电机1
    m2006_config.can_init_config.tx_id = LIFT_MOTOR_LEFT_1_ID;
    lift_motor_left_1 = DJIMotorInit(&m2006_config);
    if (lift_motor_left_1 == NULL) {
        printf("左侧电机1初始化失败!\r\n");
        return -1;
    }
    
    // 初始化左侧电机2
    m2006_config.can_init_config.tx_id = LIFT_MOTOR_LEFT_2_ID;
    lift_motor_left_2 = DJIMotorInit(&m2006_config);
    if (lift_motor_left_2 == NULL) {
        printf("左侧电机2初始化失败!\r\n");
        return -1;
    }
    
    // 初始化右侧电机1
    m2006_config.can_init_config.tx_id = LIFT_MOTOR_RIGHT_1_ID;
    lift_motor_right_1 = DJIMotorInit(&m2006_config);
    if (lift_motor_right_1 == NULL) {
        printf("右侧电机1初始化失败!\r\n");
        return -1;
    }
    
    // 初始化右侧电机2
    m2006_config.can_init_config.tx_id = LIFT_MOTOR_RIGHT_2_ID;
    lift_motor_right_2 = DJIMotorInit(&m2006_config);
    if (lift_motor_right_2 == NULL) {
        printf("右侧电机2初始化失败!\r\n");
        return -1;
    }
    
    printf("升降电机初始化成功: 左侧(ID:%d,%d) 右侧(ID:%d,%d)\r\n", 
           LIFT_MOTOR_LEFT_1_ID, LIFT_MOTOR_LEFT_2_ID,
           LIFT_MOTOR_RIGHT_1_ID, LIFT_MOTOR_RIGHT_2_ID);
    
    return 0;
}

/**
 * @brief 创建升降任务
 * @retval 0: 成功, -1: 失败
 */
int Lift_CreateTask(void)
{
    osThreadAttr_t liftTask_attributes = {
        .name = "LiftTask",
        .stack_size = LIFT_TASK_STACK_SIZE * 4,
        .priority = (osPriority_t)LIFT_TASK_PRIORITY,
    };
    
    liftTaskHandle = osThreadNew(LiftTask, NULL, &liftTask_attributes);
    if (liftTaskHandle == NULL) {
        printf("升降任务创建失败!\r\n");
        return -1;
    }
    
    printf("升降任务创建成功!\r\n");
    return 0;
}

/**
 * @brief 升降控制任务函数
 * @param argument 任务参数
 */
void LiftTask(void *argument)
{
    // 等待系统启动稳定
    osDelay(pdMS_TO_TICKS(1000));
    
    printf("升降控制任务启动!\r\n");
    
    // 任务主循环
    for(;;) {
        // 更新当前角度
        Lift_UpdateCurrentAngles();
        
        // 更新状态
        Lift_UpdateStates();
        
        // 控制电机
        Lift_ControlMotors();
        
        // 周期性延时
        osDelay(pdMS_TO_TICKS(lift_config.task_period));
    }
}

/**
 * @brief 更新当前角度
 */
static void Lift_UpdateCurrentAngles(void)
{
    if (lift_motor_left_1 && lift_motor_left_2) {
        // 左侧取两个电机角度的平均值
        float left_angle_1 = lift_motor_left_1->measure.total_angle;
        float left_angle_2 = lift_motor_left_2->measure.total_angle;
        lift_status.left_current_angle = (left_angle_1 + left_angle_2) / 2.0f;
    }
    
    if (lift_motor_right_1 && lift_motor_right_2) {
        // 右侧取两个电机角度的平均值
        float right_angle_1 = lift_motor_right_1->measure.total_angle;
        float right_angle_2 = lift_motor_right_2->measure.total_angle;
        lift_status.right_current_angle = (right_angle_1 + right_angle_2) / 2.0f;
    }
}

/**
 * @brief 更新状态
 */
static void Lift_UpdateStates(void)
{
    // 检查左侧是否到达目标
    if (lift_status.left_state == LIFT_MOVING) {
        float left_error = fabsf(lift_status.left_target_angle - lift_status.left_current_angle);
        if (left_error <= lift_config.position_tolerance) {
            lift_status.left_state = LIFT_REACHED;
        }
    }
    
    // 检查右侧是否到达目标
    if (lift_status.right_state == LIFT_MOVING) {
        float right_error = fabsf(lift_status.right_target_angle - lift_status.right_current_angle);
        if (right_error <= lift_config.position_tolerance) {
            lift_status.right_state = LIFT_REACHED;
        }
    }
}

/**
 * @brief 控制电机
 */
static void Lift_ControlMotors(void)
{
    // 位置控制模式
    if (lift_status.mode == LIFT_MODE_POSITION) {
        // 控制左侧电机
        if (lift_status.left_enabled && lift_status.left_state == LIFT_MOVING) {
            if (lift_motor_left_1) {
                DJIMotorSetRef(lift_motor_left_1, lift_status.left_target_angle);
            }
            if (lift_motor_left_2) {
                DJIMotorSetRef(lift_motor_left_2, lift_status.left_target_angle);
            }
        }
        
        // 控制右侧电机
        if (lift_status.right_enabled && lift_status.right_state == LIFT_MOVING) {
            if (lift_motor_right_1) {
                DJIMotorSetRef(lift_motor_right_1, lift_status.right_target_angle);
            }
            if (lift_motor_right_2) {
                DJIMotorSetRef(lift_motor_right_2, lift_status.right_target_angle);
            }
        }
    }
}

/**
 * @brief 设置升降位置
 * @param side 控制侧面
 * @param position 目标位置
 * @retval 0: 成功, -1: 失败
 */
int Lift_SetPosition(Lift_Side_e side, Lift_Position_e position)
{
    float target_angle = Lift_PositionToAngle(position);
    return Lift_SetAngle(side, target_angle);
}

/**
 * @brief 设置升降角度
 * @param side 控制侧面
 * @param angle 目标角度 (度)
 * @retval 0: 成功, -1: 失败
 */
int Lift_SetAngle(Lift_Side_e side, float angle)
{
    // 角度范围检查
    if (angle < LIFT_ANGLE_BOTTOM || angle > LIFT_ANGLE_TOP) {
        printf("升降角度超出范围: %.1f度 (范围: %.1f-%.1f度)\r\n", 
               angle, LIFT_ANGLE_BOTTOM, LIFT_ANGLE_TOP);
        return -1;
    }
    
    // 设置控制模式为位置控制
    lift_status.mode = LIFT_MODE_POSITION;
    
    switch (side) {
        case LIFT_SIDE_LEFT:
            lift_status.left_target_angle = angle;
            lift_status.left_position = Lift_AngleToPosition(angle);
            lift_status.left_state = LIFT_MOVING;
            printf("设置左侧升降目标角度: %.1f度\r\n", angle);
            break;
            
        case LIFT_SIDE_RIGHT:
            lift_status.right_target_angle = angle;
            lift_status.right_position = Lift_AngleToPosition(angle);
            lift_status.right_state = LIFT_MOVING;
            printf("设置右侧升降目标角度: %.1f度\r\n", angle);
            break;
            
        case LIFT_SIDE_BOTH:
            lift_status.left_target_angle = angle;
            lift_status.right_target_angle = angle;
            lift_status.left_position = Lift_AngleToPosition(angle);
            lift_status.right_position = Lift_AngleToPosition(angle);
            lift_status.left_state = LIFT_MOVING;
            lift_status.right_state = LIFT_MOVING;
            printf("设置双侧升降目标角度: %.1f度\r\n", angle);
            break;
            
        default:
            return -1;
    }
    
    return 0;
}

/**
 * @brief 手动控制升降速度
 * @param side 控制侧面
 * @param speed 控制速度 (度/秒), 正值上升，负值下降
 * @retval 0: 成功, -1: 失败
 */
int Lift_ManualControl(Lift_Side_e side, float speed)
{
    // 速度限制
    if (fabsf(speed) > lift_config.max_speed) {
        speed = (speed > 0) ? lift_config.max_speed : -lift_config.max_speed;
    }
    
    // 设置控制模式为手动控制
    lift_status.mode = LIFT_MODE_MANUAL;
    
    switch (side) {
        case LIFT_SIDE_LEFT:
            if (lift_status.left_enabled) {
                // 计算目标角度 (当前角度 + 速度 * 控制周期)
                float dt = lift_config.task_period / 1000.0f; // 转换为秒
                lift_status.left_target_angle = lift_status.left_current_angle + speed * dt;
                
                // 角度限制
                if (lift_status.left_target_angle < LIFT_ANGLE_BOTTOM) {
                    lift_status.left_target_angle = LIFT_ANGLE_BOTTOM;
                } else if (lift_status.left_target_angle > LIFT_ANGLE_TOP) {
                    lift_status.left_target_angle = LIFT_ANGLE_TOP;
                }
                
                lift_status.left_state = LIFT_MOVING;
            }
            break;
            
        case LIFT_SIDE_RIGHT:
            if (lift_status.right_enabled) {
                float dt = lift_config.task_period / 1000.0f;
                lift_status.right_target_angle = lift_status.right_current_angle + speed * dt;
                
                if (lift_status.right_target_angle < LIFT_ANGLE_BOTTOM) {
                    lift_status.right_target_angle = LIFT_ANGLE_BOTTOM;
                } else if (lift_status.right_target_angle > LIFT_ANGLE_TOP) {
                    lift_status.right_target_angle = LIFT_ANGLE_TOP;
                }
                
                lift_status.right_state = LIFT_MOVING;
            }
            break;
            
        case LIFT_SIDE_BOTH:
            if (lift_status.left_enabled && lift_status.right_enabled) {
                float dt = lift_config.task_period / 1000.0f;
                
                lift_status.left_target_angle = lift_status.left_current_angle + speed * dt;
                lift_status.right_target_angle = lift_status.right_current_angle + speed * dt;
                
                // 左侧角度限制
                if (lift_status.left_target_angle < LIFT_ANGLE_BOTTOM) {
                    lift_status.left_target_angle = LIFT_ANGLE_BOTTOM;
                } else if (lift_status.left_target_angle > LIFT_ANGLE_TOP) {
                    lift_status.left_target_angle = LIFT_ANGLE_TOP;
                }
                
                // 右侧角度限制
                if (lift_status.right_target_angle < LIFT_ANGLE_BOTTOM) {
                    lift_status.right_target_angle = LIFT_ANGLE_BOTTOM;
                } else if (lift_status.right_target_angle > LIFT_ANGLE_TOP) {
                    lift_status.right_target_angle = LIFT_ANGLE_TOP;
                }
                
                lift_status.left_state = LIFT_MOVING;
                lift_status.right_state = LIFT_MOVING;
            }
            break;
            
        default:
            return -1;
    }
    
    return 0;
}

/**
 * @brief 停止升降运动
 * @param side 控制侧面
 * @retval 0: 成功, -1: 失败
 */
int Lift_Stop(Lift_Side_e side)
{
    switch (side) {
        case LIFT_SIDE_LEFT:
            lift_status.left_state = LIFT_IDLE;
            lift_status.left_target_angle = lift_status.left_current_angle;
            break;
            
        case LIFT_SIDE_RIGHT:
            lift_status.right_state = LIFT_IDLE;
            lift_status.right_target_angle = lift_status.right_current_angle;
            break;
            
        case LIFT_SIDE_BOTH:
            lift_status.left_state = LIFT_IDLE;
            lift_status.right_state = LIFT_IDLE;
            lift_status.left_target_angle = lift_status.left_current_angle;
            lift_status.right_target_angle = lift_status.right_current_angle;
            break;
            
        default:
            return -1;
    }
    
    printf("升降运动已停止\r\n");
    return 0;
}

/**
 * @brief 使能/禁用升降电机
 * @param side 控制侧面
 * @param enable true: 使能, false: 禁用
 * @retval 0: 成功, -1: 失败
 */
int Lift_Enable(Lift_Side_e side, bool enable)
{
    switch (side) {
        case LIFT_SIDE_LEFT:
            lift_status.left_enabled = enable;
            if (lift_motor_left_1) {
                DJIMotorEnable(lift_motor_left_1);
            }
            if (lift_motor_left_2) {
                DJIMotorEnable(lift_motor_left_2);
            }
            break;
            
        case LIFT_SIDE_RIGHT:
            lift_status.right_enabled = enable;
            if (lift_motor_right_1) {
                DJIMotorEnable(lift_motor_right_1);
            }
            if (lift_motor_right_2) {
                DJIMotorEnable(lift_motor_right_2);
            }
            break;
            
        case LIFT_SIDE_BOTH:
            lift_status.left_enabled = enable;
            lift_status.right_enabled = enable;
            if (lift_motor_left_1) {
                DJIMotorEnable(lift_motor_left_1);
            }
            if (lift_motor_left_2) {
                DJIMotorEnable(lift_motor_left_2);
            }
            if (lift_motor_right_1) {
                DJIMotorEnable(lift_motor_right_1);
            }
            if (lift_motor_right_2) {
                DJIMotorEnable(lift_motor_right_2);
            }
            break;
            
        default:
            return -1;
    }
    
    printf("升降电机使能状态已设置: %s\r\n", enable ? "开启" : "关闭");
    return 0;
}

/**
 * @brief 获取升降状态
 * @return 升降状态结构体指针
 */
Lift_Status_t* Lift_GetStatus(void)
{
    return &lift_status;
}

/**
 * @brief 获取当前位置
 * @param side 查询侧面
 * @return 当前角度 (度)
 */
float Lift_GetCurrentAngle(Lift_Side_e side)
{
    switch (side) {
        case LIFT_SIDE_LEFT:
            return lift_status.left_current_angle;
            
        case LIFT_SIDE_RIGHT:
            return lift_status.right_current_angle;
            
        case LIFT_SIDE_BOTH:
            // 返回双侧的平均值
            return (lift_status.left_current_angle + lift_status.right_current_angle) / 2.0f;
            
        default:
            return 0.0f;
    }
}

/**
 * @brief 检查是否到达目标位置
 * @param side 检查侧面
 * @return true: 已到达, false: 未到达
 */
bool Lift_IsReached(Lift_Side_e side)
{
    switch (side) {
        case LIFT_SIDE_LEFT:
            return (lift_status.left_state == LIFT_REACHED);
            
        case LIFT_SIDE_RIGHT:
            return (lift_status.right_state == LIFT_REACHED);
            
        case LIFT_SIDE_BOTH:
            return (lift_status.left_state == LIFT_REACHED && 
                    lift_status.right_state == LIFT_REACHED);
            
        default:
            return false;
    }
}

/**
 * @brief 升降系统复位到底部位置
 * @retval 0: 成功, -1: 失败
 */
int Lift_Reset(void)
{
    printf("升降系统复位到底部位置...\r\n");
    return Lift_SetPosition(LIFT_SIDE_BOTH, LIFT_BOTTOM);
}

/**
 * @brief 位置枚举转换为角度
 * @param position 位置枚举
 * @return 对应角度 (度)
 */
static float Lift_PositionToAngle(Lift_Position_e position)
{
    switch (position) {
        case LIFT_BOTTOM:
            return LIFT_ANGLE_BOTTOM;
        case LIFT_MIDDLE:
            return LIFT_ANGLE_MIDDLE;
        case LIFT_TOP:
            return LIFT_ANGLE_TOP;
        default:
            return LIFT_ANGLE_BOTTOM;
    }
}

/**
 * @brief 角度转换为位置枚举
 * @param angle 角度 (度)
 * @return 最接近的位置枚举
 */
static Lift_Position_e Lift_AngleToPosition(float angle)
{
    float bottom_diff = fabsf(angle - LIFT_ANGLE_BOTTOM);
    float middle_diff = fabsf(angle - LIFT_ANGLE_MIDDLE);
    float top_diff = fabsf(angle - LIFT_ANGLE_TOP);
    
    if (bottom_diff <= middle_diff && bottom_diff <= top_diff) {
        return LIFT_BOTTOM;
    } else if (middle_diff <= top_diff) {
        return LIFT_MIDDLE;
    } else {
        return LIFT_TOP;
    }
}