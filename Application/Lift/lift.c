/**
 * @attention
 *
 * 极简化的升降系统，使用4个M2006电机：
 * - 只保留基本的上升/下降运动功能
 * - 控制单位：位移（厘米），负值表示向下运动
 * - 机械参数：主动轮直径12cm，36:1减速比
 *
 * 使用示例：
 * Lift_Up(5.0f);          // 以5cm/s速度上升
 * Lift_Down(0.0f);        // 以默认速度下降
 * Lift_Stop();            // 停止运动
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
DJIMotorInstance *lift_motors[4] = {NULL, NULL, NULL, NULL};
Lift_Status_t lift_status;
osThreadId_t liftTaskHandle = NULL;

/* 私有函数声明 */
static float Lift_AngleToDisplacement(float angle);
static float Lift_DisplacementToAngle(float displacement);
static void Lift_UpdateCurrentDisplacement(void);
static void Lift_UpdateCurrentSpeed(void);
static void Lift_ControlMotors(void);
void Lift_WaitUntilAtTarget(void);

/**
 * @brief 升降系统初始化
 * @retval 0: 成功, -1: 失败
 */
int Lift_Init(void)
{
    // 状态初始化
    memset(&lift_status, 0, sizeof(Lift_Status_t));
    lift_status.enabled = true;
    lift_status.current_displacement = 0.0f;
    lift_status.target_displacement = 0.0f;
    lift_status.speed = 0.0f;
    lift_status.is_moving = false;

    // 初始化电机配置
    Motor_Init_Config_s m2006_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1
        },
        .motor_type = M2006,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f, 
                .Ki = 0.3f, 
                .Kd = 0.0f, 
                .MaxOut = 30000.0f, 
                .DeadBand = 1.0f, 
                .Improve = PID_Integral_Limit | PID_DerivativeFilter, 
                .IntegralLimit = 10.0f, 
                .Derivative_LPF_RC = 0.01f
            }, 
            .speed_PID = {
                .Kp = 0.28f, 
                .Ki = 0.01f, 
                .Kd = 0.0f, 
                .MaxOut = 30000.0f, 
                .DeadBand = 0.5f, 
                .Improve = PID_Integral_Limit, 
                .IntegralLimit = 3000.0f
            }
        },
        .controller_setting_init_config = {
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .outer_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED, 
            .speed_feedback_source = MOTOR_FEED, 
            .feedforward_flag = FEEDFORWARD_NONE
        }
    };

    // 初始化4个电机
    for (int i = 0; i < 4; i++)
    {
        m2006_config.can_init_config.tx_id = LIFT_MOTOR_1_ID + i;
        
        // 设置ID为1和3的电机反向转动
        if (i == 0 || i == 2) // ID 1和3的电机 (数组索引0和2)
        {
            m2006_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
        }
        else
        {
            m2006_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
        }        
        
        if(i == 0) // ID 1的电机 (数组索引0)
        {
            // 设置速度PID参数
            m2006_config.controller_param_init_config.speed_PID.Kp = 0.7f;
            m2006_config.controller_param_init_config.speed_PID.Ki = 0.01f;
            m2006_config.controller_param_init_config.speed_PID.Kd = 0.0f;
            m2006_config.controller_param_init_config.speed_PID.MaxOut = 30000.0f;
            m2006_config.controller_param_init_config.speed_PID.DeadBand = 0.5f;
            m2006_config.controller_param_init_config.speed_PID.Improve = PID_Integral_Limit;
            m2006_config.controller_param_init_config.speed_PID.IntegralLimit = 3000.0f;
            
            // 设置角度PID参数
            m2006_config.controller_param_init_config.angle_PID.Kp = 10.0f;
            m2006_config.controller_param_init_config.angle_PID.Ki = 0.3f;
            m2006_config.controller_param_init_config.angle_PID.Kd = 0.0f;
            m2006_config.controller_param_init_config.angle_PID.MaxOut = 30000.0f;
            m2006_config.controller_param_init_config.angle_PID.DeadBand = 1.0f;
            m2006_config.controller_param_init_config.angle_PID.Improve = PID_Integral_Limit | PID_DerivativeFilter;
            m2006_config.controller_param_init_config.angle_PID.IntegralLimit = 10.0f;
            m2006_config.controller_param_init_config.angle_PID.Derivative_LPF_RC = 0.01f;
        }
        if(i == 1) // ID 2的电机 (数组索引1)
        {
            // 设置速度PID参数
            m2006_config.controller_param_init_config.speed_PID.Kp = 0.8f;
            m2006_config.controller_param_init_config.speed_PID.Ki = 0.01f;
            m2006_config.controller_param_init_config.speed_PID.Kd = 0.0f;
            m2006_config.controller_param_init_config.speed_PID.MaxOut = 30000.0f;
            m2006_config.controller_param_init_config.speed_PID.DeadBand = 0.5f;
            m2006_config.controller_param_init_config.speed_PID.Improve = PID_Integral_Limit;
            m2006_config.controller_param_init_config.speed_PID.IntegralLimit = 3000.0f;
            
            // 设置角度PID参数
            m2006_config.controller_param_init_config.angle_PID.Kp = 10.0f;
            m2006_config.controller_param_init_config.angle_PID.Ki = 0.3f;
            m2006_config.controller_param_init_config.angle_PID.Kd = 0.0f;
            m2006_config.controller_param_init_config.angle_PID.MaxOut = 30000.0f;
            m2006_config.controller_param_init_config.angle_PID.DeadBand = 1.0f;
            m2006_config.controller_param_init_config.angle_PID.Improve = PID_Integral_Limit | PID_DerivativeFilter;
            m2006_config.controller_param_init_config.angle_PID.IntegralLimit = 10.0f;
            m2006_config.controller_param_init_config.angle_PID.Derivative_LPF_RC = 0.01f;
        }
        
        lift_motors[i] = DJIMotorInit(&m2006_config);
        if (lift_motors[i] == NULL)
        {
            printf("升降系统电机初始化失败!\r\n");
            return -1;
        }
        // 使能电机
        DJIMotorEnable(lift_motors[i]);
    }

    printf("升降系统初始化完成!\r\n");
    
    // 直接创建升降控制任务
    const osThreadAttr_t liftTask_attributes = {
        .name = "LiftTask",
        .stack_size = LIFT_TASK_STACK_SIZE * 4,
        .priority = (osPriority_t)LIFT_TASK_PRIORITY,
    };

    liftTaskHandle = osThreadNew(LiftTask, NULL, &liftTask_attributes);
    if (liftTaskHandle == NULL)
    {
        printf("升降任务创建失败!\r\n");
        return -1;
    }
    
    return 0;
}

/**
 * @brief 升降控制任务函数
 * @param argument 任务参数
 */
void LiftTask(void *argument)
{
    printf("升降任务启动\r\n");

    while (1)
    {
        // 更新当前位移
        Lift_UpdateCurrentDisplacement();
        
        // 更新当前速度（用于监控）
        Lift_UpdateCurrentSpeed();
        
        // 控制电机
        Lift_ControlMotors();
        
        // 延时
        osDelay(pdMS_TO_TICKS(LIFT_TASK_PERIOD));
    }
}

/**
 * @brief 更新当前位移
 */
static void Lift_UpdateCurrentDisplacement(void)
{
    if (!lift_status.enabled) return;

    float total_angle = 0.0f;
    int valid_motors = 0;

    // 计算所有电机角度的平均值，考虑反向电机的角度值
    for (int i = 0; i < 4; i++)
    {
        if (lift_motors[i])
        {
            float motor_angle = lift_motors[i]->measure.total_angle;
            
            // 对于反向电机（ID为1和3，数组索引0和2），需要取反角度值
            if (i == 0 || i == 2)
            {
                motor_angle = -motor_angle;
            }
            
            total_angle += motor_angle;
            valid_motors++;
        }
    }

    if (valid_motors > 0)
    {
        float avg_angle = total_angle / valid_motors;
        lift_status.current_displacement = Lift_AngleToDisplacement(avg_angle);
    }
}

/**
 * @brief 控制电机
 */
static void Lift_ControlMotors(void)
{
    if (!lift_status.enabled) return;

    // 位移控制模式：将目标位移转换为电机目标角度
    float target_angle = Lift_DisplacementToAngle(lift_status.target_displacement);
    
    // 设置所有电机的目标角度
    for (int i = 0; i < 4; i++)
    {
        if (lift_motors[i])
        {
            DJIMotorSetRef(lift_motors[i], target_angle);
        }
    }
    
}

/**
 * @brief 升降向上移动
 * @param displacement 目标位移 (cm)，相对于当前位置的增量
 * @retval 0: 成功, -1: 失败
 */
int Lift_Up(float displacement)
{
    lift_status.target_displacement = displacement;

    return 0;
}

/**
 * @brief 停止升降运动
 * @retval 0: 成功, -1: 失败
 */
int Lift_Stop(void)
{
    // 设置目标位移为当前位移，停止运动
    lift_status.target_displacement = lift_status.current_displacement;
    lift_status.is_moving = false;

    printf("升降运动已停止在位移: %.2fcm\r\n", lift_status.current_displacement);
    return 0;
}

/**
 * @brief 获取升降状态
 * @return 升降状态结构体指针
 */
Lift_Status_t *Lift_GetStatus(void)
{
    return &lift_status;
}

/**
 * @brief 获取当前位移
 * @return 当前位移 (cm)，负值表示向下位移
 */
float Lift_GetCurrentDisplacement(void)
{
    return lift_status.current_displacement;
}

/**
 * @brief 电机角度转换为位移
 * @param angle 电机角度 (度)
 * @return 对应的位移 (cm)
 * @note 计算公式: 位移 = (角度 / 360度 / 减速比) * 主动轮周长 * 100
 */
static float Lift_AngleToDisplacement(float angle)
{
    // 角度转换为位移，考虑36:1减速比
    // 位移[m] = (角度 / 360度 / 减速比) * 轮周长[m]
    float displacement_m = (angle / 360.0f / LIFT_GEAR_RATIO) * LIFT_WHEEL_CIRCUMFERENCE;
    return displacement_m * 100.0f; // m转换为cm
}

/**
 * @brief 位移转换为电机角度
 * @param displacement 位移 (cm)
 * @return 对应的电机角度 (度)
 * @note 计算公式: 角度 = (位移[m] / 轮周长[m]) * 360度 * 减速比
 */
static float Lift_DisplacementToAngle(float displacement)
{
    // 位移转换为角度，考虑36:1减速比
    // 角度 = (位移[m] / 轮周长[m]) * 360度 * 减速比
    float displacement_m = displacement / 100.0f; // cm转换为m
    float angle = (displacement_m / LIFT_WHEEL_CIRCUMFERENCE) * 360.0f * LIFT_GEAR_RATIO;
    return angle;
}

/**
 * @brief 更新当前速度（用于状态监控）
 */
static void Lift_UpdateCurrentSpeed(void)
{
    if (!lift_status.enabled) return;
    
    static float last_displacement = 0.0f;
    static uint32_t last_time = 0;
    
    uint32_t current_time = HAL_GetTick();
    
    // 初始化或时间间隔太小时跳过计算
    if (last_time == 0 || (current_time - last_time) < LIFT_TASK_PERIOD)
    {
        last_displacement = lift_status.current_displacement;
        last_time = current_time;
        return;
    }
    
    // 计算速度 = 位移变化 / 时间间隔
    float displacement_change = lift_status.current_displacement - last_displacement;
    float time_interval = (current_time - last_time) / 1000.0f; // ms转换为s
    
    lift_status.speed = displacement_change / time_interval; // cm/s
    
    last_displacement = lift_status.current_displacement;
    last_time = current_time;
}

/**
 * @brief 检查是否到达目标位置
 * @return true: 已到达, false: 未到达
 */
void Lift_WaitUntilAtTarget(void)
{
    const float tolerance = 0.5f; // 位移容差 0.5cm
    float error;

    do {
        error = fabsf(lift_status.current_displacement - lift_status.target_displacement);
        vTaskDelay(pdMS_TO_TICKS(10));  // 等待 10ms
    } while (error > tolerance);
}
/**
 * @brief 移动到指定位置
 * @param target_displacement 目标绝对位移 (cm)，负值表示向下位移
 * @retval 0: 成功, -1: 失败
 */
int Lift_MoveTo(float target_displacement)
{
    if (!lift_status.enabled)
    {
        printf("升降系统未使能!\r\n");
        return -1;
    }

    // 位移限制检查
    if (target_displacement > LIFT_MAX_DISPLACEMENT)
    {
        target_displacement = LIFT_MAX_DISPLACEMENT;
        printf("目标位移限制到最大值: %.1fcm\r\n", LIFT_MAX_DISPLACEMENT);
    }
    else if (target_displacement < LIFT_MIN_DISPLACEMENT)
    {
        target_displacement = LIFT_MIN_DISPLACEMENT;
        printf("目标位移限制到最小值: %.1fcm\r\n", LIFT_MIN_DISPLACEMENT);
    }

    lift_status.target_displacement = target_displacement;
    lift_status.is_moving = true;

    printf("升降移动到目标位移: %.2fcm\r\n", target_displacement);
    return 0;
}
void Lift_To_High1(void)
{
    lift_status.target_displacement = 165;
    Lift_WaitUntilAtTarget();

}
void Lift_To_High2(void)
{
    lift_status.target_displacement = 260;
    Lift_WaitUntilAtTarget();
}
void Lift_To_StartHeight(void)
{
    lift_status.target_displacement = 120;
    Lift_WaitUntilAtTarget();
}
void Lift_To_HighA(void)
{
    lift_status.target_displacement = 30;
    Lift_WaitUntilAtTarget();
}
void Lift_To_HighB(void)
{
    lift_status.target_displacement = 340;
    Lift_WaitUntilAtTarget();
}
void Lift_To_HighBMove(void)
{
    lift_status.target_displacement = 360;
    Lift_WaitUntilAtTarget();
}
void Lift_To_PUT2HIGH(void)
{
    lift_status.target_displacement = 240;
    Lift_WaitUntilAtTarget();
}
void Lift_To_PUTDown(void)
{
    lift_status.target_displacement = 110;
    Lift_WaitUntilAtTarget();
}
void Lift_To_PUTspecialDown(void)
{
    lift_status.target_displacement = 200;
    Lift_WaitUntilAtTarget();
}
void Lift_To_PUTspecialUP(void)
{
    lift_status.target_displacement = 260;
    Lift_WaitUntilAtTarget();
}
void Lift_To_PUTspecialUUP(void)
{
    lift_status.target_displacement = 300;
    Lift_WaitUntilAtTarget();
}
