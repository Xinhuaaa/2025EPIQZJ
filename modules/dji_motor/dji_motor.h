/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件 (已合并motor_def.h并精简为仅M2006相关代码)
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "PID.h"
#include "stdint.h"
#include "stdbool.h"

#define DJI_MOTOR_CNT 12
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED=0,
    OTHER_FEED,
} Feedback_Source_e;

/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

/* 电机类型枚举 - 只保留M2006相关 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    M2006,
} Motor_Type_e;

/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct
{
    Closeloop_Type_e outer_loop_type;        // 最外层的闭环,未设置时默认为最高级的闭环
    Closeloop_Type_e close_loop_type;        // 使用几个闭环(串级)
    Motor_Reverse_Flag_e motor_reverse_flag; // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source; // 角度反馈类型
    Feedback_Source_e speed_feedback_source; // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;      // 前馈标志

} Motor_Control_Setting_s;

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
typedef struct
{
    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;

    float *speed_feedforward_ptr;   // 速度前馈数据指针
    float *current_feedforward_ptr; // 电流前馈数据指针

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
} Motor_Controller_s;

/**
 * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
 *        如果不需要某个控制环,可以不设置对应的pid config
 *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
 */
typedef struct
{
    float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float *speed_feedforward_ptr;   // 速度前馈数据指针
    float *current_feedforward_ptr; // 电流前馈数据指针

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
} Motor_Init_Config_s;

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    uint32_t feed_cnt;
    float dt;
} DJIMotorInstance;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention C610(M2006)的反馈报文是0x200+id,请注意不要产生ID冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

#endif // !DJI_MOTOR_H