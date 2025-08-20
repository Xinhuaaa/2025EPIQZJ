/**
 ******************************************************************************
 * @file    Crawl.h
 * @brief   抓取结构控制系统头文件
 * @author  
 * @date    2025-05-30
 ******************************************************************************
 * @attention
 *
 * 抓取结构控制系统，集成升降、步进电机前后运动和舵机抓取动作控制
 * 
 * 主要功能：
 * - 升降结构控制（基于M2006电机）
 * - 步进电机前后伸缩控制（基于EMM V5步进电机）
 * - 舵机抓取动作控制（基于Lobot舵机控制板）
 * 
 ******************************************************************************
 */

#ifndef __CRAWL_H
#define __CRAWL_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含的头文件 */
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdio.h"

/* 包含相关模块头文件 */
#include "lift.h"           // 升降结构控制
#include "Emm_V5.h"         // 步进电机控制
#include "servo.h"          // 舵机控制

/* 宏定义 */
#define CRAWL_TASK_PERIOD       100          // 抓取任务周期，单位ms
#define CRAWL_MAX_WAIT_TIME     10000       // 最大等待时间，单位ms
#define CRAWL_DEFAULT_TIMEOUT   5000        // 默认超时时间，单位ms

/* 步进电机参数 */
#define CRAWL_STEPPER_FRONT_ID  1           // 前步进电机ID
#define CRAWL_STEPPER_BACK_ID   2           // 后步进电机ID
#define CRAWL_STEPPER_SPEED     500         // 步进电机默认速度 (RPM)
#define CRAWL_STEPPER_ACC       50          // 步进电机加速度
#define CRAWL_EXTEND_PULSES     10000       // 前伸脉冲数
#define CRAWL_RETRACT_PULSES    10000       // 收回脉冲数

/* 舵机参数 */
#define CRAWL_SERVO_ACTION_GRAB     1       // 抓取动作组编号
#define CRAWL_SERVO_ACTION_RELEASE  2       // 释放动作组编号
#define CRAWL_SERVO_TIMES           1       // 动作组执行次数
#define CRAWL_ACTION_NONE            0       // 无动作

/* 升降参数 */
#define CRAWL_LIFT_DEFAULT_HEIGHT   10.0f   // 默认升降高度，单位cm
#define CRAWL_LIFT_MIN_HEIGHT       0.0f    // 最小升降高度，单位cm
#define CRAWL_LIFT_MAX_HEIGHT       50.0f   // 最大升降高度，单位cm

/* 枚举定义 */
typedef enum {
    CRAWL_STATE_IDLE = 0,           // 空闲状态
    CRAWL_1,
    CRAWL_2,
    CRAWL_3,
    CRAWL_4,
    CRAWL_5,
    CRAWL_6,
    PUT_1,                          // 放置动作1
    PUT_2,                          // 放置动作2
    PUT_3,                          // 放置动作3
    PUT_4,                          // 放置动作4
    PUT_5,                          // 放置动作5
    PUT_6,                          // 放置动作6
    Test
} Crawl_State_t;

typedef enum {
    CRAWL_ACTION_GRAB = 0,          // 抓取动作
    CRAWL_ACTION_RELEASE,           // 释放动作
    CRAWL_ACTION_GRAB_AND_LIFT,     // 抓取并上升
    CRAWL_ACTION_PLACE_AND_LOWER    // 放置并下降
} Crawl_Action_t;

/* 结构体定义 */
typedef struct {
    Crawl_State_t state;            // 当前状态
    Crawl_Action_t action;          // 当前动作
    bool is_busy;                   // 是否忙碌
    bool is_enabled;                // 是否使能
    float target_height;            // 目标高度，单位cm
    uint32_t start_time;            // 开始时间
    uint32_t timeout;               // 超时时间
    int error_code;                 // 错误代码
} Crawl_Status_t;

/* 全局变量声明 */
extern Crawl_Status_t crawl_status;
extern osThreadId_t crawlTaskHandle;

/* 函数声明 */

/**
 * @brief 抓取系统初始化
 * @retval 0: 成功, -1: 失败
 */
int Crawl_Init(void);

/**
 * @brief 抓取控制任务函数
 * @param argument 任务参数
 */
void CrawlTask(void *argument);

/**
 * @brief 执行抓取动作（升降到指定高度 -> 前伸 -> 抓取）
 * @param height 升降目标高度，单位cm
 * @param timeout 超时时间，单位ms，0表示使用默认超时
 * @retval 0: 成功启动, -1: 失败
 */
int Crawl_GrabAction(float height, uint32_t timeout);

/**
 * @brief 执行释放动作（释放 -> 收回 -> 下降）
 * @param timeout 超时时间，单位ms，0表示使用默认超时
 * @retval 0: 成功启动, -1: 失败
 */
int Crawl_ReleaseAction(uint32_t timeout);

/**
 * @brief 执行抓取并上升动作
 * @param grab_height 抓取时的高度，单位cm
 * @param lift_height 抓取后的升降高度，单位cm
 * @param timeout 超时时间，单位ms，0表示使用默认超时
 * @retval 0: 成功启动, -1: 失败
 */
int Crawl_GrabAndLift(float grab_height, float lift_height, uint32_t timeout);

/**
 * @brief 执行放置并下降动作
 * @param place_height 放置时的高度，单位cm
 * @param lower_height 放置后的下降高度，单位cm
 * @param timeout 超时时间，单位ms，0表示使用默认超时
 * @retval 0: 成功启动, -1: 失败
 */
int Crawl_PlaceAndLower(float place_height, float lower_height, uint32_t timeout);

/**
 * @brief 停止当前抓取动作
 * @retval 0: 成功, -1: 失败
 */
int Crawl_Stop(void);

/**
 * @brief 急停所有运动
 * @retval 0: 成功, -1: 失败
 */
int Crawl_EmergencyStop(void);

/**
 * @brief 获取抓取系统状态
 * @return 抓取系统状态结构体指针
 */
Crawl_Status_t *Crawl_GetStatus(void);

/**
 * @brief 检查抓取动作是否完成
 * @return true: 完成, false: 未完成
 */
bool Crawl_IsCompleted(void);

/**
 * @brief 检查抓取系统是否忙碌
 * @return true: 忙碌, false: 空闲
 */
bool Crawl_IsBusy(void);

/**
 * @brief 重置抓取系统状态
 * @retval 0: 成功, -1: 失败
 */
int Crawl_Reset(void);

/**
 * @brief 设置抓取系统使能状态
 * @param enabled true: 使能, false: 禁用
 * @retval 0: 成功, -1: 失败
 */
int Crawl_SetEnabled(bool enabled);

/* 内部功能函数 */

/**
 * @brief 步进电机前伸
 * @retval 0: 成功, -1: 失败
 */
int Crawl_StepperExtend(void);

/**
 * @brief 步进电机收回
 * @retval 0: 成功, -1: 失败
 */
int Crawl_StepperRetract(void);

/**
 * @brief 舵机抓取
 * @retval 0: 成功, -1: 失败
 */
int Crawl_ServoGrab(void);

/**
 * @brief 舵机释放
 * @retval 0: 成功, -1: 失败
 */
int Crawl_ServoRelease(void);

/**
 * @brief 检查升降是否到达目标位置
 * @return true: 到达, false: 未到达
 */
bool Crawl_IsLiftReached(void);

/**
 * @brief 检查步进电机是否运动完成
 * @return true: 完成, false: 未完成
 */
bool Crawl_IsStepperReached(void);

/**
 * @brief 抓取箱子接口函数
 * @param box_number 箱子编号 (1-6)
 *                   1-3: 上层箱子
 *                   4-6: 下层箱子
 *                   排列方式：
 *                   1 2 3
 *                   4 5 6
 * @param box_count 当前已放置的箱子数量 (0-5)
 * @param next_box_number 下一个箱子编号 (1-6, 0表示无下一个箱子)
 * @retval 0: 成功, -1: 失败
 */
int Crawl_GrabBox(int box_number, int box_count, int next_box_number,int spareStack);
/**
 * @brief 放置箱子接口函数
 * @param box_count 当前已放置的箱子数量 (0-5)
 * @param special_box 特殊箱子编号 (0表示无特殊箱子)
 * @retval 0: 成功, -1: 失败
 */
int Crawl_PlaceBox(int box_count, int special_box, int is_reverse_place);

#ifdef __cplusplus
}
#endif

#endif /* __CRAWL_H */