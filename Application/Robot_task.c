/**
  ******************************************************************************
  * @file    Robot_task.c
  * @brief   机器人主控模块
  ******************************************************************************
  * @attention
  *
  * 描述：机器人系统初始化，负责调用各个模块的初始化函数
  *
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "chassis.h"
#include "lift.h"
#include "Crawl.h"
#include "dji_motor.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "Hwt101.h"
#include "emm_v5.h"

int Robot_Init(void)
{   
    __disable_irq();
    DWT_Init(168);
    BSPLogInit();
    __enable_irq();
    DWT_Delay(1);
    HWT101_TaskInit();

    Chassis_Init();
    Lift_Init();
    
    Crawl_Init(); 

    return 0;
}
void Robot_task()
{
    //机器人主任务（逻辑性的任务）
    // Chassis_SetTargetPosition(1.64f, 0.0f, 0.0f);
    // if (target[0] == 1 || target[1] == 2 || target[0] == 1 || target[1] == 2)
    // {
    //     Chassis_SetTargetPosition(1.64f, -0.5f, 0.0f);
    //     if (target[0] == 1)
    //     {
    //       Lift_MoveTo(10.0f); // 升降移动到10cm位置
    //       while(!Lift_IsAtTarget());
    //     }
    //     else if (target[1]==2)
    //     {
    //       Lift_MoveTo(20.0f); // 升降移动到20cm位置
    //       while(!Lift_IsAtTarget());
    //     }
        
    // }
    // {
        
    // }
    
}