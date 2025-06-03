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

int Robot_Init(void)
{   
        __disable_irq();

    //BSP先初始化
    DWT_Init(168);
    BSPLogInit();
        __enable_irq();

    HWT101_TaskInit();
    //APP初始化
    Chassis_Init();
    Lift_Init();
    
    // Crawl_Init(); 
    Chassis_ResetPosition();

    return 0;
}
void Robot_task()
{
    
}