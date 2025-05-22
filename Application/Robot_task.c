/**
  ******************************************************************************
  * @file    Robot_task.c
  * @brief   机器人任务实现
  ******************************************************************************
  * @attention
  *
  * 描述：实现机器人主控任务，包括底盘导航控制
  *
  ******************************************************************************
  */

  #include "FreeRTOS.h"
  #include "task.h"
  #include "main.h"
  #include "cmsis_os.h"
  #include <stdio.h>
  #include <string.h>
  #include <stdlib.h>
  #include <math.h>
  #include "usart.h"
  #include "chassis.h"
  #include "Hwt101.h"
  #include "Emm_V5_CAN.h"
  
  /* 任务句柄 */
  osThreadId_t chassisTaskHandle;
  osThreadId_t navigationTaskHandle;
  
  /* 队列句柄 */
  osMessageQueueId_t navigationQueueHandle;
  
  /* 导航点结构体 */
  typedef struct {
      float x;      // X坐标，单位m
      float y;      // Y坐标，单位m
      float yaw;    // 偏航角，单位度
  } NavigationPoint_t;
  
  /* 预定义导航路径点 */
  const NavigationPoint_t predefinedPath[] = {
      {0.4f, 0.4f, 90.0f},     // 向前移动1米
      {0.4f, 0.4f, 90.0f},    // 向右移动1米并旋转90度
      {0.4f, 0.4f, 90.0f},   // 向后移动1米并旋转到180度
      {0.4f, 0.4f, 90.0f}      // 回到原点并恢复0度方向
  };
  
  /**
    * @brief  底盘控制任务函数
    * @param  argument 任务参数
    * @retval 无
    */
  void ChassisTask(void *argument)
  {
      // 等待系统启动稳定
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 初始化底盘控制
      Chassis_Init();
      
      // 重置底盘位置为原点
      Chassis_ResetPosition();
      
      // 打印初始化完成信息
      printf("底盘初始化完成，准备就绪!\r\n");
      
      // 循环执行底盘控制
      for(;;)
      {
          // 执行底盘控制循环，周期20ms
          Chassis_Control_Loop();
          
          // 获取并打印当前位置（每1秒打印一次）
          static uint32_t printTick = 0;
          if (xTaskGetTickCount() - printTick > pdMS_TO_TICKS(1000))
          {
              float x, y, yaw;
              Chassis_GetCurrentPosition(&x, &y, &yaw);
              printf("当前位置: X=%.2f m, Y=%.2f m, Yaw=%.2f 度\r\n", x, y, yaw);
              printTick = xTaskGetTickCount();
          }
          
          // 周期性执行，确保实时控制
          vTaskDelay(pdMS_TO_TICKS(10));
      }
  }
  
  /**
    * @brief  导航任务函数，负责规划路径并发送导航点给底盘控制任务
    * @param  argument 任务参数
    * @retval 无
    */
  void NavigationTask(void *argument)
  {
      // 等待底盘初始化完成
      vTaskDelay(pdMS_TO_TICKS(2000));
      
      // 导航参数
      bool navigationActive = false;
      uint8_t currentPointIndex = 0;
      const uint8_t totalPoints = sizeof(predefinedPath) / sizeof(NavigationPoint_t);
      NavigationPoint_t targetPoint;
      
      // 任务主循环
      for(;;)
      {
          // 检查是否有接收到命令启动导航
          if (!navigationActive && currentPointIndex < totalPoints)
          {
              // 此处可以添加接收命令的代码，例如通过串口接收
              // 这里简化为自动开始导航
              navigationActive = true;
              printf("开始导航路径...\r\n");
              
              // 设置第一个导航点
              targetPoint = predefinedPath[currentPointIndex];
              Chassis_SetTargetPosition(targetPoint.x, targetPoint.y, targetPoint.yaw);
              
              printf("前往导航点 %d: X=%.2f m, Y=%.2f m, Yaw=%.2f 度\r\n",
                    currentPointIndex + 1, targetPoint.x, targetPoint.y, targetPoint.yaw);
          }
          
          // 如果导航活跃，检查是否到达当前目标点
          if (navigationActive)
          {
              float x, y, yaw;
              Chassis_GetCurrentPosition(&x, &y, &yaw);
              
              // 计算与目标点的距离和角度差
              float distanceError = sqrtf((targetPoint.x - x) * (targetPoint.x - x) + 
                                        (targetPoint.y - y) * (targetPoint.y - y));
              float yawError = fabsf(targetPoint.yaw - yaw);
              if (yawError > 180.0f) yawError = 360.0f - yawError;
              
              // 判断是否到达目标点（距离误差小于3cm，角度误差小于2度）
              if (distanceError < 0.02f && yawError < 1.5f)
              {
                  printf("到达导航点 %d\r\n", currentPointIndex + 1);
                  osDelay(pdMS_TO_TICKS(5000)); // 等待1秒钟
              if (distanceError < 0.02f && yawError < 1.5f)
              {
                  // 前往下一个导航点
                  currentPointIndex++;
              }
                  // 检查是否完成所有导航点
                  if (currentPointIndex >= totalPoints)
                  {
                      printf("完成全部导航路径!\r\n");
                      navigationActive = false;
                      currentPointIndex = 0; // 重置以便再次开始
                      
                      // 等待10秒钟后重新开始导航
                      vTaskDelay(pdMS_TO_TICKS(10000));
                  }
                  else
                  {
                      // 设置下一个导航点
                      targetPoint = predefinedPath[currentPointIndex];
                      Chassis_SetTargetPosition(targetPoint.x, targetPoint.y, targetPoint.yaw);
                      
                      printf("前往导航点 %d: X=%.2f m, Y=%.2f m, Yaw=%.2f 度\r\n",
                            currentPointIndex + 1, targetPoint.x, targetPoint.y, targetPoint.yaw);
                  }
              }
          }
          
          // 周期性执行，100ms检查一次
          vTaskDelay(pdMS_TO_TICKS(100));
      }
  }
  
  /**
    * @brief  创建机器人相关任务
    * @param  无
    * @retval 无
    */
  void Robot_TaskCreate(void)
  {
      // 创建底盘控制任务
      const osThreadAttr_t chassisTaskAttributes = {
          .name = "ChassisTask",
          .stack_size = 512 * 4,
          .priority = (osPriority_t) osPriorityNormal,
      };
      chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTaskAttributes);
      
      // 创建导航任务
      const osThreadAttr_t navigationTaskAttributes = {
          .name = "NavigationTask",
          .stack_size = 512 * 4,
          .priority = (osPriority_t) osPriorityBelowNormal,
      };
      navigationTaskHandle = osThreadNew(NavigationTask, NULL, &navigationTaskAttributes);
  }
  
  /**
    * @brief  测试底盘坐标导航控制
    * @param  无
    * @retval 无
    */
  void Chassis_TestNavigation(void)
  {
      // 初始化底盘
      Chassis_Init();
      
      // 重置位置
      Chassis_ResetPosition();
      
      printf("开始底盘导航测试...\r\n");
      
      // 前往第一个点(0.5m, 0, 0度)
      printf("前往点1: (0.5, 0, 0)\r\n");
      Chassis_SetTargetPosition(0.5f, 0.0f, 0.0f);
      while(!Chassis_Control_Loop())
      {
          vTaskDelay(pdMS_TO_TICKS(20));
      }
      printf("到达点1\r\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 前往第二个点(0.5m, 0.5m, 90度)
      printf("前往点2: (0.5, 0.5, 90)\r\n");
      Chassis_SetTargetPosition(0.5f, 0.5f, 90.0f);
      while(!Chassis_Control_Loop())
      {
          vTaskDelay(pdMS_TO_TICKS(20));
      }
      printf("到达点2\r\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 前往第三个点(0, 0.5m, 180度)
      printf("前往点3: (0, 0.5, 180)\r\n");
      Chassis_SetTargetPosition(0.0f, 0.5f, 180.0f);
      while(!Chassis_Control_Loop())
      {
          vTaskDelay(pdMS_TO_TICKS(20));
      }
      printf("到达点3\r\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 返回原点(0, 0, 0度)
      printf("返回原点: (0, 0, 0)\r\n");
      Chassis_SetTargetPosition(0.0f, 0.0f, 0.0f);
      while(!Chassis_Control_Loop())
      {
          vTaskDelay(pdMS_TO_TICKS(20));
      }
      printf("测试完成!\r\n");
  }