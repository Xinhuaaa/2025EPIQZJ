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
  #include "lift.h"
  #include "dji_motor.h"    /* 任务句柄 */
  osThreadId_t chassisTaskHandle;
  osThreadId_t navigationTaskHandle;
  osThreadId_t djiMotorTaskHandle;
  osThreadId_t liftTestTaskHandle;
  
  /* 队列句柄 */
  osMessageQueueId_t navigationQueueHandle;
  
  /* 导航点结构体 */
  typedef struct {
      float x;      // X坐标，单位m
      float y;      // Y坐标，单位m
      float yaw;    // 偏航角，单位度
} NavigationPoint_t;

void DJIMotorTask(void *argument);
void LiftTestTask(void *argument);

/**
  * @brief  DJI电机任务函数
  * @param  argument 任务参数
  * @retval 无
  */
void DJIMotorTask(void *argument)
 {
     // 等待系统启动稳定
     vTaskDelay(pdMS_TO_TICKS(500));
     
     printf("DJI电机任务启动\r\n");
     
     for(;;)
     {
         // 执行DJI电机控制循环
         DJIMotorControl();
         
         // 20ms周期执行
         vTaskDelay(pdMS_TO_TICKS(20));
     } }

/**
  * @brief  升降测试任务函数
  * @param  argument 任务参数
  * @retval 无
  */
void LiftTestTask(void *argument)
{
    // 等待系统启动稳定
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    printf("升降测试任务启动\r\n");
    
    // 执行升降测试
    Lift_TestControl();
    
    // 测试完成后删除任务
    printf("升降测试任务完成，任务即将结束\r\n");
    vTaskDelete(NULL);
}

  /* 预定义导航路径点 */
  const NavigationPoint_t predefinedPath[] = {
      {0.0f, 0.0f, 0.0f},     // 向前移动1米
      {0.0f, 0.0f, 0.0f},    // 向右移动1米并旋转90度
      {0.0f, 0.0f, 0.0f},   // 向后移动1米并旋转到180度
      {0.0f, 0.0f, 0.0f}      // 回到原点并恢复0度方向
  };
  
  /**
    * @brief  底盘控制任务函数
    * @param  argument 任务参数
    * @retval 无
    */  void ChassisTask(void *argument)
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
                  }              }
          }

          // 周期性执行，100ms检查一次
          vTaskDelay(pdMS_TO_TICKS(100));
      }
  }    /**
    * @brief  创建机器人相关任务
    * @param  无
    * @retval 无
    */  void Robot_TaskCreate(void)
  {
      // 初始化升降结构
      Lift_Init();
      printf("升降结构初始化完成\r\n");
      
      // 创建DJI电机任务 - 优先级最高，负责底层电机控制
      const osThreadAttr_t djiMotorTaskAttributes = {
          .name = "DJIMotorTask",
          .stack_size = 512 * 4,
          .priority = (osPriority_t) osPriorityAboveNormal,
      };      djiMotorTaskHandle = osThreadNew(DJIMotorTask, NULL, &djiMotorTaskAttributes);
      
      // 创建升降结构任务
      Lift_CreateTask();
      
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
      };      navigationTaskHandle = osThreadNew(NavigationTask, NULL, &navigationTaskAttributes);
  }

/**
  * @brief  启动升降测试任务
  * @param  无
  * @retval 无
  */  
void Robot_StartLiftTest(void)
{
    // 创建升降测试任务
    const osThreadAttr_t liftTestTaskAttributes = {
        .name = "LiftTestTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    liftTestTaskHandle = osThreadNew(LiftTestTask, NULL, &liftTestTaskAttributes);
    
    if (liftTestTaskHandle != NULL) {
        printf("升降测试任务已启动\r\n");
    } else {
        printf("升降测试任务启动失败\r\n");
    }
}
  
    /**
    * @brief  测试升降结构控制
    * @param  无
    * @retval 无
    */
  void Lift_TestControl(void)
  {
      printf("开始升降结构测试...\r\n");
      
      // 初始化升降结构
      Lift_Init();
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 测试移动到中间位置 (25cm)
      printf("升降到中间位置 (25cm)...\r\n");
      Lift_SetHeight(0.25f);
      
      // 等待移动完成
      while(!Lift_IsReached())
      {
          vTaskDelay(pdMS_TO_TICKS(100));
      }
      printf("到达中间位置\r\n");
      vTaskDelay(pdMS_TO_TICKS(2000));
      
      // 测试移动到顶部位置 (50cm)
      printf("升降到顶部位置 (50cm)...\r\n");
      Lift_SetHeight(0.5f);
      
      // 等待移动完成
      while(!Lift_IsReached())
      {
          vTaskDelay(pdMS_TO_TICKS(100));
      }
      printf("到达顶部位置\r\n");
      vTaskDelay(pdMS_TO_TICKS(2000));
      
      // 测试返回底部位置 (0cm)
      printf("升降到底部位置 (0cm)...\r\n");
      Lift_SetHeight(0.0f);
      
      // 等待移动完成
      while(!Lift_IsReached())
      {
          vTaskDelay(pdMS_TO_TICKS(100));
      }
      printf("到达底部位置\r\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 测试速度控制 - 向上移动
      printf("测试速度控制 - 向上移动...\r\n");
      Lift_Up(0.05f); // 5cm/s速度上升
      vTaskDelay(pdMS_TO_TICKS(3000)); // 3秒
      Lift_Stop();
      printf("停止上升运动\r\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 测试速度控制 - 向下移动
      printf("测试速度控制 - 向下移动...\r\n");
      Lift_Down(0.05f); // 5cm/s速度下降
      vTaskDelay(pdMS_TO_TICKS(2000)); // 2秒
      Lift_Stop();
      printf("停止下降运动\r\n");
      
      // 最终返回底部
      printf("最终返回底部...\r\n");
      Lift_SetHeight(0.0f);
      
      while(!Lift_IsReached())
      {
          vTaskDelay(pdMS_TO_TICKS(100));
      }
      
      printf("升降结构测试完成!\r\n");
  }