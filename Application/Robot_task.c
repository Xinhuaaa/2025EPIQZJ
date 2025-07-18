/**
  ******************************************************************************
  * @file    Robot_task.c
  * @brief   机器人主控模块
  *************************************************************                                  LOGINFO("移动到路线                                  LOGINFO("移动到路线3-位置3\r\n");
                        Chassis_MoveToPosition_Blocking(300, 100, 0, 11); // 路线3的第3个放置点                 LOGINFO("移动到路线3-位置2\r\n");
                        Chassis_MoveToPosition_Blocking(200, 100, 0, 11); // 路线3的第2个放置点                 LOGINFO("移动到路线3-位置1\r\n");
                        Chassis_MoveToPosition_Blocking(100, 100, 0, 11); // 路线3的第1个放置点位置3\r\n");
                        Chassis_MoveToPosition_Blocking(300, -100, 0, 11); // 路线2的第3个放置点                 LOGINFO("移动到路线2-位置2\r\n");
                        Chassis_MoveToPosition_Blocking(200, -100, 0, 11); // 路线2的第2个放置点                 LOGINFO("移动到路线2-位置1\r\n");
                        Chassis_MoveToPosition_Blocking(100, -100, 0, 11); // 路线2的第1个放置点***************
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
#include "Encoder.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdlib.h"
#include "CDCDataParser.h" // 添加数据解析模块头文件
#include "Robot_task.h" // 添加自身的头文件，包含移动宏定义


uint8_t grabSequence[6] = {4,5,6,1,2,3};  // 6个抓取顺序
uint8_t specialBox = 2;         // 特殊货箱号
uint8_t route = 1;              // 运行路线
char usbData[64] = {0};         // USB接收数据缓冲区

int Robot_Init(void)
{   
    __disable_irq();
    DWT_Init(168);
    BSPLogInit();
    __enable_irq();
    DWT_Delay(1);
    HWT101_TaskInit();
    EncoderInit();
    Chassis_Init();
    Lift_Init();
    
    Crawl_Init(); 

    return 0;
}


void Robot_task(void *argument)
{
    osDelay(100);
    runActionGroup(2,1);
    osDelay(500);
    runActionGroup(3,1);
    osDelay(500);
    Lift_To_StartHeight();
    runActionGroup(5,1);//前面舵机放置
    osDelay(500);
    runActionGroup(6,1);//后面舵机放置
    osDelay(500);
    /* 等待一段时间确保所有初始化完成 */
    // osDelay(500);
    
    // LOGINFO("Robot_task: 等待接收USB数据...\r\n");
    
    // while (1)
    // {
    //     if (USB_RxFlag == 1)
    //     {
    //         // 复制接收到的数据
    //         memcpy(usbData, UserRxBufferFS, USB_RxLen);
    //         usbData[USB_RxLen] = '\0'; // 确保字符串以NULL结尾
      
    //         // 清除接收标志
    //         USB_RxFlag = 0;
    //         USB_RxLen = 0;

    //         // 解析数据
    //         if (parseUsbData(usbData, grabSequence, &specialBox, &route) == 0)
    //         {

    //             for (int i = 0; i < 6; i++)
    //             {
    //                 LOGINFO("%d ", grabSequence[i]);
    //             }
                
    //             // 数据解析成功，退出等待循环
    //             break;
    //         }
    //         else
    //         {
    //             LOGINFO("数据格式错误，继续等待...\r\n");
    //         }
    //     }
        
    //     // 短暂延时，避免过度占用CPU
    //     vTaskDelay(100);
    // }

    // 抓取阶段
    // for (int x = 0; x < 6; x++) {
        
    //     switch (grabSequence[x]) {
    //         case 1:
    //             LOGINFO("抓取箱子类型1\r\n");
    //             MoveToCenter(); // 移动到中间
    //             MoveToLeft();   // 移动到左侧
    //             Crawl_GrabBox(1, x); // 使用抓取函数抓取1号位置的货箱，传入当前处理的箱子数量
    //             break;
                
    //         case 2:
    //             LOGINFO("抓取箱子类型2\r\n");
    //             MoveToCenter(); // 移动到中间
    //             Crawl_GrabBox(2, x); // 使用抓取函数抓取2号位置的货箱，传入当前处理的箱子数量
    //             break;
                
    //         case 3:
    //             LOGINFO("抓取箱子类型3\r\n");
    //             MoveToCenter(); // 移动到中间
    //             MoveToRight();  // 移动到右侧
    //             Crawl_GrabBox(3, x); // 使用抓取函数抓取3号位置的货箱，传入当前处理的箱子数量
    //             break;
                
    //         case 4:
    //             LOGINFO("抓取箱子类型4\r\n");
    //             MoveToCenter(); // 移动到中间
    //             MoveToLeft();   // 移动到左侧
    //             Crawl_GrabBox(4, x); // 使用抓取函数抓取4号位置的货箱，传入当前处理的箱子数量
    //             break;
                
    //         case 5:
    //             LOGINFO("抓取箱子类型5\r\n");
    //             MoveToCenter(); // 移动到中间
    //             Crawl_GrabBox(5, x); // 使用抓取函数抓取5号位置的货箱，传入当前处理的箱子数量
    //             break;
                
    //         case 6:
    //             LOGINFO("抓取箱子类型6\r\n");
    //             MoveToCenter(); // 移动到中间
    //             MoveToRight();  // 移动到右侧
    //             Crawl_GrabBox(6, x); // 使用抓取函数抓取6号位置的货箱，传入当前处理的箱子数量
    //     }        
    //     vTaskDelay(500); // 添加一些延时，以便观察运行过程
    // }

    // 放置阶段
    LOGINFO("开始放置阶段...\r\n");
    LOGINFO("选择路线: %d\r\n", route);
    
    // 计算特殊货箱在放置顺序中的索引（抓取顺序的逆序）
    uint8_t placementSpecialBox = 5 - specialBox;
    LOGINFO("特殊货箱抓取索引: %d，放置索引: %d\r\n", specialBox, placementSpecialBox);
    
    // 根据不同的路线选择不同的放置策略
    switch(route) {
        case 1:
            LOGINFO("执行路线1的放置策略\r\n");
            // 路线1的放置逻辑
            for(int i = 0; i < 6; i++) {
                LOGINFO("放置第%d个箱子\r\n", i+1);
                
                // 如果当前不是特殊箱子，才移动到新位置
                if(i != placementSpecialBox) {
                    // 根据当前箱子序号选择不同的放置位置
                    switch(i) {
                        case 0:
                            LOGINFO("移动到路线1-位置1\r\n");
                            MoveToA(); // 路线1的第1个放置点
                            break;
                        case 1:
                            LOGINFO("移动到路线1-位置2\r\n");
                            MoveToB(); // 路线1的第2个放置点
                            break;
                        case 2:
                            LOGINFO("移动到路线1-位置3\r\n");
                            MoveToC(); // 路线1的第3个放置点
                            break;
                        case 3:
                            LOGINFO("移动到路线1-位置4\r\n");
                            MoveToD(); // 路线1的第4个放置点
                            break;
                        case 4:
                            LOGINFO("移动到路线1-位置5\r\n");
                            MoveToE(); // 路线1的第5个放置点
                            break;
                        case 5:
                            LOGINFO("移动到路线1-位置6\r\n");
                            MoveToF(); // 路线1的第6个放置点
                            break;
                    }
                } else {
                    LOGINFO("特殊箱子(%d)，保持位置不变\r\n", placementSpecialBox);
                }
                
                // 放置箱子
                if(Crawl_PlaceBox(i, placementSpecialBox) == 0) {
                    LOGINFO("箱子%d放置成功\r\n", i+1);
                } else {
                    LOGINFO("箱子%d放置失败\r\n", i+1);
                }
                
                vTaskDelay(500);
            }
            break;
            
        case 2:
            LOGINFO("执行路线2的放置策略\r\n");
            // 路线2的放置逻辑
            for(int i = 0; i < 6; i++) {
                LOGINFO("放置第%d个箱子\r\n", i+1);
                
                // 如果当前不是特殊箱子，才移动到新位置
                if(i != placementSpecialBox) {
                    // 根据当前箱子序号选择不同的放置位置
                    switch(i) {
                        case 0:
                            MoveToA(); // 路线2的第1个放置点
                            break;
                        case 1:
                            MoveToB(); // 路线2的第2个放置点
                            break;
                        case 2:
                            MoveToC(); // 路线2的第3个放置点
                            break;
                        case 3:
                            MoveToD(); // 路线2的第4个放置点
                            break;
                        case 4:
                            MoveToE(); // 路线2的第5个放置点
                            break;
                        case 5:
                            MoveToF(); // 路线2的第6个放置点
                            break;
                    }
                } else {
                    LOGINFO("特殊箱子(%d)，保持位置不变\r\n", placementSpecialBox);
                }
                
                // 放置箱子
                if(Crawl_PlaceBox(i, placementSpecialBox) == 0) {
                } else {
                }
                
                vTaskDelay(500);
            }
            break;
            
        case 3:
            for(int i = 0; i < 6; i++) {
                LOGINFO("放置第%d个箱子\r\n", i+1);
                
                // 如果当前不是特殊箱子，才移动到新位置
                if(i != placementSpecialBox) {
                    switch(i) {
                        case 0:
                            MoveToA(); // 路线3的第1个放置点
                            break;
                        case 1:
                            MoveToB(); // 路线3的第2个放置点
                            break;
                        case 2:
                            MoveToC(); // 路线3的第3个放置点
                            break;
                        case 3:
                            MoveToD(); // 路线3的第4个放置点
                            break;
                        case 4:
                            MoveToE(); // 路线3的第5个放置点
                            break;
                        case 5:
                            MoveToF(); // 路线3的第6个放置点
                            break;
                    }
                } else {
                    LOGINFO("特殊箱子(%d)，保持位置不变\r\n", placementSpecialBox);
                }
                
                // 放置箱子
                if(Crawl_PlaceBox(i, placementSpecialBox) == 0) {
                } else {
                }
                
                vTaskDelay(500);
            }
            break;
            
        default:
            for(int i = 0; i < 6; i++) {
                LOGINFO("放置第%d个箱子\r\n", i+1);
                
                // 如果当前不是特殊箱子，才移动到新位置
                if(i != placementSpecialBox) {
                    switch(i) {
                        case 0:
                            MoveToA(); // 默认路线的第1个放置点
                            break;
                        case 1:
                            MoveToB(); // 默认路线的第2个放置点
                            break;
                        case 2:
                            MoveToC(); // 默认路线的第3个放置点
                            break;
                        case 3:
                            MoveToD(); // 默认路线的第4个放置点
                            break;
                        case 4:
                            MoveToE(); // 默认路线的第5个放置点
                            break;
                        case 5:
                            MoveToF(); // 默认路线的第6个放置点
                            break;
                    }
                } else {
                    LOGINFO("特殊箱子(%d)，保持位置不变\r\n", placementSpecialBox);
                }
                Crawl_PlaceBox(i, placementSpecialBox);

                vTaskDelay(500);
            }
    }
    
    LOGINFO("任务完成\r\n");
    
    while(1) {
        vTaskDelay(1000);
    }
}