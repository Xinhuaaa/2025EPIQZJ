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
#include "CDCDataParser.h" 
#include "Robot_task.h" 
uint8_t grabSequence[6] = {1,3,5,6,2,4};  // 6个抓取顺序
uint8_t specialBox = 3;         // 特殊货箱号
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
    runActionGroup(2,1);
    runActionGroup(3,1);
    Lift_To_StartHeight();
    runActionGroup(5,1);//前面舵机放置
    runActionGroup(6,1);//后面舵机放置
    /* 等待一段时间确保所有初始化完成 */
    
    LOGINFO("Robot_task: 等待接收USB数据...\r\n");
    
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
    MoveToCenter();
    MoveToCenter();
    MoveToCenter();
    for (int x = 0; x < 6; x++) {
        // 确定下一个箱子编号
        int next_box = (x < 5) ? grabSequence[x + 1] : 0;
        
        switch (grabSequence[x]) {
            case 1:
            if (x == 0)
            {
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
            }
            
                MoveToLeft();   // 移动到左侧
                Crawl_GrabBox(1, x, next_box);
                break;
                
            case 2:
                        if (x == 0)
            {
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
            }
                MoveToCenter(); // 移动到中间
                Crawl_GrabBox(2, x, next_box);
                break;
                
            case 3:
                if (x == 0)
            {
                MoveToRight();  // 移动到右侧
                MoveToRight();
                MoveToRight();
                MoveToRight();  // 移动到右侧
                MoveToRight();
                MoveToRight();
            }
                MoveToRight();
                Crawl_GrabBox(3, x, next_box);
                break;
                
            case 4:
                if (x == 0)
            {
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
                MoveToLeft();
            }
                MoveToLeft();   // 移动到左侧
                Crawl_GrabBox(4, x, next_box);
                break;
                
            case 5:
                if (x == 0)
            {
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
                MoveToCenter();
            }
                MoveToCenter(); // 移动到中间
                Crawl_GrabBox(5, x, next_box);
                break;
                
            case 6:
                if (x == 0)
            {
                MoveToRight();
                MoveToRight();
                MoveToRight();
                MoveToRight();
                MoveToRight();
                MoveToRight();
            }
                MoveToRight();  // 移动到右侧
                
                Crawl_GrabBox(6, x, next_box);
                break;
        }        
        vTaskDelay(500);
    }
Chassis_SetXPIDParams(0.55f, 0.001f, 0.0f);
Chassis_MoveToY_Blocking(0,0);
Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 0); 
//     // 放置阶段
    // route：放置的第几个箱子
    // placementSpecialBox：放置的第几个箱子是特殊箱子
    uint8_t placementSpecialBox = 5 - specialBox;
        switch(route) {
        case 1:
            for(int i = 0; i < 6; i++) {                
                if(i != placementSpecialBox) {
                    switch(i) {
                        case 0:
                            MoveToB();
                            break;
                        case 1:
                            MoveToC();
                            break;
                        case 2:
                            MoveToD(); 
                            break;
                        case 3:
                            MoveToE();
                            Lift_To_PUTspecialUUP();
                            break;
                        case 4:
                            MoveToF();
                            break;
                        case 5:
                            MoveToA();
                            break;
                    }
                }
                // 如果是A位置（路线1的第6个箱子），使用反向放置
                int is_reverse_place = (i == 5 && route == 1 && i != placementSpecialBox) ? 1 : 0;
                Crawl_PlaceBox(i, placementSpecialBox, is_reverse_place);
                if(i == placementSpecialBox) {
                Lift_To_PUTspecialUUP();
                }
            }
            break;
            
        case 2:
            for(int i = 0; i < 6; i++) {                
                if(i != placementSpecialBox) {
                    switch(i) {
                        case 0:
                            MoveToB(); 
                            break;
                        case 1:
                            MoveToC(); 
                            break;
                        case 2:
                            MoveToD(); 
                            break;
                        case 3:
                            MoveToE(); 
                            Lift_To_PUTspecialUUP();
                            break;
                        case 4:
                            MoveToF(); 
                            break;
                        case 5:
                            MoveToA(); 
                            break;
                    }
                }
                int is_reverse_place = (i == 5 && route == 2 && i != placementSpecialBox) ? 1 : 0;
                Crawl_PlaceBox(i, placementSpecialBox, is_reverse_place);
                if(i == placementSpecialBox) {
                Lift_To_PUTspecialUUP();
                }
            }

            break;
            
        case 3:
            for(int i = 0; i < 6; i++) {
                if(i != placementSpecialBox) {
                    switch(i) {
                        case 0:
                            MoveToE(); 
                            break;
                        case 1:
                            MoveToD();
                            break;
                        case 2:
                            MoveToC();
                            break;
                        case 3:
                            MoveToB(); 
                            break;
                        case 4:
                            Lift_To_PUTspecialUUP();
                            Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 0); 
                            MoveToA(); 
                            break;
                        case 5:
                            MoveToF();
                        break;
                    }
                }
                int is_reverse_place = (i == 4 && route == 3 && i != placementSpecialBox) ? 1 : 0;
                Crawl_PlaceBox(i, placementSpecialBox, is_reverse_place);
                if(i == placementSpecialBox) {
                Lift_To_PUTspecialUUP();
                }
            }
            break;
            
    }

    while(1) {
        vTaskDelay(1000);
    }
}