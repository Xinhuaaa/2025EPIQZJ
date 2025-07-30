/**
 ******************************************************************************
 * @file    Robot_task.c
 * @brief   机器人主控模块
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
uint8_t grabSequence[6] = {5, 6, 4, 2, 1, 3}; // 6个抓取顺序
uint8_t specialBox = 1;                       // 特殊货箱号
uint8_t spareStack = 2;                       // 轮空纸垛
char usbData[64] = {0};                       // USB接收数据缓冲区

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
    runActionGroup(2, 1);
    runActionGroup(3, 1);
    Lift_To_StartHeight();
    runActionGroup(5, 1);
    runActionGroup(6, 1);
    /* 等待一段时间确保所有初始化完成 */

    LOGINFO("Robot_task: 等待接收USB数据...\r\n");

    // while (1)
    // {
    //         if (USB_RxFlag == 1)
    //         {
    //         // 复制接收到的数据
    //         memcpy(usbData, UserRxBufferFS, USB_RxLen);
    //         usbData[USB_RxLen] = '\0'; // 确保字符串以NULL结尾

    //         // 清除接收标志
    //         USB_RxFlag = 0;
    //         USB_RxLen = 0;

    //         // 解析数据
    //         if (parseUsbData(usbData, grabSequence, &specialBox, &spareStack) == 0)
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
    //         }

    //         // 短暂延时，避免过度占用CPU
    //         vTaskDelay(100);
    // }

    // 抓取阶段
    MoveToCenter();
    MoveToCenter();
    MoveToCenter();
    for (int x = 0; x < 6; x++)
    {
        // 确定下一个箱子编号
        int next_box = (x < 5) ? grabSequence[x + 1] : 0;

        switch (grabSequence[x])
        {
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

            MoveToLeft(); // 移动到左侧
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
                MoveToRight(); // 移动到右侧
                MoveToRight();
                MoveToRight();
                MoveToRight(); // 移动到右侧
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
            MoveToLeft(); // 移动到左侧
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
            MoveToRight(); // 移动到右侧

            Crawl_GrabBox(6, x, next_box);
            break;
        }
        vTaskDelay(500);
    }
    Chassis_SetXPIDParams(0.55f, 0.001f, 0.0f);
    Chassis_MoveToY_Blocking(0, 0);
    Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 0);
    //  放置阶段
    //     placementSpecialBox：放置的第几个箱子是特殊箱子
    uint8_t placementSpecialBox = specialBox;
    int skip_position = 0;
    int box_counter = 0;
    switch (spareStack <= 5)
    {
    case 1: // spareStack = 1-5 (区域A-E)
        for (int i = 0; i < 6; i++)
        {
            int is_reverse_place = (i == 5 && spareStack != 1) ? 1 : 0;

            switch (spareStack)
            {
            case 1:
                skip_position = 5;
                break; // A区
            case 2:
                skip_position = 0;
                break; // B区
            case 3:
                skip_position = 1;
                break; // C区
            case 4:
                skip_position = 2;
                break; // D区
            case 5:
                skip_position = 3;
                break; // E区
            }

            if (i == skip_position)
            {
                continue;
            }

            // 普通箱子位置：先移动底盘到对应位置
            switch (i)
            {
            case 0:
                MoveToB();
                break; // 第1个箱子 -> B区
            case 1:
                MoveToC();
                break; // 第2个箱子 -> C区
            case 2:
                MoveToD();
                break; // 第3个箱子 -> D区
            case 3:
                MoveToE();
                break; // 第4个箱子 -> E区
            case 4:
                MoveToF();
                break; // 第5个箱子 -> F区
            case 5:
                MoveToA();
                break; // 第6个箱子 -> A区
            }

            // 判断是否是反向放置（A位置 i=5 且备用堆栈不是A区）

            // 执行普通放置
            Crawl_PlaceBox(box_counter, placementSpecialBox, is_reverse_place);
            box_counter += 1;
            if (box_counter == placementSpecialBox)
            {
                // 如果是特殊箱子位置，执行叠放处理
                Crawl_PlaceBox(box_counter, placementSpecialBox, is_reverse_place);
                box_counter += 1;
                Lift_To_PUTspecialUUP();
            }
            // 如果是E位置（i==3），执行额外的移动操作
            if (i == 3)
            {
                Lift_To_PUTspecialUUP();
                Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 0);
            }
        }

        break;

    case 0: // spareStack = 6 (区域F)
        for (int i = 0; i < 6; i++)
        {
            // 判断是否是反向放置（A位置 i=5）
            int is_reverse_place = (i == 5) ? 1 : 0;
            // 跳过F区位置（i=4，第5个箱子位置对应F区）
            if (i == 4)
            {
                continue;
            }

            // 普通箱子位置：先移动底盘到对应位置
            switch (i)
            {
            case 0:
                MoveToE();
                break; // 第1个箱子 -> E区
            case 1:
                MoveToD();
                break; // 第2个箱子 -> D区
            case 2:
                MoveToC();
                break; // 第3个箱子 -> C区
            case 3:
                MoveToB();
                break; // 第4个箱子 -> B区
            case 4:    // 第5个箱子 -> F区 (已跳过)
                break;
            case 5: // 第6个箱子 -> A区 (反向放置)
                Lift_To_PUTspecialUUP();
                Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 0);
                MoveToA();
                break;
            }

            // 执行普通放置
            Crawl_PlaceBox(box_counter, placementSpecialBox, is_reverse_place);
            box_counter += 1;

            if (box_counter == placementSpecialBox)
            {
                // 如果是特殊箱子位置，执行叠放处理
                Crawl_PlaceBox(box_counter, placementSpecialBox, is_reverse_place);
                Lift_To_PUTspecialUUP();
                box_counter += 1;
            }

        }

        break;
    }
                osDelay(2500); // 等待放置完成
                Lift_To_PUTspecialUUP();
                Chassis_MoveToPosition_Blocking(0, 0, 0, 0);
    while (1)
    {
        osDelay(1000);
    }
}
