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
#include "bsp_key.h"
uint8_t grabSequence[6] = {1, 6, 4, 2, 5, 3}; // 6个抓取顺序
uint8_t specialBox = 1;                       // 特殊货箱号
uint8_t spareStack = 1;                       // 轮空纸垛
char usbData[64] = {0};                       // USB接收数据缓冲区

int Robot_Init(void)
{
    __disable_irq();
    DWT_Init(168);
    BSPLogInit();
    __enable_irq();
    Key_Init();
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
    bool usb_ready = true;
    bool key_pressed = false;

    runActionGroup(2, 1);
    runActionGroup(3, 1);
    key_event_t evt;

    while (!(usb_ready && key_pressed))
    {
        // USB 接收处理
        if (!usb_ready && USB_RxFlag == 1)
        {
            memcpy(usbData, UserRxBufferFS, USB_RxLen);
            usbData[USB_RxLen] = '\0';

            USB_RxFlag = 0;
            USB_RxLen = 0;

            if (parseUsbData(usbData, grabSequence, &specialBox, &spareStack) == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    LOGINFO("%d ", grabSequence[i]);
                }
                usb_ready = true;
            }
        }

        if (osMessageQueueGet(key_event_queue, &evt, NULL, 0) == osOK)
        {
            if (evt.event == KEY_EVENT_PRESSED)
            {
                key_pressed = true;
                const char msg = 'T';
                CDC_Transmit_FS((uint8_t *)&msg, 1);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    Lift_To_StartHeight();
    runActionGroup(5, 1);
    runActionGroup(6, 1);

    // 抓取阶段

    for (int x = 0; x < 6; x++)
    {
        // 确定下一个箱子编号
        int next_box = (x < 5) ? grabSequence[x + 1] : 0;

        switch (grabSequence[x])
        {
        case 1:
            if (x == 0)
            {
                STARTMoveToLeft();
                osDelay(2000);

            }

            MoveToLeft(); // 移动到左侧
            Crawl_GrabBox(1, x, next_box);
            break;

        case 2:
            if (x == 0)
            {
                STARTMoveToCenter();
                osDelay(2000);
            }
            MoveToCenter(); // 移动到中间
            Crawl_GrabBox(2, x, next_box);
            break;

        case 3:
            if (x == 0)
            {
                STARTMoveToRight();
                osDelay(2000);

            }
            MoveToRight();
            Crawl_GrabBox(3, x, next_box);
            break;

        case 4:
            if (x == 0)
            {
                STARTMoveToLeft();
                osDelay(2000);

            }
            MoveToLeft(); // 移动到左侧
            Crawl_GrabBox(4, x, next_box);
            break;

        case 5:
            if (x == 0)
            {
                STARTMoveToCenter();
                 osDelay(2000);

            }
            MoveToCenter(); // 移动到中间
            Crawl_GrabBox(5, x, next_box);
            break;

        case 6:
            if (x == 0)
            {
                STARTMoveToRight();
                osDelay(2000);

            }
            MoveToRight(); // 移动到右侧

            Crawl_GrabBox(6, x, next_box);
            break;
        }
        vTaskDelay(500);
    }
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
                MoveToB1();
                break;
            case 1:
                MoveToC1();
                break;
            case 2:
                MoveToD1();
                break;
            case 3:
                MoveToE1();
                break;
            case 4:
                MoveToF1();
                break;
            case 5:
                MoveToA1();
                break;
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
                Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 4500);
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
                MoveToE0();
                break;
            case 1:
                MoveToD0();
                break;
            case 2:
                MoveToC0();
                break;
            case 3:
                MoveToB0();
                break;
            case 4:
                break;
            case 5:
                Lift_To_PUTspecialUUP();
                Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 0);
                MoveToA0();
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
    Chassis_SetXPIDParams(0.83f,0.01f,0.0f );
    Chassis_MoveToPosition_Blocking(0, 0.0, 0, 30000);
    lift_status.target_displacement = 12;
    while (1)
    {
        osDelay(1000);
    }
}
