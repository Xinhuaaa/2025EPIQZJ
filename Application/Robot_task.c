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

// 坐标/姿态误差（通过 CDC 动态调整）
float ErrX = 0.0f;                            // m
float ErrY = 0.0f;                            // m
float ErrYaw = 0.0f;                          // deg
uint8_t grabSequence[6] = {5, 3, 4, 6, 2, 1}; // 6个抓取顺序
uint8_t specialBox = 1;                       // 特殊货箱号
uint8_t spareStack = 1;                       // 轮空纸垛
char usbData[64] = {0};                       // USB接收数据缓冲区
static bool placement_calibrated = false;     // 放置阶段是否已完成首次校准

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
    bool usb_ready = false;
    bool key_pressed = false;
    runActionGroup(2, 1);
    runActionGroup(3, 1);
    key_event_t evt;

    while (!(usb_ready && key_pressed))
    {
        // USB 接收处理
        if (USB_RxFlag == 1)
        {
            memcpy(usbData, UserRxBufferFS, USB_RxLen);
            usbData[USB_RxLen] = '\0';

            USB_RxFlag = 0;
            USB_RxLen = 0;

            if (!usb_ready && parseUsbData(usbData, grabSequence, &specialBox, &spareStack) == 0)
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

    lift_status.target_displacement = 120;
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
            }

            MoveToLeft(); // 移动到左侧
            Crawl_GrabBox(1, x, next_box);
            break;

        case 2:
            if (x == 0)
            {
                STARTMoveToCenter();
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
        Chassis_MoveToPosition_Blocking(-0.700, -0.203, 0, 135000);
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

            // 仅在 case1 中的第一个未跳过位置执行两步校准
            if (!placement_calibrated)
            {
                const char Startmsg[] = "Start\n";
                CDC_Transmit_FS((uint8_t *)Startmsg, sizeof(Startmsg) - 1);

                if (i != skip_position)
                {
                    // --- Y 校准 ---
                    LOGINFO("[CAL] Starting Y offset calibration\r\n");

                    // 发送 Yoffset 到上位机
                    const char yoffset_msg[] = "Yoffset\n";
                    CDC_Transmit_FS((uint8_t *)yoffset_msg, sizeof(yoffset_msg) - 1);

                    float initial_target_y = g_target_pos.y; // 记录初始Y目标值

                    while (1)
                    {
                        if (USB_RxFlag == 1)
                        {
                            // 清空接收数据末尾，确保字符串结束
                            UserRxBufferFS[USB_RxLen] = '\0';

                            if (strncmp((char *)UserRxBufferFS, "OffsetOK", 8) == 0)
                            {
                                // 校准完成，计算Y轴变化量作为ErrY
                                ErrY = g_target_pos.y - initial_target_y;
                                LOGINFO("[CAL] Y calibration complete, ErrY=%.3f\r\n", ErrY);

                                // 重新执行移动到对应位置
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

                                const char okmsg[] = "OK\n";
                                CDC_Transmit_FS((uint8_t *)okmsg, sizeof(okmsg) - 1);

                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                break;
                            }
                            else if (strncmp((char *)UserRxBufferFS, "Left", 4) == 0)
                            {
                                // 左移：Y轴目标值减少
                                g_target_pos.y -= 0.001f;
                                LOGINFO("[CAL] Left command, target_y=%.3f\r\n", g_target_pos.y);
                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                osDelay(100); // 等待0.1秒
                            }
                            else if (strncmp((char *)UserRxBufferFS, "Right", 5) == 0)
                            {
                                // 右移：Y轴目标值增加
                                g_target_pos.y += 0.001f;
                                LOGINFO("[CAL] Right command, target_y=%.3f\r\n", g_target_pos.y);
                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                osDelay(100); // 等待0.1秒
                            }
                            else
                            {
                                // 未识别的命令，清空标志位
                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                            }
                        }
                        osDelay(10);
                    }

                    // --- X+Yaw 校准 ---
                    LOGINFO("[CAL] Wait X+Yaw offset: x<dx>/y0/yaw<dYaw>\r\n");
                    while (1)
                    {
                        if (USB_RxFlag == 1)
                        {
                            float dx, dy, dyaw;
                            if (parseOffsetCommand_raw(UserRxBufferFS, USB_RxLen, &dx, &dy, &dyaw) == 0)
                            {
                                ErrX -= dx;
                                ErrYaw += dyaw;
                                ErrY += dy;
                                LOGINFO("[CAL] X,Yaw applied (%.3f, %.3f) => ErrX=%.3f ErrYaw=%.3f\r\n",
                                        dx, dyaw, ErrX, ErrYaw);

                                // X+Yaw校准后立即重新执行移动到对应位置
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

                                const char okmsg2[] = "OK\n";
                                CDC_Transmit_FS((uint8_t *)okmsg2, sizeof(okmsg2) - 1);

                                placement_calibrated = true;

                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                break;
                            }
                        }
                        osDelay(10);
                    }
                }
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
                lift_status.target_displacement = 300;
                Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 90000);
            }
        }

        break;
    case 0:         // spareStack = 6 (区域F)
        MoveToD0(); // 移动到D0位置

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
                if (!placement_calibrated)
                {
                    const char Startmsg[] = "Start\n";
                    CDC_Transmit_FS((uint8_t *)Startmsg, sizeof(Startmsg) - 1);

                    // --- Y 校准 ---
                    LOGINFO("[CAL] Starting Y offset calibration\r\n");

                    // 发送 Yoffset 到上位机
                    const char yoffset_msg[] = "Yoffset\n";
                    CDC_Transmit_FS((uint8_t *)yoffset_msg, sizeof(yoffset_msg) - 1);

                    float initial_target_y = g_target_pos.y; // 记录初始Y目标值

                    while (1)
                    {
                        if (USB_RxFlag == 1)
                        {
                            // 清空接收数据末尾，确保字符串结束
                            UserRxBufferFS[USB_RxLen] = '\0';

                            if (strncmp((char *)UserRxBufferFS, "OffsetOK", 8) == 0)
                            {
                                // 校准完成，计算Y轴变化量作为ErrY
                                ErrY = g_target_pos.y - initial_target_y;
                                LOGINFO("[CAL] Y calibration complete, ErrY=%.3f\r\n", ErrY);

                                // Y校准后立即重新执行移动到E0位置
                                MoveToE0();

                                const char okmsg[] = "OK\n";
                                CDC_Transmit_FS((uint8_t *)okmsg, sizeof(okmsg) - 1);

                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                break; // Y 校准完成
                            }
                            else if (strncmp((char *)UserRxBufferFS, "Left", 4) == 0)
                            {
                                // 左移：Y轴目标值减少
                                g_target_pos.y -= 0.001f;
                                LOGINFO("[CAL] Left command, target_y=%.3f\r\n", g_target_pos.y);
                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                osDelay(100); // 等待0.1秒
                            }
                            else if (strncmp((char *)UserRxBufferFS, "Right", 5) == 0)
                            {
                                // 右移：Y轴目标值增加
                                g_target_pos.y += 0.001f;
                                LOGINFO("[CAL] Right command, target_y=%.3f\r\n", g_target_pos.y);
                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                osDelay(100); // 等待0.1秒
                            }
                            else
                            {
                                // 未识别的命令，清空标志位
                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                            }
                        }
                        osDelay(10);
                    }

                    // --- X+Yaw 校准 ---
                    LOGINFO("[CAL] Wait X+Yaw offset: x<dx>/y0/yaw<dYaw>\r\n");
                    while (1)
                    {
                        if (USB_RxFlag == 1)
                        {
                            float dx, dy, dyaw;
                            if (parseOffsetCommand_raw(UserRxBufferFS, USB_RxLen, &dx, &dy, &dyaw) == 0)
                            {
                                ErrX -= dx;
                                ErrYaw += dyaw;
                                ErrY += dy;
                                LOGINFO("[CAL] X,Yaw applied (%.3f, %.3f) => ErrX=%.3f ErrYaw=%.3f\r\n",
                                        dx, dyaw, ErrX, ErrYaw);

                                // X+Yaw校准后立即重新执行移动到E0位置
                                MoveToE0();

                                const char okmsg2[] = "OK\n";
                                CDC_Transmit_FS((uint8_t *)okmsg2, sizeof(okmsg2) - 1);

                                placement_calibrated = true;

                                USB_RxFlag = 0;
                                USB_RxLen = 0;
                                break; // X+Yaw 校准完成
                            }
                        }
                        osDelay(10);
                    }
                }

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
    Chassis_SetXPIDParams(0.83f, 0.01f, 0.0f);
    Chassis_MoveToPosition_Blocking(0, 0.0, 0, 30000);
    lift_status.target_displacement = 12;
    while (1)
    {
        osDelay(1000);
    }
}
