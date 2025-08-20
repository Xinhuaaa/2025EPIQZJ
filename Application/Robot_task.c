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

// 校准最终微调补偿基准 (每次 y 指令计数 * 该基准值)
#define CALI_COMP_BASE 0.003f

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
    // runActionGroup(2, 1);
    // runActionGroup(3, 1);
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
            Crawl_GrabBox(1, x, next_box,spareStack);
            break;

        case 2:
            if (x == 0)
            {
                STARTMoveToCenter();
            }
            MoveToCenter(); // 移动到中间
            Crawl_GrabBox(2, x, next_box,spareStack);
            break;

        case 3:
            if (x == 0)
            {
                STARTMoveToRight();
                osDelay(2000);
            }
            MoveToRight();
            Crawl_GrabBox(3, x, next_box,spareStack);
            break;

        case 4:
            if (x == 0)
            {
                STARTMoveToLeft();
                osDelay(2000);
            }
            MoveToLeft(); // 移动到左侧
            Crawl_GrabBox(4, x, next_box,spareStack);
            break;

        case 5:
            if (x == 0)
            {
                STARTMoveToCenter();
                osDelay(2000);
            }
            MoveToCenter(); // 移动到中间
            Crawl_GrabBox(5, x, next_box,spareStack);
            break;

        case 6:
            if (x == 0)
            {
                STARTMoveToRight();
                osDelay(2000);
            }
            MoveToRight(); // 移动到右侧

            Crawl_GrabBox(6, x, next_box,spareStack);
            break;
        }
        vTaskDelay(500);
    }
    lift_status.target_displacement = 240;
    runActionGroup(1, 1); // 使用常规舵机组抓取
    osDelay(1500);
    //  放置阶段
    //     placementSpecialBox：放置的第几个箱子是特殊箱子
    uint8_t placementSpecialBox = specialBox;
    int skip_position = 0;
    int box_counter = 0;
    bool mid_recal_triggered = false; // 中段(索引2或3)只触发一次复位
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
            // 在第一次遇到索引 2 或 3 (且未被跳过) 时，触发一次重新校准
            if (!mid_recal_triggered && i != skip_position && i != 5)
            {
                placement_calibrated = false; // 允许再次进入校准流程
            }
            // 只对实际参与放置的第一个纸垛位置执行校准（如果 i==0 被跳过则在下一个有效 i 校准）
            if (i != skip_position && !placement_calibrated && i != 5)
            {
                placement_calibrated = true; // 标记已完成首次校准
                const char yoffset_msg[] = "offset\n";
                CDC_Transmit_FS((uint8_t *)yoffset_msg, sizeof(yoffset_msg) - 1);

                // 记录初始目标值
                float initial_target_y;
                if (i == 4)
                {
                    // F位置(i=4)或A位置(i=5)时，记录X目标值
                    initial_target_y = g_target_pos.x;
                }
                else
                {
                    // 其他位置记录Y目标值
                    initial_target_y = g_target_pos.y;
                }

                // Y校准状态：0=等待指令, 1=左移中, 2=右移中, 3=完成
                int calibration_state = 0;
                int y_cmd_count = 0; // 收到 'y' 指令次数（步数权重）

                while (1)
                {
                    // 临界区保护USB数据读取

                    bool usb_received = (USB_RxFlag == 1);
                    uint8_t rx_len = USB_RxLen;
                    if (usb_received && rx_len > 0)
                    {
                        memcpy(usbData, UserRxBufferFS, rx_len);
                        usbData[rx_len] = '\0';
                        USB_RxFlag = 0;
                        USB_RxLen = 0;
                    }

                    // 处理接收到的USB数据
                    if (usb_received && rx_len > 0)
                    {
                        if (strstr(usbData, "OffsetOK") != NULL)
                        {
                            // 根据 y_cmd_count 动态计算补偿幅值
                            float comp = y_cmd_count * CALI_COMP_BASE; // 动态补偿
                            if (comp > 0.02f)
                                comp = 0.02f; // 保险限幅（可调）
                            if (calibration_state == 1)
                            {
                                // 左移过程：F/A 位走 X，其他位走 Y，结束向反方向回一点
                                if (i == 4)
                                {
                                    Chassis_AddOffset(-comp, 0.0f, 0.0f); // 左移(X-) 结束向右(X+)
                                }
                                else
                                {
                                    Chassis_AddOffset(0.0f, -comp, 0.0f); // 左移(Y+) 结束向右(Y-)
                                }
                            }
                            else if (calibration_state == 2)
                            {
                                if (i == 4)
                                {
                                    Chassis_AddOffset(comp, 0.0f, 0.0f); // 右移(X+) 结束向左(X-)
                                }
                                else
                                {
                                    Chassis_AddOffset(0.0f, comp, 0.0f); // 右移(Y-) 结束向左(Y+)
                                }
                            }
                            // 校准完成，计算轴变化量作为误差
                            if (i == 4 || i == 5)
                            {
                                // F位置(i=4)或A位置(i=5)时，计算X轴变化量作为ErrX
                                ErrX = g_target_pos.x - initial_target_y;
                            }
                            else
                            {
                                // 其他位置正常计算Y轴变化量作为ErrY
                                ErrY = g_target_pos.y - initial_target_y;
                            }

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

                            // 过调补偿：根据最后一次手动方向，反向微调0.005
                            // calibration_state: 1=最后在左移, 2=最后在右移

                            const char okmsg[] = "OK\n";
                            CDC_Transmit_FS((uint8_t *)okmsg, sizeof(okmsg) - 1);
                            break; // 校准完成，退出循环
                        }
                        else if (strstr(usbData, "Left") != NULL)
                        {
                            calibration_state = 1; // 设置为左移状态
                        }
                        else if (strstr(usbData, "Right") != NULL)
                        {
                            calibration_state = 2; // 设置为右移状态
                        }
                        else if (strstr(usbData, "y") != NULL)
                        {
                            // 收到Y指令，根据当前状态移动1cm
                            if (calibration_state == 1) // 左移
                            {
                                // 在MoveToA和MoveToF位置（i=4或i=5），由于旋转-90度，x和y坐标轴互换
                                if (i == 4)
                                {

                                    // F位置(i=4)或A位置(i=5)时，左移应该调整x而不是y
                                    // F位：用x轴偏移
                                    Chassis_AddOffset(0.005f, 0.0f, 0.0f);
                                    y_cmd_count++;
                                }
                                else
                                {
                                    // 其他位置正常调整y

                                    Chassis_AddOffset(0.0f, 0.005f, 0.0f);
                                    y_cmd_count++;
                                }
                            }
                            else if (calibration_state == 2) // 右移
                            {
                                // 在MoveToA和MoveToF位置（i=4或i=5），由于旋转-90度，x和y坐标轴互换
                                if (i == 4)
                                {
                                    // F位置(i=4)时，右移应该调整x而不是y

                                    Chassis_AddOffset(-0.005f, 0.0f, 0.0f);
                                    y_cmd_count++;
                                }
                                else
                                {
                                    // 其他位置正常调整y

                                    Chassis_AddOffset(0.0f, -0.005f, 0.0f);
                                    y_cmd_count++;
                                }
                            }
                        }
                    }

                    osDelay(100); // 短暂延时，等待下一次USB数据
                }
                // --- X+Yaw 校准 ---
                while (1)
                {
                    if (USB_RxFlag == 1)
                    {
                        float dx, dy, dyaw;
                        if (parseOffsetCommand_raw(UserRxBufferFS, USB_RxLen, &dx, &dy, &dyaw) == 0)
                        {
                            /* 统一逻辑：所有校准增量直接通过底盘偏移接口应用。
                                dx,dy,dyaw 视为本次新增校正量(世界坐标系)，累加到 g_offset。
                                ErrX/ErrY/ErrYaw 仅用于记录累计校正(调试/上位机显示)。 */
                            Chassis_AddOffset(-dx, dy, dyaw);
                            ErrX += dx;
                            ErrY += dy;
                            ErrYaw += dyaw;
                            osDelay(100);
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

                            USB_RxFlag = 0;
                            USB_RxLen = 0;
                            break;
                        }
                    }
                    osDelay(10);
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
                Chassis_MoveToPosition_Blocking(-0.71, 0.0, 0, 8000);
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
                Chassis_MoveToPosition_Blocking(-0.74, 0.0, 0, 6000);
                MoveToA0();
                break;
            }

            // 在第一次遇到索引 2 或 3 (且未被跳过) 时，触发一次重新校准
            if (!mid_recal_triggered && i != 5)
            {
                placement_calibrated = false; // 允许再次进入校准流程
            }

            // 只对实际参与放置的第一个纸垛位置执行校准（如果 i==0 被跳过则在下一个有效 i 校准）
            // 不对A区(i=5)进行校准
            if (i != 5 && !placement_calibrated)
            {
                placement_calibrated = true; // 标记已完成首次校准
                const char yoffset_msg[] = "offset\n";
                CDC_Transmit_FS((uint8_t *)yoffset_msg, sizeof(yoffset_msg) - 1);

                // 记录初始目标值
                float initial_target_y = g_target_pos.y;

                // Y校准状态：0=等待指令, 1=左移中, 2=右移中, 3=完成
                int calibration_state = 0;
                int y_cmd_count = 0; // 收到 'y' 指令次数（步数权重）

                while (1)
                {
                    // 临界区保护USB数据读取
                    bool usb_received = (USB_RxFlag == 1);
                    uint8_t rx_len = USB_RxLen;
                    if (usb_received && rx_len > 0)
                    {
                        memcpy(usbData, UserRxBufferFS, rx_len);
                        usbData[rx_len] = '\0';
                        USB_RxFlag = 0;
                        USB_RxLen = 0;
                    }

                    // 处理接收到的USB数据
                    if (usb_received && rx_len > 0)
                    {
                        if (strstr(usbData, "OffsetOK") != NULL)
                        {
                            // 根据 y_cmd_count 动态计算补偿幅值
                            float comp = y_cmd_count * CALI_COMP_BASE; // 动态补偿
                            if (comp > 0.02f)
                                comp = 0.02f; // 保险限幅（可调）
                            if (calibration_state == 1)
                            {
                                // 左移过程结束向反方向回一点
                                Chassis_AddOffset(0.0f, -comp, 0.0f); // 左移(Y+) 结束向右(Y-)
                            }
                            else if (calibration_state == 2)
                            {
                                Chassis_AddOffset(0.0f, comp, 0.0f); // 右移(Y-) 结束向左(Y+)
                            }
                            // 校准完成，计算轴变化量作为误差
                            ErrY = g_target_pos.y - initial_target_y;

                            // 重新执行移动到对应位置
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
                            }

                            const char okmsg[] = "OK\n";
                            CDC_Transmit_FS((uint8_t *)okmsg, sizeof(okmsg) - 1);
                            break; // 校准完成，退出循环
                        }
                        else if (strstr(usbData, "Left") != NULL)
                        {
                            calibration_state = 1; // 设置为左移状态
                        }
                        else if (strstr(usbData, "Right") != NULL)
                        {
                            calibration_state = 2; // 设置为右移状态
                        }
                        else if (strstr(usbData, "y") != NULL)
                        {
                            // 收到Y指令，根据当前状态移动1cm
                            if (calibration_state == 1) // 左移
                            {
                                // 正常调整y
                                Chassis_AddOffset(0.0f, 0.005f, 0.0f);
                                y_cmd_count++;
                            }
                            else if (calibration_state == 2) // 右移
                            {
                                // 正常调整y
                                Chassis_AddOffset(0.0f, -0.005f, 0.0f);
                                y_cmd_count++;
                            }
                        }
                    }

                    osDelay(100); // 短暂延时，等待下一次USB数据
                }
                // --- X+Yaw 校准 ---
                while (1)
                {
                    if (USB_RxFlag == 1)
                    {
                        float dx, dy, dyaw;
                        if (parseOffsetCommand_raw(UserRxBufferFS, USB_RxLen, &dx, &dy, &dyaw) == 0)
                        {
                            /* 统一逻辑：所有校准增量直接通过底盘偏移接口应用。
                                dx,dy,dyaw 视为本次新增校正量(世界坐标系)，累加到 g_offset。
                                ErrX/ErrY/ErrYaw 仅用于记录累计校正(调试/上位机显示)。 */
                            Chassis_AddOffset(-dx, dy, dyaw);
                            ErrX += dx;
                            ErrY += dy;
                            ErrYaw += dyaw;
                            osDelay(100);
                            // X+Yaw校准后立即重新执行移动到对应位置
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
                            }

                            const char okmsg2[] = "OK\n";
                            CDC_Transmit_FS((uint8_t *)okmsg2, sizeof(okmsg2) - 1);

                            USB_RxFlag = 0;
                            USB_RxLen = 0;
                            break;
                        }
                    }
                    osDelay(10);
                }
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
