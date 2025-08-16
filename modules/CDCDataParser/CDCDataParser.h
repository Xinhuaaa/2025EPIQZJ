/**
  ******************************************************************************
  * @file    data_parser.h
  * @brief   数据解析模块头文件
  ******************************************************************************
  * @attention
  *
  * 描述：提供各种格式数据的解析功能
  *
  ******************************************************************************
  */

#ifndef __DATA_PARSER_H
#define __DATA_PARSER_H

#include "main.h"
#include <stdint.h>

/**
 * @brief 解析USB接收到的数据
 * 格式：2,5,4,1,3,6:1;2
 * 冒号前面的逗号分隔数字是抓取顺序(固定6个)，冒号和分号之间的数字是特殊货箱号(1个字节)，分号后面的数字是运行路线(1个字节)
 * 
 * @param usbData 接收到的数据字符串
 * @param grabSequence 解析后的抓取顺序数组(固定长度6)
 * @param specialBox 特殊货箱号
 * @param route 运行路线
 * @return int 解析结果：0成功，-1失败
 */
int parseUsbData(char* usbData, uint8_t* grabSequence, uint8_t* specialBox, uint8_t* route);

/**
 * @brief 解析偏置调节命令，格式示例：x+0.30/y-0.10/yaw-1.5
 * 支持空白和正负号，单位：x,y（米），yaw（度）。不完全匹配则返回 -1，不修改输出。
 * @param str 输入字符串（以'\0'结束）
 * @param dx  若成功解析赋值 x 偏置（相对值累加或绝对覆盖由上层决定，此函数只解析数值）
 * @param dy  若成功解析赋值 y 偏置
 * @param dyaw 若成功解析赋值 yaw 偏置
 * @return 0 成功；-1 失败/格式不匹配
 */
int parseOffsetCommand_raw(const char *buf, size_t len, float *dx, float *dy, float *dyaw);
#endif /* __DATA_PARSER_H */
