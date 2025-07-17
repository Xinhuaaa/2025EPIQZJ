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

#endif /* __DATA_PARSER_H */
