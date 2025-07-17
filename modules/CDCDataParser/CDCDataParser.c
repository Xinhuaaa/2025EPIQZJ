/**
  ******************************************************************************
  * @file    data_parser.c
  * @brief   数据解析模块
  ******************************************************************************
  * @attention
  *
  * 描述：提供各种格式数据的解析功能
  *
  ******************************************************************************
  */

#include "CDCDataParser.h"
#include <string.h>
#include <stdlib.h>

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
int parseUsbData(char* usbData, uint8_t* grabSequence, uint8_t* specialBox, uint8_t* route)
{
    if (usbData == NULL || grabSequence == NULL || specialBox == NULL || route == NULL)
    {
        return -1;
    }

    char* colonPos = strchr(usbData, ':');
    char* semicolonPos = strchr(usbData, ';');
    
    if (colonPos == NULL || semicolonPos == NULL)
    {
        return -1;
    }

    // 解析抓取顺序，固定6个数字
    char* start = usbData;
    for (int i = 0; i < 6; i++)
    {
        if (start >= colonPos)
        {
            return -1;  // 数据格式错误，不足6个数字
        }
        
        grabSequence[i] = atoi(start);
        
        // 移动到下一个数字
        char* nextComma = strchr(start, ',');
        if (nextComma == NULL && i < 5)
        {
            return -1;  // 格式错误，缺少逗号
        }
        start = (i < 5) ? nextComma + 1 : colonPos;
    }
    
    // 解析特殊货箱号 (1个字节)
    *specialBox = atoi(colonPos + 1);
    
    // 解析运行路线 (1个字节)
    *route = atoi(semicolonPos + 1);
    
    return 0;
}
