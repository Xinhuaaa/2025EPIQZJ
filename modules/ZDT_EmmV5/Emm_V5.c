#include "stm32f4xx_hal.h"
#include "Emm_V5.h"
#include "cmsis_os.h" // FreeRTOS API
#include "semphr.h"
#include "bsp_log.h"

// 函数前向声明，避免隐式声明警告
void Emm_V5_Set_Encoder_Zero(uint8_t id);

static SemaphoreHandle_t usart6_rx_sem = NULL;

// 编码器累积值，用于处理周期性清零问题
static int32_t accumulated_encoder[4] = {0};
static int32_t last_raw_encoder[4] = {0};
static const int32_t ENCODER_MAX = 65535;  // 编码器一圈的最大值
static const int32_t ENCODER_HALF = 32768; // 编码器最大值的一半，用于判断溢出

// 添加软件零点偏移变量
static int32_t encoder_zero_offset[4] = {0};

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
	HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t1 s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 8);
  osDelay(3);

}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
   uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)cmd, 5);
}

/**
  * @brief    初始化信号量和累积编码器计数器
  */
void Emm_V5_Init(void)
{
    if (usart6_rx_sem == NULL) {
        usart6_rx_sem = xSemaphoreCreateBinary();
    }
    Emm_V5_Get_All_Encoders(accumulated_encoder);
    // 初始化编码器累积值和原始值
    for (int i = 0; i < 4; i++) {
        accumulated_encoder[i] = 0;
        last_raw_encoder[i] = 0;
    }
Emm_V5_Set_Encoder_Zero(0);
}

/**
  * @brief    DMA接收回调
  * @param    huart  ：UART句柄
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (usart6_rx_sem) {
            xSemaphoreGiveFromISR(usart6_rx_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// 读取单个电机编码器累计值（内部使用）
static int32_t Emm_V5_Read_Encoder(uint8_t addr)
{
    uint8_t cmd[3] = {addr, 0x31, 0x6B}; // S_ENCL指令
    uint8_t rx_buf[5] = {0}; // 返回5字节：地址 + 0x31 + 高8位 + 低8位 + 校验字节
    int32_t raw_value = 0;
    
    // 避免频繁清空缓冲区和日志，提高速度
    static uint8_t debug_count = 0;
    bool debug_log = (++debug_count % 10) == 0; // 只有每10次才输出日志
    
    // 先清空缓冲区和状态
    __HAL_UART_FLUSH_DRREGISTER(&huart6);
    HAL_UART_DMAStop(&huart6);
    
    // 记录发送指令 - 减少日志输出提高速度
    if (debug_log) {
        LOGINFO("发送指令 ID:%d", addr);
    }
    
    // 发送读取编码器命令 - 使用阻塞方式但缩短超时
    HAL_UART_Transmit(&huart6, cmd, 3, 5);
    
    // 启动DMA接收，返回5字节数据
    HAL_UART_Receive_DMA(&huart6, rx_buf, 5);
    
    if (usart6_rx_sem) {
        if (xSemaphoreTake(usart6_rx_sem, pdMS_TO_TICKS(30)) == pdTRUE) { // 适当减少超时时间
            // 首先检查地址匹配
            if (rx_buf[0] == addr && rx_buf[1] == 0x31) {
                // 编码器值 = 高8位 << 8 | 低8位
                raw_value = (int32_t)((rx_buf[2] << 8) | rx_buf[3]);
                
                // 计算编码器累积值 - 处理周期性清零和溢出
                int idx = addr - 1;
                
                // 检测清零或大幅度变化
                int32_t diff = raw_value - last_raw_encoder[idx];
                
                // 处理编码器周期性清零 - 如果原来不是0而现在变成了较小的值，说明清零了
                if (last_raw_encoder[idx] > ENCODER_HALF && raw_value < ENCODER_HALF/2) {
                    // 检测到清零周期，累加上一个最大值
                    accumulated_encoder[idx] += (ENCODER_MAX - last_raw_encoder[idx]) + raw_value;
                }
                // 处理编码器溢出 - 如果原来接近最大值而现在变成了较小的值，说明溢出了
                else if (diff < -ENCODER_HALF) {
                    accumulated_encoder[idx] += ENCODER_MAX + diff;
                }
                // 处理编码器反向溢出 - 如果原来是较小的值而现在变成了接近最大值，说明反向溢出了
                else if (diff > ENCODER_HALF) {
                    accumulated_encoder[idx] -= ENCODER_MAX - diff;
                }
                // 正常情况直接累加差值
                else {
                    accumulated_encoder[idx] += diff;
                }
                
                // 更新上次的原始编码器值
                last_raw_encoder[idx] = raw_value;
                
                if (debug_log) {
                    LOGINFO("ID:%d 值=%d, 累积值=%d", addr, raw_value, accumulated_encoder[idx]);
                }
                return accumulated_encoder[idx]; // 返回累积值
                
            } else if (debug_log) {
                LOGWARNING("ID:%d 响应错误: [%02X %02X %02X %02X %02X]", 
                       addr, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
            }
        } else if (debug_log) {
            LOGERROR("ID:%d 接收超时", addr);
        }
    }
    
    osDelay(5); // 减少延时，加快读取速度
    return accumulated_encoder[addr-1]; // 即使读取失败也返回上次的累积值
}

/**
  * @brief    快速读取1-4号电机编码器累计值
  * @note     针对某个电机不可用的情况进行了优化
  * @param    encoder 存储编码器累计值的数组，大小为4
  */
void Emm_V5_Get_All_Encoders(int32_t encoder[4])
{
    static uint8_t debug_count = 0;
    bool debug_log = (++debug_count % 20) == 0; // 每20次输出一次日志
    
    if (debug_log) {
        LOGINFO("读取编码器...");
    }
    
    // 先清空串口缓冲区
    __HAL_UART_FLUSH_DRREGISTER(&huart6);
    HAL_UART_DMAStop(&huart6);
    osDelay(5); // 减少等待时间
    
    // 读取1、2、3、4号电机
    int32_t raw_encoder[4];
    raw_encoder[0] = Emm_V5_Read_Encoder(1);
    raw_encoder[1] = Emm_V5_Read_Encoder(2);
    raw_encoder[2] = Emm_V5_Read_Encoder(3);
    raw_encoder[3] = Emm_V5_Read_Encoder(4);
    
    // 应用零点偏移
    for (int i = 0; i < 4; i++) {
        encoder[i] = raw_encoder[i] - encoder_zero_offset[i];
    }
    
    if (debug_log) {
        LOGINFO("编码器原始值: [%d, %d, %d, %d]", raw_encoder[0], raw_encoder[1], raw_encoder[2], raw_encoder[3]);
        LOGINFO("编码器偏移后结果: [%d, %d, %d, %d]", encoder[0], encoder[1], encoder[2], encoder[3]);
    }
}

/**
  * @brief    重置编码器累积计数
  * @param    id 电机ID，1-4，如果为0则重置所有电机
  */
void Emm_V5_Reset_Encoder_Accumulation(uint8_t id)
{
    if (id == 0) {
        // 重置所有电机的累积值
        for (int i = 0; i < 4; i++) {
            accumulated_encoder[i] = 0;
            last_raw_encoder[i] = 0;
        }
    } else if (id <= 4) {
        // 重置指定电机的累积值
        accumulated_encoder[id-1] = 0;
        last_raw_encoder[id-1] = 0;
    }
}

/**
  * @brief    设置编码器软件零点（不改变原始编码器值，只设置偏移量）
  * @param    id 电机ID，1-4，如果为0则设置所有电机
  */
void Emm_V5_Set_Encoder_Zero(uint8_t id)
{
    int32_t current_values[4] = {0};
    
    // 先获取当前的编码器值
    Emm_V5_Get_All_Encoders(current_values);
    
    if (id == 0) {
        // 设置所有电机的零点偏移
        for (int i = 0; i < 4; i++) {
            encoder_zero_offset[i] = current_values[i];
            LOGINFO("电机%d设置零点偏移: %d", i+1, encoder_zero_offset[i]);
        }
    } else if (id <= 4) {
        // 设置指定电机的零点偏移
        encoder_zero_offset[id-1] = current_values[id-1];
        LOGINFO("电机%d设置零点偏移: %d", id, encoder_zero_offset[id-1]);
    }
}
void X_vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[9] = {0};

    cmd[0] =  addr;                       
    cmd[1] =  0xF6;                       
    cmd[2] =  dir;                        
    cmd[3] =  (uint8_t)(acc >> 8);        
    cmd[4] =  (uint8_t)(acc >> 0);        
		cmd[5] =  (uint8_t)(vel >> 8);        
    cmd[6] =  (uint8_t)(vel >> 0);        
    cmd[7] =  snF;                        
    cmd[8] =  0x6B;                       

    HAL_UART_Transmit_DMA(&huart6, cmd, 9);
    HAL_Delay(10);
}