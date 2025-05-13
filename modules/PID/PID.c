/**
 *******************************************************************************
 * @file    PID.c
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief   PID控制器实现
 *******************************************************************************
 * @attention
 *
 *******************************************************************************
 */
#include "PID.h"

/**
 * @brief 初始化PID实例
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // 复制基本配置参数
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->DeadBand = config->DeadBand;
    
    // 复制增强功能配置
    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
    
    // 初始化S型加减速参数，支持自动计算
    if (pid->Improve & PID_SCurve_Acceleration)
    {
        // 如果未设定MaxAccel，则根据输出限幅自动计算
        if (config->MaxAccel <= 0.0f && pid->MaxOut > 0.0f)
        {
            // 默认将最大加速度设为MaxOut的25%
            pid->MaxAccel = pid->MaxOut * 0.25f;
        }
        else
        {
            pid->MaxAccel = config->MaxAccel;
        }
        
        // 如果未设定MaxJerk，则根据最大加速度自动计算
        if (config->MaxJerk <= 0.0f && pid->MaxAccel > 0.0f)
        {
            // 默认将最大加加速度设为最大加速度的50%，确保平滑的S型曲线
            pid->MaxJerk = pid->MaxAccel * 0.5f;
        }
        else
        {
            pid->MaxJerk = config->MaxJerk;
        }
    }
    else
    {
        pid->MaxAccel = config->MaxAccel;
        pid->MaxJerk = config->MaxJerk;
    }
    
    pid->Target_Speed = 0.0f;
    pid->Current_Speed = 0.0f;
    pid->Current_Accel = 0.0f;
    
    // 初始化计算相关变量
    pid->Measure = 0;
    pid->Last_Measure = 0;
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->Last_ITerm = 0;
    
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->ITerm = 0;
    
    pid->Output = 0;
    pid->Last_Output = 0;
    pid->Last_Dout = 0;
    
    pid->Ref = 0;
    
    pid->DWT_CNT = 0;
    pid->dt = 0;
    
    // 初始化错误处理
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
}

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 更新测量值和目标值
    pid->Measure = measure;
    pid->Ref = ref;
    
    // 计算dt - 这里应该使用实际系统的控制周期
    pid->dt = 0.01f; // 假设控制周期为10ms，需要根据实际系统调整
    // 如果您的系统有DWT函数用于精确计算时间差，可以使用它替换上面的固定值
    // pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
    
    // 计算误差
    if (pid->Improve & PID_Proportional_On_Measurement)
    {
        // 在测量值上作比例项，误差仅用于积分和微分
        pid->Err = pid->Ref - pid->Measure;
    }
    else
    {
        pid->Err = pid->Ref - pid->Measure;
    }
    
    if (pid->Improve & PID_SCurve_Acceleration)
    {
        // 计算目标速度比例
        float target_ratio = pid->Kp * pid->Err / pid->MaxOut;
        if (target_ratio > 1.0f) target_ratio = 1.0f;
        else if (target_ratio < -1.0f) target_ratio = -1.0f;
    
        // 目标速度
        pid->Target_Speed = target_ratio * pid->MaxOut;
    
        // 允许的最大加速度变化（jerk 限制）
        float max_accel_change = pid->MaxJerk * pid->dt;
    
        // 计算期望加速度
        float desired_accel = (pid->Target_Speed - pid->Current_Speed) / pid->dt;
    
        // 限制期望加速度不超过 MaxAccel
        if (desired_accel > pid->MaxAccel) desired_accel = pid->MaxAccel;
        if (desired_accel < -pid->MaxAccel) desired_accel = -pid->MaxAccel;
    
        // 根据 jerk 限制调整当前加速度逼近期望加速度
        float accel_diff = desired_accel - pid->Current_Accel;
        if (accel_diff > max_accel_change)
            accel_diff = max_accel_change;
        else if (accel_diff < -max_accel_change)
            accel_diff = -max_accel_change;
    
        pid->Current_Accel += accel_diff;
    
        // 积分计算当前速度
        pid->Current_Speed += pid->Current_Accel * pid->dt;
    
        // 限制速度不超目标速度，防止超调
        if ((pid->Current_Accel > 0 && pid->Current_Speed > pid->Target_Speed) ||
            (pid->Current_Accel < 0 && pid->Current_Speed < pid->Target_Speed))
        {
            pid->Current_Speed = pid->Target_Speed;
            pid->Current_Accel = 0;
        }
    
        // 输出速度作为 PID 输出
        pid->Output = pid->Current_Speed;
    
        // 保存状态
        pid->Last_Measure = pid->Measure;
        pid->Last_Err = pid->Err;
        pid->Last_Output = pid->Output;
    
        // 计算调试信息
        pid->Pout = pid->Kp * pid->Err;
        pid->Iout = pid->ITerm;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    
        return pid->Output;
    }
    
    // 以下是原始PID控制逻辑（当不使用S型加减速时）
    
    // 死区控制
    if (fabsf(pid->Err) <= pid->DeadBand)
    {
        pid->Err = 0;
    }
    
    // 变速积分
    if (pid->Improve & PID_ChangingIntegrationRate)
    {
        if (fabsf(pid->Err) <= pid->CoefB)
        {
            // 误差小，正常积分
            pid->ITerm += pid->Ki * pid->Err * pid->dt;
        }
        else if (fabsf(pid->Err) <= pid->CoefA + pid->CoefB)
        {
            // 误差适中，根据误差大小调整积分速率
            float rate = (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
            pid->ITerm += pid->Ki * pid->Err * pid->dt * rate;
        }
        else
        {
            // 误差过大，不积分
            // 可以在这里选择不积分，或保持上一次的积分值
        }
    }
    else
    {
        // 普通积分
        pid->ITerm += pid->Ki * pid->Err * pid->dt;
    }
    
    // 梯形积分
    if (pid->Improve & PID_Trapezoid_Intergral)
    {
        pid->ITerm = pid->Ki * (pid->Err + pid->Last_Err) * pid->dt / 2.0f;
    }
    
    // 积分限幅
    if (pid->Improve & PID_Integral_Limit)
    {
        if (pid->ITerm > pid->IntegralLimit)
        {
            pid->ITerm = pid->IntegralLimit;
        }
        else if (pid->ITerm < -pid->IntegralLimit)
        {
            pid->ITerm = -pid->IntegralLimit;
        }
    }
    
    // 计算比例输出
    pid->Pout = pid->Kp * pid->Err;
    
    // 微分项计算
    if (pid->Improve & PID_Derivative_On_Measurement)
    {
        // 在测量值上微分，避免参考值突变引起微分输出突变
        pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        // 在误差上微分
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    }
    
    // 微分项滤波
    if (pid->Improve & PID_DerivativeFilter)
    {
        float alpha = pid->dt / (pid->Derivative_LPF_RC + pid->dt);
        pid->Dout = pid->Dout * alpha + pid->Last_Dout * (1 - alpha);
    }
    
    // 积分输出
    pid->Iout = pid->ITerm;
    
    // 计算总输出
    pid->Output = pid->Pout + pid->Iout + pid->Dout;
    
    // 输出滤波
    if (pid->Improve & PID_OutputFilter)
    {
        float alpha = pid->dt / (pid->Output_LPF_RC + pid->dt);
        pid->Output = pid->Output * alpha + pid->Last_Output * (1 - alpha);
    }
    
    // 输出限幅
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    else if (pid->Output < -pid->MaxOut)
    {
        pid->Output = -pid->MaxOut;
    }
    
    // 保存状态，用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Err = pid->Err;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_ITerm = pid->ITerm;
    
    return pid->Output;
}