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
#include "bsp_dwt.h"

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
    pid->SmoothRef = 0.0f;

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
            pid->MaxAccel = pid->MaxOut * 0.50f;
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
    pid->Measure = measure;
    pid->Ref = ref;

    // 控制周期
    pid->dt = 0.01f; // 可换成 DWT_GetDeltaT(&pid->DWT_CNT);

    /************* S型加减速平滑参考值 *************/
    if (pid->Improve & PID_SCurve_Acceleration)
    {
        // 初始化 SmoothRef 第一次使用时避免跳变
        static uint8_t first_time = 1;
        if (first_time)
        {
            pid->SmoothRef = ref;
            pid->Current_Speed = 0;
            pid->Current_Accel = 0;
            first_time = 0;
        }

        float diff = pid->Ref - pid->SmoothRef;
        float target_ratio = pid->Kp * diff / pid->MaxOut;

        if (target_ratio > 1.0f) target_ratio = 1.0f;
        else if (target_ratio < -1.0f) target_ratio = -1.0f;

        pid->Target_Speed = target_ratio * pid->MaxOut;

        float max_accel_change = pid->MaxJerk * pid->dt;
        float desired_accel = (pid->Target_Speed - pid->Current_Speed) / pid->dt;

        if (desired_accel > pid->MaxAccel) desired_accel = pid->MaxAccel;
        if (desired_accel < -pid->MaxAccel) desired_accel = -pid->MaxAccel;

        float accel_diff = desired_accel - pid->Current_Accel;
        if (accel_diff > max_accel_change) accel_diff = max_accel_change;
        else if (accel_diff < -max_accel_change) accel_diff = -max_accel_change;

        pid->Current_Accel += accel_diff;
        pid->Current_Speed += pid->Current_Accel * pid->dt;
        pid->SmoothRef += pid->Current_Speed * pid->dt;

        // 防止越过目标
        if ((diff > 0 && pid->SmoothRef > pid->Ref) ||
            (diff < 0 && pid->SmoothRef < pid->Ref))
        {
            pid->SmoothRef = pid->Ref;
            pid->Current_Speed = 0;
            pid->Current_Accel = 0;
        }
    }
    else
    {
        pid->SmoothRef = pid->Ref;
    }

    /************* PID误差计算 *************/
    pid->Err = pid->SmoothRef - pid->Measure;

    if (fabsf(pid->Err) <= pid->DeadBand)
    {
        pid->Err = 0;
    }

    /************* 积分处理 *************/
    if (pid->Improve & PID_ChangingIntegrationRate)
    {
        if (fabsf(pid->Err) <= pid->CoefB)
        {
            pid->ITerm += pid->Ki * pid->Err * pid->dt;
        }
        else if (fabsf(pid->Err) <= pid->CoefA + pid->CoefB)
        {
            float rate = (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
            pid->ITerm += pid->Ki * pid->Err * pid->dt * rate;
        }
    }
    else
    {
        pid->ITerm += pid->Ki * pid->Err * pid->dt;
    }

    if (pid->Improve & PID_Trapezoid_Intergral)
    {
        pid->ITerm = pid->Ki * (pid->Err + pid->Last_Err) * pid->dt / 2.0f;
    }

    if (pid->Improve & PID_Integral_Limit)
    {
        if (pid->ITerm > pid->IntegralLimit)
            pid->ITerm = pid->IntegralLimit;
        else if (pid->ITerm < -pid->IntegralLimit)
            pid->ITerm = -pid->IntegralLimit;
    }

    /************* 比例和微分 *************/
    pid->Pout = pid->Kp * pid->Err;

    if (pid->Improve & PID_Derivative_On_Measurement)
    {
        pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    }

    if (pid->Improve & PID_DerivativeFilter)
    {
        float alpha = pid->dt / (pid->Derivative_LPF_RC + pid->dt);
        pid->Dout = pid->Dout * alpha + pid->Last_Dout * (1.0f - alpha);
    }

    /************* 总输出计算 *************/
    pid->Iout = pid->ITerm;
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    if (pid->Improve & PID_OutputFilter)
    {
        float alpha = pid->dt / (pid->Output_LPF_RC + pid->dt);
        pid->Output = pid->Output * alpha + pid->Last_Output * (1.0f - alpha);
    }

    if (pid->Output > pid->MaxOut)
        pid->Output = pid->MaxOut;
    else if (pid->Output < -pid->MaxOut)
        pid->Output = -pid->MaxOut;

    /************* 保存状态 *************/
    pid->Last_Measure = pid->Measure;
    pid->Last_Err = pid->Err;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
