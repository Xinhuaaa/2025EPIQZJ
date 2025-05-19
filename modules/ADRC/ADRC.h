#ifndef ADRC_H
#define ADRC_H

#include <stdint.h>

/**
 * @brief ADRC控制器初始化配置结构体
 */
typedef struct {
    /* 基本参数 */
    float r;           // 跟踪速度因子
    float h;           // 积分步长（一般为控制周期）
    float b0;          // 系统输入增益
    float max_output;  // 控制器最大输出
    
    /* ESO参数 - 扩张状态观测器 */
    float w0;       // ESO增益
    float beta01;      // ESO反馈增益1
    float beta02;      // ESO反馈增益2
    float beta03;      // ESO反馈增益3
    
    /* NLSEF参数 - 非线性误差反馈控制率 */
    float beta1;       // 非线性增益1
    float beta2;       // 非线性增益2
    float alpha1;      // 非线性因子1
    float alpha2;      // 非线性因子2
    float delta;       // 线性区宽度
} ADRC_Init_Config_t;

/**
 * @brief ADRC控制器结构体
 */
typedef struct {
    /* 控制器参数 */
    float r;        // 跟踪速度因子
    float h;        // 积分步长
    float b0;       // 系统输入增益
    
    /* TD参数 - 跟踪微分器 */
    float v1;       // TD状态跟踪量
    float v2;       // TD状态微分量
    float z1;       // TD中间变量
    float z2;       // TD中间变量
    
    /* ESO参数 - 扩张状态观测器 */
    float w0;      // ESO增益    
    float beta01;   // ESO反馈增益1
    float beta02;   // ESO反馈增益2
    float beta03;   // ESO反馈增益3
    float z01;      // 系统输出估计
    float z02;      // 系统状态微分估计
    float z03;      // 系统总扰动估计
    
    /* NLSEF参数 - 非线性误差反馈控制率 */
    float beta1;    // 非线性增益1
    float beta2;    // 非线性增益2
    float alpha1;   // 非线性因子1
    float alpha2;   // 非线性因子2
    float delta;    // 线性区宽度
    
    /* 控制相关数据 */
    float u;        // 控制器输出
    float u0;       // 基础控制量
    float ref;      // 参考输入
    float fh;       // 误差反馈量
    float e1;       // 跟踪误差
    float e2;       // 速度误差
    
    /* 控制限幅 */
    float max_output;  // 输出限幅
} ADRC_Controller;

/**
 * @brief 创建并初始化ADRC控制器
 * @param adrc 控制器结构体指针
 * @param config ADRC配置结构体指针
 */
void ADRC_Init(ADRC_Controller *adrc, ADRC_Init_Config_t *config);

/**
 * @brief 创建并初始化ADRC控制器(旧接口，保留兼容性)
 * @param adrc 控制器结构体指针
 * @param r 跟踪速度因子
 * @param h 积分步长
 * @param b0 系统输入增益
 * @param max_output 输出限幅
 */
void ADRC_Init_Legacy(ADRC_Controller *adrc, float r, float h, float b0, float max_output);

/**
 * @brief 设置ADRC的跟踪微分器参数
 * @param adrc 控制器结构体指针
 * @param r 跟踪速度因子
 */
void ADRC_SetTD(ADRC_Controller *adrc, float r);

/**
 * @brief 设置ADRC的扩张状态观测器参数
 * @param adrc 控制器结构体指针
 * @param beta01 ESO反馈增益1
 * @param beta02 ESO反馈增益2
 * @param beta03 ESO反馈增益3
 */
void ADRC_SetESO(ADRC_Controller *adrc, float beta01, float beta02, float beta03);

/**
 * @brief 设置ADRC的非线性误差反馈控制律参数
 * @param adrc 控制器结构体指针
 * @param beta1 非线性增益1
 * @param beta2 非线性增益2
 * @param alpha1 非线性因子1
 * @param alpha2 非线性因子2
 * @param delta 线性区宽度
 */
void ADRC_SetNLSEF(ADRC_Controller *adrc, float beta1, float beta2, float alpha1, float alpha2, float delta);

/**
 * @brief ADRC控制器计算函数
 * @param adrc 控制器结构体指针
 * @param ref 参考输入
 * @param feedback 系统反馈
 * @return 控制输出
 */
float ADRC_Compute(ADRC_Controller *adrc, float ref, float feedback);

/**
 * @brief 重置ADRC控制器状态
 * @param adrc 控制器结构体指针
 */
void ADRC_Reset(ADRC_Controller *adrc);

#endif /* ADRC_H */