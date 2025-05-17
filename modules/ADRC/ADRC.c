#include "ADRC.h"
#include <math.h>

/* 内部辅助函数声明 */
static float fhan(float x1, float x2, float r, float h);
static float sign(float x);
static float fal(float e, float alpha, float delta);

/**
 * @brief 创建并初始化ADRC控制器
 */
void ADRC_Init(ADRC_Controller *adrc, ADRC_Init_Config_t *config) {
    /* 基本参数设置 */
    adrc->r = config->r;
    adrc->h = config->h;
    adrc->b0 = config->b0;
    adrc->max_output = config->max_output;

    /* TD参数初始化 */
    adrc->v1 = 0.0f;
    adrc->v2 = 0.0f;
    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;

    /* ESO参数初始化 */
    adrc->beta01 = config->beta01;
    adrc->beta02 = config->beta02;
    adrc->beta03 = config->beta03;
    adrc->z01 = 0.0f;
    adrc->z02 = 0.0f;
    adrc->z03 = 0.0f;

    /* NLSEF参数初始化 */
    adrc->beta1 = config->beta1;
    adrc->beta2 = config->beta2;
    adrc->alpha1 = config->alpha1;
    adrc->alpha2 = config->alpha2;
    adrc->delta = config->delta;

    /* 其他控制相关数据初始化 */
    adrc->u = 0.0f;
    adrc->u0 = 0.0f;
    adrc->ref = 0.0f;
    adrc->fh = 0.0f;
    adrc->e1 = 0.0f;
    adrc->e2 = 0.0f;
}

/**
 * @brief 设置ADRC的跟踪微分器参数
 */
void ADRC_SetTD(ADRC_Controller *adrc, float r) {
    adrc->r = r;
}

/**
 * @brief 设置ADRC的扩张状态观测器参数
 */
void ADRC_SetESO(ADRC_Controller *adrc, float beta01, float beta02, float beta03) {
    adrc->beta01 = beta01;
    adrc->beta02 = beta02;
    adrc->beta03 = beta03;
}

/**
 * @brief 设置ADRC的非线性误差反馈控制律参数
 */
void ADRC_SetNLSEF(ADRC_Controller *adrc, float beta1, float beta2, float alpha1, float alpha2, float delta) {
    adrc->beta1 = beta1;
    adrc->beta2 = beta2;
    adrc->alpha1 = alpha1;
    adrc->alpha2 = alpha2;
    adrc->delta = delta;
}

/**
 * @brief ADRC控制器计算函数
 */
float ADRC_Compute(ADRC_Controller *adrc, float ref, float feedback) {
    float fh, u0;
    adrc->ref = ref;
    
    /* 跟踪微分器 */
    adrc->fh = fhan(adrc->v1 - adrc->ref, adrc->v2, adrc->r, adrc->h);
    adrc->v1 = adrc->v1 + adrc->h * adrc->v2;
    adrc->v2 = adrc->v2 + adrc->h * adrc->fh;
    
    /* 扩张状态观测器 */
    adrc->e1 = adrc->z01 - feedback;
    adrc->z01 = adrc->z01 + adrc->h * (adrc->z02 - adrc->beta01 * adrc->e1);
    adrc->z02 = adrc->z02 + adrc->h * (adrc->z03 - adrc->beta02 * adrc->e1 + adrc->b0 * adrc->u);
    adrc->z03 = adrc->z03 - adrc->h * adrc->beta03 * adrc->e1;
    
    /* 非线性误差反馈控制律 */
    adrc->e1 = adrc->v1 - adrc->z01;
    adrc->e2 = adrc->v2 - adrc->z02;
    
    fh = fal(adrc->e1, adrc->alpha1, adrc->delta) + fal(adrc->e2, adrc->alpha2, adrc->delta);
    u0 = adrc->beta1 * fh;
    
    /* 计算控制量并考虑扰动补偿 */
    adrc->u0 = u0;
    adrc->u = adrc->u0 - adrc->z03 / adrc->b0;
    
    /* 输出限幅 */
    if (adrc->u > adrc->max_output) {
        adrc->u = adrc->max_output;
    } else if (adrc->u < -adrc->max_output) {
        adrc->u = -adrc->max_output;
    }
    
    return adrc->u;
}

/**
 * @brief 重置ADRC控制器状态
 */
void ADRC_Reset(ADRC_Controller *adrc) {
    /* TD参数重置 */
    adrc->v1 = 0.0f;
    adrc->v2 = 0.0f;
    
    /* ESO参数重置 */
    adrc->z01 = 0.0f;
    adrc->z02 = 0.0f;
    adrc->z03 = 0.0f;
    
    /* 控制相关数据重置 */
    adrc->u = 0.0f;
    adrc->u0 = 0.0f;
    adrc->fh = 0.0f;
    adrc->e1 = 0.0f;
    adrc->e2 = 0.0f;
}

/**
 * @brief 创建并初始化ADRC控制器(旧接口，保留兼容性)
 */
void ADRC_Init_Legacy(ADRC_Controller *adrc, float r, float h, float b0, float max_output) {
    ADRC_Init_Config_t config;
    
    /* 设置基本参数 */
    config.r = r;
    config.h = h;
    config.b0 = b0;
    config.max_output = max_output;
    
    /* 默认ESO参数 */
    config.beta01 = 100.0f;
    config.beta02 = 300.0f;
    config.beta03 = 1000.0f;
    
    /* 默认NLSEF参数 */
    config.beta1 = 0.5f;
    config.beta2 = 0.8f;
    config.alpha1 = 0.75f;
    config.alpha2 = 1.5f;
    config.delta = 0.1f;
    
    /* 调用新的初始化函数 */
    ADRC_Init(adrc, &config);
}

/**
 * @brief 最速控制综合函数
 * @param x1 跟踪量
 * @param x2 速度量
 * @param r 跟踪速度因子
 * @param h 积分步长
 * @return 最速跟踪控制输出
 */
static float fhan(float x1, float x2, float r, float h) {
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    float a2 = a0 + sign(y) * (a1 - d) * 0.5f;
    float sy = (sign(y + d) - sign(y - d)) * 0.5f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sign(a + d) - sign(a - d)) * 0.5f;
    
    return -r * (a / d) - r * sign(a) * (1.0f - sa);
}

/**
 * @brief 符号函数
 * @param x 输入
 * @return 符号: 1 for x>0, -1 for x<0, 0 for x=0
 */
static float sign(float x) {
    if (x > 0.0f) {
        return 1.0f;
    } else if (x < 0.0f) {
        return -1.0f;
    } else {
        return 0.0f;
    }
}

/**
 * @brief 非线性函数
 * @param e 误差
 * @param alpha 非线性度
 * @param delta 线性区宽度
 * @return 非线性输出
 */
static float fal(float e, float alpha, float delta) {
    float abs_e = fabsf(e);
    
    if (abs_e <= delta) {
        return e / powf(delta, 1.0f - alpha);
    } else {
        return powf(abs_e, alpha) * sign(e);
    }
}