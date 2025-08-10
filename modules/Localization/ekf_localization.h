// 简易二维位姿扩展卡尔曼滤波器（EKF）
// 状态: x [mm], y [mm], theta [rad]

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 初始化滤波器
// x0, y0 单位: mm；theta0 单位: rad
void ekf_init(float x0, float y0, float theta0);

// 预测步骤（离散位移模型）
// dx_body_mm, dy_body_mm: 车体坐标系的平移增量(mm)
// dtheta_rad: 航向增量(rad)
void ekf_predict(float dx_body_mm, float dy_body_mm, float dtheta_rad);

// 观测更新：使用陀螺仪绝对航向（或累计航向）单位: rad
void ekf_update_yaw(float yaw_meas_rad);

// 设置过程噪声参数（可选调参）
// q_trans_per_mm: 每 mm 位移带来的过程噪声(方差)系数
// q_theta_per_rad: 每 rad 角度变化带来的过程噪声(方差)系数
// q_min_xy/q_min_theta: 静止时的最小过程噪声(方差)
void ekf_set_process_noise(float q_trans_per_mm, float q_theta_per_rad,
                           float q_min_xy, float q_min_theta);

// 可分别设置 x/y 的过程噪声（更细粒度调参）
// qx_trans_per_mm/qy_trans_per_mm: 每 mm 位移带来的过程噪声(方差)系数（x/y 分别）
// q_min_x/q_min_y: 静止时的最小过程噪声(方差)（x/y 分别）
void ekf_set_process_noise_xy(float qx_trans_per_mm, float qy_trans_per_mm,
                              float q_min_x, float q_min_y);

// 设置观测噪声（yaw 角测量方差）
void ekf_set_measurement_noise(float r_yaw_var);

// 获取当前状态
void ekf_get_state(float *x_mm, float *y_mm, float *theta_rad);

#ifdef __cplusplus
}
#endif
