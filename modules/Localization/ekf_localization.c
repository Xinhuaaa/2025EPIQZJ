#include "ekf_localization.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 状态向量 x = [x, y, theta]^T (mm, mm, rad)
static float X[3];
// 协方差 P 3x3，按行主序存储
static float P[9];

// 过程噪声缩放参数
static float Q_trans_per_mm = 1e-3f;   // 位移每 mm 增加的过程噪声方差（通道共用）
static float Q_theta_per_rad = 1e-3f;  // 角度每 rad 增加的过程噪声方差
static float Q_min_xy = 1e-4f;         // 最小平移过程噪声（共用）
static float Q_min_theta = 1e-4f;      // 最小角度过程噪声

// 可选：分别为 x/y 设置的过程噪声（如未设置则为负，表示不启用该通道化参数）
static float Qx_trans_per_mm = 1e-3f;
static float Qy_trans_per_mm = 1e-3f;
static float Q_min_x = 1e-4f;
static float Q_min_y = 1e-4f;

// 观测噪声（只观测 theta）
static float R_yaw = 0.01f; // rad^2，可按陀螺仪稳定性调参

static float wrap_pi(float a){
    while(a > (float)M_PI) a -= 2.0f*(float)M_PI;
    while(a < -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}

void ekf_set_process_noise(float q_trans_mm, float q_theta_rad,
                           float q_min_xy_in, float q_min_theta_in){
    Q_trans_per_mm = q_trans_mm;
    Q_theta_per_rad = q_theta_rad;
    Q_min_xy = q_min_xy_in;
    Q_min_theta = q_min_theta_in;
}

void ekf_set_measurement_noise(float r_yaw_var){
    R_yaw = r_yaw_var;
}

void ekf_set_process_noise_xy(float qx_trans_mm, float qy_trans_mm,
                              float q_min_x_in, float q_min_y_in){
    Qx_trans_per_mm = qx_trans_mm;
    Qy_trans_per_mm = qy_trans_mm;
    Q_min_x = q_min_x_in;
    Q_min_y = q_min_y_in;
}

void ekf_init(float x0, float y0, float theta0){
    X[0] = x0; X[1] = y0; X[2] = wrap_pi(theta0);
    // 初始协方差设置为较小不确定
    memset(P, 0, sizeof(P));
    P[0] = 10.0f;  // x 方差 (mm^2)
    P[4] = 10.0f;  // y 方差 (mm^2)
    P[8] = 0.05f;  // theta 方差 (rad^2)
}

// 矩阵工具: C = A*B (3x3)
static void mat33_mul(const float *A, const float *B, float *C){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            C[i*3+j] = A[i*3+0]*B[0*3+j] + A[i*3+1]*B[1*3+j] + A[i*3+2]*B[2*3+j];
        }
    }
}

// 矩阵: C = A + B (3x3)
static void mat33_add(const float *A, const float *B, float *C){
    for(int i=0;i<9;i++) C[i] = A[i] + B[i];
}

// 矩阵: B = A^T (3x3)
static void mat33_transpose(const float *A, float *B){
    B[0]=A[0]; B[1]=A[3]; B[2]=A[6];
    B[3]=A[1]; B[4]=A[4]; B[5]=A[7];
    B[6]=A[2]; B[7]=A[5]; B[8]=A[8];
}

void ekf_predict(float dx_body_mm, float dy_body_mm, float dtheta_rad){
    // 状态转移：把车体位移旋转到世界坐标系
    float c = cosf(X[2]);
    float s = sinf(X[2]);
    float dxg = c*dx_body_mm - s*dy_body_mm;
    float dyg = s*dx_body_mm + c*dy_body_mm;

    X[0] += dxg;
    X[1] += dyg;
    X[2] = wrap_pi(X[2] + dtheta_rad);

    // 雅可比 F = d f / d x
    // x' = x + cos(theta)*dx - sin(theta)*dy
    // y' = y + sin(theta)*dx + cos(theta)*dy
    // th'= th + dtheta
    float F[9] = {
        1.0f, 0.0f, -s*dx_body_mm - c*dy_body_mm,
        0.0f, 1.0f,  c*dx_body_mm - s*dy_body_mm,
        0.0f, 0.0f,  1.0f
    };

    // 过程噪声 Q，依据本次运动量自适应
    float move = fabsf(dx_body_mm) + fabsf(dy_body_mm);
    // 通道化或共用噪声
    float qx_base = (Q_min_x >= 0.0f ? Q_min_x : Q_min_xy);
    float qy_base = (Q_min_y >= 0.0f ? Q_min_y : Q_min_xy);
    float qx_k = (Qx_trans_per_mm >= 0.0f ? Qx_trans_per_mm : Q_trans_per_mm);
    float qy_k = (Qy_trans_per_mm >= 0.0f ? Qy_trans_per_mm : Q_trans_per_mm);
    float q_x = qx_base + qx_k * (fabsf(dx_body_mm) + 0.2f * fabsf(dy_body_mm)); // 允许轻微耦合
    float q_y = qy_base + qy_k * (fabsf(dy_body_mm) + 0.2f * fabsf(dx_body_mm));
    float q_th = Q_min_theta + Q_theta_per_rad * fabsf(dtheta_rad); // rad^2
    float Q[9] = { q_x, 0, 0, 0, q_y, 0, 0, 0, q_th };

    // P = F P F^T + Q
    float FP[9];
    float Ft[9];
    float FPFt[9];
    mat33_mul(F, P, FP);
    mat33_transpose(F, Ft);
    mat33_mul(FP, Ft, FPFt);
    mat33_add(FPFt, Q, P);
}

void ekf_update_yaw(float yaw_meas_rad){
    // 观测: z = theta
    // H = [0 0 1]
    float y = wrap_pi(yaw_meas_rad - X[2]); // 创新
    float S = P[8] + R_yaw;                 // 标量
    float K0 = P[2] / S; // 对应第一行第三列
    float K1 = P[5] / S; // 第二行第三列
    float K2 = P[8] / S; // 第三行第三列

    // 状态更新
    X[0] += K0 * y;
    X[1] += K1 * y;
    X[2] = wrap_pi(X[2] + K2 * y);

    // 协方差更新: P = (I - K H) P
    // I - K H =
    // [1 0 0; 0 1 0; 0 0 1] - [K0;K1;K2]*[0 0 1] =
    // [1 0 -K0; 0 1 -K1; 0 0 1-K2]
    float I_KH[9] = {
        1.0f, 0.0f, -K0,
        0.0f, 1.0f, -K1,
        0.0f, 0.0f, 1.0f - K2
    };
    float newP[9];
    mat33_mul(I_KH, P, newP);
    memcpy(P, newP, sizeof(P));
}

void ekf_get_state(float *x_mm, float *y_mm, float *theta_rad){
    if(x_mm) *x_mm = X[0];
    if(y_mm) *y_mm = X[1];
    if(theta_rad) *theta_rad = X[2];
}
