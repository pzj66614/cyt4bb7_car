
#include "imu_fusion.h"
#include "zf_device_imu660rb.h"  // 提供全局变量 imu660rb_acc_x 等
#include "zf_common_headfile.h"
#include <math.h>

// IMU 轴向宏定义 - 处理 y, z 轴反向的问题
#define imu660rb_Acc_x   imu660rb_acc_x
#define imu660rb_Acc_y   (-imu660rb_acc_y)
#define imu660rb_Acc_z   (-imu660rb_acc_z)
#define imu660rb_Gyro_x  imu660rb_gyro_x
#define imu660rb_Gyro_y  (-imu660rb_gyro_y)
#define imu660rb_Gyro_z  (-imu660rb_gyro_z)

// IMU 偏置全局变量定义
IMU_Bias_t imu_bias = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 滤波参数，可根据需要调整
static const float Q_angle = 0.001f;    // 角度过程噪声
static const float Q_bias  = 0.003f;    // 偏置过程噪声
static const float R_measure = 0.03f;   // 测量噪声
static const float RAD2DEG = 57.29577951308232f;
extern uint64_t last_update_time; // 上次更新的时间戳
// 采样周期由调用者通过参数传入，取消全局变量

// 内部状态变量
static float angle_roll = 0.0f;  // 滤波器估计的角度
static float bias_roll  = 0.0f;  // 估计的陀螺偏置
static float P_roll[2][2]= {{1.0f, 0.0f}, {0.0f, 1.0f}};       // 协方差矩阵

static float angle_pitch = 0.0f;
static float bias_pitch  = 0.0f;
static float P_pitch[2][2]= {{1.0f, 0.0f}, {0.0f, 1.0f}};
static float gz_filtered = 0;//yaw 角度的简单积分输出
static float yaw = 0.0f;        // 简单积分 yaw
static float current_pitch_gyro = 0.0f;  // 当前pitch轴角速度，供control模块使用
static float current_yaw_gyro = 0.0f;    // 当前yaw轴角速度，供control模块使用
static float init_roll_deg = 0.0f;   // 上电初始 roll
static float init_pitch_deg = 0.0f;  // 上电初始 pitch

// 前向声明，避免在首次调用前出现隐式声明
static void acc_to_angles(float ax, float ay, float az, float *roll, float *pitch);

void imu_zero_bias_init(int sample_count)
{
     imu_bias.ax = imu_bias.ay = imu_bias.az = 0.0f;
     imu_bias.gx = imu_bias.gy = imu_bias.gz = 0.0f;

    for (int i = 0; i < sample_count; ++i) {
        imu660rb_get_gyro();
        imu660rb_get_acc();
        imu_bias.ax += imu660rb_Acc_x;
        imu_bias.ay += imu660rb_Acc_y;
        imu_bias.az += imu660rb_Acc_z;
        imu_bias.gx += imu660rb_Gyro_x;
        imu_bias.gy += imu660rb_Gyro_y;
        imu_bias.gz += imu660rb_Gyro_z;
       // 5ms 采样间隔，避免过快 
        system_delay_ms(5);
    }
    //imu_bias.ax /= sample_count;
    //imu_bias.ay /= sample_count;
    //imu_bias.az /= sample_count;
    imu_bias.ax = 0;
    imu_bias.ay = 0;
    imu_bias.az = 0; 
    imu_bias.gx /= sample_count;
    imu_bias.gy /= sample_count;
    imu_bias.gz /= sample_count;
}
static void imu_capture_initial_angles(void)
{
    const int sample_num = 20;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;

    for (int i = 0; i < sample_num; ++i)
    {
        imu660rb_get_acc();

        float ax = (float)(imu660rb_Acc_x - imu_bias.ax);
        float ay = (float)(imu660rb_Acc_y - imu_bias.ay);
        float az = (float)(imu660rb_Acc_z - imu_bias.az);
        float roll_r = 0.0f;
        float pitch_r = 0.0f;

        acc_to_angles(ax, ay, az, &roll_r, &pitch_r);
        roll_sum += roll_r * RAD2DEG;
        pitch_sum += pitch_r * RAD2DEG;
        system_delay_ms(2);
    }

    init_roll_deg = roll_sum / sample_num;
    init_pitch_deg = pitch_sum / sample_num;
}
// 将原始加速度转为 roll/pitch 角度（弧度）
static void acc_to_angles(float ax, float ay, float az, float *roll, float *pitch)
{
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 1e-6f) norm = 1.0f;
    ax /= norm; ay /= norm; az /= norm;
    *roll  = atan2f(ay, az);
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
}

void imu_fusion_init(void)
{
    angle_roll = angle_pitch = yaw = 0.0f;
    bias_roll = bias_pitch = 0.0f;
    P_roll[0][0] = 1.0f;  P_roll[0][1] = 0.0f;
    P_roll[1][0] = 0.0f;  P_roll[1][1] = 1.0f;
    
    P_pitch[0][0] = 1.0f; P_pitch[0][1] = 0.0f;
    P_pitch[1][0] = 0.0f; P_pitch[1][1] = 1.0f;
    /*P_roll[0][0] = P_roll[0][1] = P_roll[1][0] = P_roll[1][1] = 0.0f;
    P_pitch[0][0] = P_pitch[0][1] = P_pitch[1][0] = P_pitch[1][1] = 0.0f;*/
    imu_capture_initial_angles();
}

// 单轴卡尔曼滤波更新函数
void kalman_update(float dt, float rate, float measured_angle,
                          float *angle, float *bias, float P[2][2])
{
    // 预测
    float rate_unbiased = rate - *bias;
    *angle += dt * rate_unbiased;

    // 更新协方差矩阵 P = A*P*A' + Q
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // 计算卡尔曼增益
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    // 更新估计
    float y = measured_angle - *angle;
    *angle += K0 * y;
    *bias  += K1 * y;

    // 更新协方差 P = (I - K*H) * P
    float P00 = P[0][0];
    float P01 = P[0][1];
    float P10 = P[1][0];
    float P11 = P[1][1];

    P[0][0] = P00 - K0 * P00;
    P[0][1] = P01 - K0 * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P01;
}

void imu_fusion_update(IMU_Angles_t *out, float dt)
{
    // 读取原始传感器值

    float ax = (float)(imu660rb_Acc_x - imu_bias.ax);
    float ay = (float)(imu660rb_Acc_y - imu_bias.ay);
    float az = (float)(imu660rb_Acc_z - imu_bias.az);
    // 使用 imu660rb 的转换函数将原始值转换为 °/s
    float gx = imu660rb_gyro_transition((int16)(imu660rb_Gyro_x - imu_bias.gx)); // °/s
    float gy = imu660rb_gyro_transition((int16)(imu660rb_Gyro_y - imu_bias.gy));
    float gz = imu660rb_gyro_transition((int16)(imu660rb_Gyro_z - imu_bias.gz));
    
    // 保存当前角速度，供control模块使用
    current_pitch_gyro = gy;
    current_yaw_gyro = gz;

    // --- DEBUG: 输出经零偏校准后的加速度，可以看出重力方向 ---
    // 注意：ax, ay, az 结果已经乘以 IMU_ACC_*_SIGN，
    // 平放时应该只有一个轴接近 +1g，其它两个靠近 0。
    //printf("raw acc: ax=%.3f ay=%.3f az=%.3f\n", ax, ay, az);
    
    // 加速度转换为角度（弧度），然后转成度
    float roll_m, pitch_m;
    acc_to_angles(ax, ay, az, &roll_m, &pitch_m);
    const float rad2deg = 180.0f / 3.14159265358979f;
    roll_m  *= rad2deg;
    pitch_m *= rad2deg;
    float roll_rel_m = roll_m - init_roll_deg;
    float pitch_rel_m = pitch_m - init_pitch_deg;
    // 经过上面的打印会发现 roll_m 几乎恒为 -90°，这说明
    // 平置时 ay ≃ -1g && az ≃ 0；即传感器的 Y 轴指向地面。
    // 解决办法：要么翻转 IMU_ACC_Y_SIGN，要么物理旋转模块。

    // 卡尔曼滤波每轴，所有值保持度制
    kalman_update(dt, gx, roll_rel_m, &angle_roll, &bias_roll, P_roll);
    kalman_update(dt, gy, pitch_rel_m, &angle_pitch, &bias_pitch, P_pitch);
    if (gz >0.14f || gz < -0.14f)
    {gz_filtered = 0.8f * gz_filtered + 0.2f * gz;
    yaw += dt * gz_filtered;}
    if(out)
    {
        out->roll = angle_roll + init_roll_deg;
        out->pitch = angle_pitch + init_pitch_deg;
        out->yaw = yaw;
    }
}

float imu_get_pitch_angle(void)
{
    // angle_pitch 已经是度，但加速度计算时用了原始值
    // 这里需要按比例缩小（原始值/4098，所以角度也偏差4098倍）
    return  angle_pitch + init_pitch_deg;
}

float imu_get_pitch_gyro(void)
{
    // 原始int16值除以14.3转换为 °/s
    return (float)current_pitch_gyro / 14.3f;
}

float imu_get_yaw_angle(void)
{
    // yaw 已经是度
    return yaw;
}

float imu_get_yaw_gyro(void)
{
    // 原始int16值除以14.3转换为 °/s
    return (float)current_yaw_gyro / 14.3f;
}
