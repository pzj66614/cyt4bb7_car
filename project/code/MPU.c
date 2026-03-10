#include <math.h>
#include <stdio.h>
#include "zf_device_imu660rb.h"
#include "zf_common_headfile.h"
#include "MPU.h"
#include "imu_fusion.h"
void imu660rb_collect_data()
{
    imu660rb_get_gyro();
    imu660rb_get_acc();
}
// 参数配置
/*#define Kp 0.2f       // 比例增益
#define Ki 0.001f     // 积分增益
#define DT 0.01f      // 采样周期 (100Hz)
#define M_PI 3.1415926
// 全局变量
Quat q = {1.0f, 0, 0, 0}; // 初始四元数
float exInt = 0, eyInt = 0, ezInt = 0; // 积分项
static SensorData imu_bias = {0};
//零漂初始化：读  取多次静止数据并求平均作为偏移
// 模拟 IMU 数据读取函数（实际需替换为硬件驱动）
void imu660ra_get_data(SensorData *data) {
    // TODO: 替换为实际 SPI/I2C 读取
    data->ax = 0.0f;
    data->ay = 0.0f;
    data->az = 1.0f;   // 静止时加速度近似为重力
    data->gx = 0.0f;
    data->gy = 0.0f;
    data->gz = 0.0f;
}
void imu660ra_collect_data()
{
    imu660ra_get_gyro();
    imu660ra_get_acc();
}
// 姿态解算函数
void imu_update(SensorData *d, float *roll, float *pitch, float *yaw) {
    float norm, vx, vy, vz;
    float ex, ey, ez;
    float q0_dot, q1_dot, q2_dot, q3_dot;

    // 1. 加速度归一化
    norm = sqrt(d->ax*d->ax + d->ay*d->ay + d->az*d->az);
    if (norm == 0) return; // 防止除零
    d->ax /= norm; d->ay /= norm; d->az /= norm;

    // 2. 预测重力方向
    vx = 2*(q.q1*q.q3 - q.q0*q.q2);
    vy = 2*(q.q0*q.q1 + q.q2*q.q3);
    vz = q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3;

    // 3. 误差计算（加速度与预测重力的叉积）
    ex = (d->ay*vz - d->az*vy);
    ey = (d->az*vx - d->ax*vz);
    ez = (d->ax*vy - d->ay*vx);

    // 4. 积分项更新
    exInt += ex * Ki * DT;
    eyInt += ey * Ki * DT;
    ezInt += ez * Ki * DT;

    // 5. 陀螺仪修正
    d->gx += Kp*ex + exInt;
    d->gy += Kp*ey + eyInt;
    d->gz += Kp*ez + ezInt;

    // 6. 四元数更新
    q0_dot = -q.q1*d->gx - q.q2*d->gy - q.q3*d->gz;
    q1_dot =  q.q0*d->gx + q.q2*d->gz - q.q3*d->gy;
    q2_dot =  q.q0*d->gy - q.q1*d->gz + q.q3*d->gx;
    q3_dot =  q.q0*d->gz + q.q1*d->gy - q.q2*d->gx;

    q.q0 += q0_dot * DT * 0.5f;
    q.q1 += q1_dot * DT * 0.5f;
    q.q2 += q2_dot * DT * 0.5f;
    q.q3 += q3_dot * DT * 0.5f;

    // 7. 四元数归一化
    norm = sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
    q.q0 /= norm; q.q1 /= norm; q.q2 /= norm; q.q3 /= norm;

    // 8. 四元数转欧拉角
    *roll  = atan2(2*(q.q2*q.q3 + q.q0*q.q1), q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3) * 180.0f/M_PI;
    *pitch = asin(-2*(q.q1*q.q3 - q.q0*q.q2)) * 180.0f/M_PI;
    *yaw   = atan2(2*(q.q1*q.q2 + q.q0*q.q3), q.q0*q.q0 + q.q1*q.q1 - q.q2*q.q2 - q.q3*q.q3) * 180.0f/M_PI;
}
void imu_zero_bias_init(int sample_count)
{
    SensorData d;
    imu_bias.ax = imu_bias.ay = imu_bias.az = 0;
    imu_bias.gx = imu_bias.gy = imu_bias.gz = 0;

    for (int i = 0; i < sample_count; ++i) {
        imu660ra_get_gyro();
        imu660ra_get_acc();
        imu_bias.ax += imu660ra_acc_x;
        imu_bias.ay += imu660ra_acc_y;
        imu_bias.az += imu660ra_acc_z;
        imu_bias.gx += imu660ra_gyro_x;
        imu_bias.gy += imu660ra_gyro_y;
        imu_bias.gz += imu660ra_gyro_z;
        5ms 采样间隔，避免过快 
        system_delay_ms(5);
    }
    imu_bias.ax /= sample_count;
    imu_bias.ay /= sample_count;
    imu_bias.az /= sample_count;
    imu_bias.gx /= sample_count;
    imu_bias.gy /= sample_count;
    imu_bias.gz /= sample_count;
}
// 主函数示例
int main() {
    SensorData data;
    float roll, pitch, yaw;

    while (1) {
        imu660ra_get_data(&data); // 读取数据
        imu_update(&data, &roll, &pitch, &yaw); // 姿态解算

        printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", roll, pitch, yaw);
    }
    return 0;
}

// 示例：供主程序或测试调用的融合示例函数（非阻塞）
void imu_fusion_example_once(void)
{
    IMU_Angles_t ang;
    imu_fusion_init(0.005f); // 若 PIT 为5ms 则取 0.005
    imu_fusion_update(&ang);
    printf("Fusion Roll=%.2f Pitch=%.2f Yaw=%.2f\r\n", ang.roll, ang.pitch, ang.yaw);
}*/

