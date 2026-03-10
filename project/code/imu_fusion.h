#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdint.h>
// --- 传感器轴向定义（可在项目配置中重定义） -------------------------
// 如果物理安装方向与默认相反，可以在编译时通过定义下面的宏调整符号
// ----------------------------------------------------------------------
// 欧拉角输出结构（单位：度）
typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMU_Angles_t;

// IMU 偏置结构体（用于零偏初始化）
typedef struct {
    float ax, ay, az;  // 加速度偏置
    float gx, gy, gz;  // 陀螺仪偏置
} IMU_Bias_t;

// 全局偏置变量声明
extern IMU_Bias_t imu_bias;

// IMU 零偏初始化（采样 sample_count 次求平均偏置）
void imu_zero_bias_init(int sample_count);

// 初始化滤波器，重置内部状态（可在此传入默认采样周期，但可选）
void imu_fusion_init(void);

// 每次从 imu660rb 全局变量读取当前原始数据并进行卡尔曼滤波
// out 如果非 NULL 会写入最新欧拉角
// dt 采样周期（秒），由调用者在每次循环中计算并传入
void imu_fusion_update(IMU_Angles_t *out, float dt);

// 获取 Pitch 角度（度）
float imu_get_pitch_angle(void);

// 获取 Pitch 轴角速度（度/秒）
float imu_get_pitch_gyro(void);

// 获取 Yaw 角度（度）
float imu_get_yaw_angle(void);

// 获取 Yaw 轴角速度（度/秒）
float imu_get_yaw_gyro(void);

#endif // IMU_FUSION_H
