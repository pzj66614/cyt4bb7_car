#ifndef code_MPU_h_
#define code_MPU_h_
// 四元数结构体
/*typedef struct {
    float q0, q1, q2, q3;
} Quat;

// 传感器数据结构体定义（与 MPU.c 保持一致）
typedef struct {
    float ax, ay, az; // 加速度
    float gx, gy, gz; // 陀螺仪
} SensorData;

// 零漂校准函数：采集若干次静止数据计算平均偏移
// 参数 sample_count 采样次数，推荐 100～500
void imu_zero_bias_init(int sample_count);
void imu660rb_collect_data(); // IMU数据采集处理函数*/
void imu660rb_collect_data();
#endif