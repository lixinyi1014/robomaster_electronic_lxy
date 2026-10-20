//
// Created by lixin on 2026/10/18.
//

#ifndef RM_6_IMU_H
#define RM_6_IMU_H
#include "bmi.h"

#ifdef __cplusplus
#include "mahony.h"
class IMU {
public:
    void mahonyTransform(void);

    void acc_calculate();
    void gyro_calculate();
    void sensorDataTransform(void);
    void rpyTransform(void);
    void initIMU_();

    void calculate_dt(uint32_t tick, uint32_t last_tick);
    //raw
    float acc[3];
    float gyro[3];

    //sensor
    float accel_sensor_[3];
    float gyro_sensor_[3];

    //machine
    float roll = 0.0f,pitch = 0.0f ,yaw = 0.0f;
    float roll_deg,pitch_deg,yaw_deg;
    // 传感器转机体
    //现在是旋转矩阵之后的结果测不出来，上机之后再开始，和板子的相对关系我搞到飞书上
    //原版是-1，1，-1,用板子的时候先I
    const float R_imu_[3][3]={{1.0f, 0.0f,  0.0f},
                             {0.0f , 1.0f,  0.0f},
                             {0.0f , 0.0f, 1.0f}};
private:
    Mahony mahony_;
    float dt = 1.0f / 1000.0f;
    uint8_t raw_range;
    uint8_t rx_acc_data[6];
    uint8_t rx_gyro_data[6];

    float accel_sen_ = 1;
    float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
    float accel_bias_[3] = {0.0f, 0.0f, 0.0f};


};

extern "C" {
#endif

void imu_update(uint32_t tick, uint32_t last_tick);

#ifdef __cplusplus
}
#endif
#endif //RM_6_IMU_H
