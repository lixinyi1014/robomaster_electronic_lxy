//
// Created by lixin on 2026/10/18.
//

#ifndef RM_6_IMU_H
#define RM_6_IMU_H
#include "bmi.h"

#ifdef __cplusplus            // 只有 C++ 文件能看到这个类
class IMU {
public:
    void acc_calculate();
    void gyro_calculate();

    float acc[3];
    float gyro[3];

private:
    uint8_t raw_range;
    uint8_t rx_acc_data[6];
    uint8_t rx_gyro_data[6];
};

extern "C" {
#endif

void imu_update(void);

#ifdef __cplusplus
}
#endif
#endif //RM_6_IMU_H
