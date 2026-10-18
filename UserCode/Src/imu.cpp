//
// Created by lixin on 2026/10/18.
//
#include "imu.h"
volatile float watch_acc[3];
volatile float watch_gyro[3];
void IMU::acc_calculate(){
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    // code here
    float range_g =  1.5f * (float)(1 << ((raw_range & 0x03) + 1));
    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((rx_acc_data[2*i+1] << 8) | rx_acc_data[2*i]);
        acc[i] = raw / 32768.0f * range_g * 9.80665f;
        watch_acc[i] = acc[i];
    }

    // code here
}

void IMU::gyro_calculate(){
    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    float range_g =  (float)(2000 >> ((raw_range & 0x04) + 1));
    // 2. 读取gyro0x02寄存器中的6位gyro数据
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((rx_gyro_data[2*i+1] << 8) | rx_gyro_data[2*i]);
        gyro[i] = raw / 32768.0f * range_g ;
        watch_gyro[i] = gyro[i];
    }
}

IMU imu;

extern "C" void imu_update() {
    imu.acc_calculate();
    imu.gyro_calculate();
}