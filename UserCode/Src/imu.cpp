//
// Created by lixin on 2026/10/18.
//
#include "imu.h"
#include <math.h>

#include "stm32f4xx_hal.h"
volatile float watch_acc[3];
volatile float watch_gyro[3];
void IMU::calculate_dt(uint32_t tick, uint32_t last_tick) {
    dt = (float)(tick - last_tick) * 0.001f;
}

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
    float range_g =  (float)(2000 >> (raw_range & 0x07));
    // 2. 读取gyro0x02寄存器中的6位gyro数据
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((rx_gyro_data[2*i+1] << 8) | rx_gyro_data[2*i]);
        gyro[i] = raw / 32768.0f * range_g * 3.1415926f / 180.0f;
        //watch_gyro[i] = gyro[i];
    }
}

//现在是旋转矩阵之后的结果测不出来，上机之后再开始，和板子的相对关系我搞到飞书上
void IMU::sensorDataTransform(void) {
    for (uint8_t i = 0; i < 3; i++) {
        gyro_sensor_[i] = gyro[0] * R_imu_[i][0] +
                          gyro[1] * R_imu_[i][1] +
                          gyro[2] * R_imu_[i][2] - gyro_bias_[i];

        accel_sensor_[i] = accel_sen_ * (acc[0] * R_imu_[i][0] +
                                         acc[1] * R_imu_[i][1] +
                                         acc[2] * R_imu_[i][2]) -
                           accel_bias_[i];
        //watch_acc[i] = accel_sensor_[i];
        watch_gyro[i] = gyro_sensor_[i];
    }//原文中提到先变换方向，再扣除偏置，更好的方法是否是偏置先计算在变方向，更符合
     //直觉，当然k和b的计算方法需要调整
}

void IMU::rpyTransform(void){
    float gx = gyro_sensor_[0]; float gy = gyro_sensor_[1]; float gz = gyro_sensor_[2];
    float ax = accel_sensor_[0]; float ay = accel_sensor_[1]; float az = accel_sensor_[2];

    float roll_acc = atan2f(ay,az);
    float pitch_acc = atan2f(-ax,sqrt(ay*ay+az*az));

    float roll_dt = gx + sinf(roll)*tanf(pitch)*gy + cosf(roll)*tanf(pitch)*gz;
    float pitch_dt = cosf(roll)*gy - sinf(roll)*gz;
    float yaw_dt = (sinf(roll)*gy + cosf(roll)*gz) / cosf(pitch);

    //之后要用filter，这里先手动改置信度
    float alp = 0.998f/(0.998f+dt);
    roll = alp*(roll + roll_dt*dt) + (1-alp)*roll_acc;
    pitch = alp*(pitch + pitch_dt*dt) + (1-alp)*pitch_acc;
    yaw = yaw + yaw_dt*dt;

    if (yaw<-3.1415926f) yaw += 2*3.1415926f;
    if (yaw>3.1415926f) yaw -= 2*3.1415926f;

    roll_deg = roll * 57.2957795f;
    pitch_deg = pitch * 57.2957795f;
    yaw_deg = yaw * 57.2957795f;
}


void IMU::initIMU_() {
    for (uint8_t i = 0; i < 3; i++) {
        gyro_bias_[i] = 0.0;
        accel_bias_[i] = 0.0f;
    }

    float sum_x = 0.0f ,sum_y = 0.0f,sum_z = 0.0f;
    float acc_init[3] = {0.0f, 0.0f, 0.0f};
    for (uint16_t i = 0; i < 1000; i++) {
        acc_calculate();
        gyro_calculate();
        sensorDataTransform();
        sum_x += gyro_sensor_[0];
        sum_y += gyro_sensor_[1];
        sum_z += gyro_sensor_[2];
        acc_init[0] += accel_sensor_[0];
        acc_init[1] += accel_sensor_[1];
        acc_init[2] += accel_sensor_[2];
        HAL_Delay(1);
    }
    gyro_bias_[0] = sum_x/1000.0f;
    gyro_bias_[1] = sum_y/1000.0f;
    gyro_bias_[2] = sum_z/1000.0f;
    float acc_avg[3] = { acc_init[0] / 1000.0f, acc_init[1] / 1000.0f, acc_init[2] / 1000.0f };
    mahony_.init(acc_avg);
}

void IMU::mahonyTransform(void) {
    mahony_.update(gyro_sensor_, accel_sensor_, dt);
    roll  = mahony_.roll;
    pitch = mahony_.pitch;
    yaw   = mahony_.yaw;
    roll_deg  = roll  * 57.2957795f;
    pitch_deg = pitch * 57.2957795f;
    yaw_deg   = yaw   * 57.2957795f;
}
//----------------------纯纯外用如下------------------
IMU imu;
//读取并换算单位 → 安装方向变换 → 加速度比例修正、扣除偏置 → Mahony 姿态解算
extern "C" void imu_update(uint32_t tick, uint32_t last_tick) {
    imu.calculate_dt(tick,last_tick);
    imu.acc_calculate();
    imu.gyro_calculate();

    imu.sensorDataTransform();
    //imu.rpyTransform();
    imu.mahonyTransform();
}

extern "C" void imu_init(void) {
    imu.initIMU_();
}