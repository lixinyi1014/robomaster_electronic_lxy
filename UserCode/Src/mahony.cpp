//
// Created by lixin on 2026/10/19.
//
#include "mahony.h"

#include <math.h>

void Mahony::init(const float acc[3]) {
    float roll0  = atan2f(acc[1], acc[2]);
    float pitch0 = atan2f(-acc[0], sqrtf(acc[1]*acc[1] + acc[2]*acc[2]));

    float cr = cosf(roll0 * 0.5f),  sr = sinf(roll0 * 0.5f);
    float cp = cosf(pitch0 * 0.5f), sp = sinf(pitch0 * 0.5f);
    q[0] =  cr * cp;
    q[1] =  sr * cp;
    q[2] =  cr * sp;
    q[3] = -sr * sp;

    eInt[0] = eInt[1] = eInt[2] = 0.0f;
    roll = roll0;  pitch = pitch0;  yaw = 0.0f;
}

void Mahony::update(const float gyro[3], const float acc[3], float dt) {
//先手写一下框架
    /*
     *输入：gyro (rad/s，已减零偏)，acc，dt
 ① 加速度归一化

 ② 用 q 算"以为重力在哪"
 ③ 比较 a 和 v，算出误差       → e = a × v
 ④ 用误差修正陀螺仪           → g += Kp·e
 ⑤ 用修正后的 g 更新 q         → 积分
 ⑥ q 归一化
 ⑦ q 转换成 roll/pitch/yaw
     */

    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float ax = acc[0],  ay = acc[1],  az = acc[2];

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.0f) {
        // 加速度全为 0 时没法纠正，跳过 ②~④
        ax /= norm;  ay /= norm;  az /= norm;

        float vx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
        float vy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
        float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;

        if (ki > 0.0f) {
            eInt[0] += ex * dt;  eInt[1] += ey * dt;  eInt[2] += ez * dt;
            gx += ki * eInt[0];  gy += ki * eInt[1];  gz += ki * eInt[2];
        }
        gx += kp * ex;  gy += kp * ey;  gz += kp * ez;
    }
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float h = 0.5f * dt;
    q[0] += (-q1*gx - q2*gy - q3*gz) * h;
    q[1] += ( q0*gx + q2*gz - q3*gy) * h;
    q[2] += ( q0*gy - q1*gz + q3*gx) * h;
    q[3] += ( q0*gz + q1*gy - q2*gx) * h;

    norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm;  q[1] /= norm;  q[2] /= norm;  q[3] /= norm;

    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));

    float s = 2.0f*(q[0]*q[2] - q[1]*q[3]);
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    pitch = asinf(s);

    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));


}


//--------以下为外用接口-------
