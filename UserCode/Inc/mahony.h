//
// Created by lixin on 2026/10/19.
//

#ifndef RM_6_MAHONY_H
#define RM_6_MAHONY_H
class Mahony {
public:
    void init(const float acc[3]);                              // 用初始加速度设置 q
    void update(const float gyro[3], const float acc[3], float dt);
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float roll, pitch, yaw;
private:
    float kp = 1.0f, ki = 0.0f;
    float eInt[3] = {0};
};
#endif //RM_6_MAHONY_H
