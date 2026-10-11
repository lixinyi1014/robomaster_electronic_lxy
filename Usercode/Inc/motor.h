//
// Created by lixin on 2026/10/11.
//

#ifndef RM5_MOTOR_H
#define RM5_MOTOR_H
#include "pid.h"
class Motor {
private:
    PID spid_,ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_;
    float feedforward_intensity_, intensity_;
    typedef enum ControlMethod_{
        POSITION_SPEED,  // 位置+速度控制
        SPEED,// 速度控制
        TORQUE,// 力矩控制
        } ControlMethod_e;
    ControlMethod_e control_method_;
public:
    Motor(float sKp,float sKi,float sKd,
          float pKp,float pKi,float pKd,
          ControlMethod_e controlMethod);
    void handle();
    void setPosition(float target_position,float feedforward_speed,float feedforward_intensity);
    void setSpeed(float target_speed,float feedforward_intensity);
    void setIntensity(float intensity);
};
#endif //RM5_MOTOR_H
