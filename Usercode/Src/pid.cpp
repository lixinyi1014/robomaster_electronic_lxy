//
// Created by lixin on 2026/10/11.
//

//这里主要实现了pid，pid参数是在motor里调整，i_max以及out_max待查手册，低通滤波的参数在这里进行调整
//现在还没有实现低通滤波

#include "pid.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter)
    :kp_(kp), ki_(ki), kd_(kd), d_filter_(d_filter), out_max_(out_max), i_max_(i_max) {
    reset();
}

void PID::reset() {
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
}

float PID::limit_in(float i) {
    if (i>i_max_) return i_max_;
    if (i<-i_max_) return -i_max_;
    return i;
}
float PID::limit_out(float o) {
    if (o>out_max_) return out_max_;
    if (o<-out_max_) return -out_max_;
    return o;
}

float PID::calc(float ref, float fdb) {
    err_ = ref - fdb;
    float p = kp_ * err_;
    float i = ki_ * (err_sum_ + err_);
    i = limit_in(i);
    err_sum_ += err_;
    float d = kd_ * (err_-last_err_);
    last_err_ = err_;

    float output = p + d + i;
    output = limit_out(output);

    return output;
}

