//
// Created by lixin on 2026/10/11.
//

//这里主要实现了pid，pid参数是在motor里调整，i_max以及out_max待查手册，低通滤波的参数在这里进行调整
//低通滤波目前只作用在d上
//现在的问题在于有很多可以const,可以传& *的地方可以修改，这个之后给ai调一下
//C610控制转矩电流值范围为-10000~10000
#include "pid.h"

void PID::init(float kp, float ki, float kd, float d_filter){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    d_filter_ = d_filter;
    reset();
}

void PID::reset() {
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    last_d_ = 0.0f;
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
float PID::filter(float output) {
    output = (1-d_filter_)*last_d_ + d_filter_*output;
    return output;
}

float PID::calc(float ref, float fdb) {
    err_ = ref - fdb;
    float p = kp_ * err_;
    float i = ki_ * (err_sum_ + err_);
    i = limit_in(i);
    err_sum_ += err_;
    float d = kd_ * (err_-last_err_);
    d = filter(d);
    last_err_ = err_;

    float output = p + d + i;
    output = limit_out(output);

    return output;
}

