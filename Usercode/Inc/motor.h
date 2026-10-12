//
// Created by lixin on 2026/10/11.
//

#ifndef RM5_MOTOR_H
#define RM5_MOTOR_H
#include "pid.h"
#include <stdint.h>


class Motor {
public:
    typedef enum ControlMethod_{
        POSITION_SPEED,  // 位置+速度控制
        SPEED,// 速度控制
        TORQUE,// 力矩控制
        } ControlMethod_e;
private:
    //--- 配置 ---
    const float ratio_ = 36; //减速比（M2006减速比32：1）
    static constexpr uint16_t kEncoderRange = 8192;

    //--- 这一帧解出来的 ---
    float ecd_angle_ = 0;   //机械角度（0~8191）
    float speedRpm_ = 0;
    float currentA_ = 0;

    //--- 连续角度 ---
    float angle_ = 0;       //累计角度
    float last_encoder_ = 0;    //上一帧
    bool received_ = false; //TODO:第一帧pid是否需要这个处理？

    //--- 待发送 ---
    uint8_t tx_data_[8] = {0};

    //PID相关
    PID spid_,ppid_;
    float target_angle_ = 0.0f, fdb_angle_ = 0.0f;
    float target_speed_ = 0.0f, fdb_speed_ = 0.0f;
    float feedforward_intensity_ = 0.0f, intensity_ = 0.0f;
    float feedforward_speed_ = 0.0f;
    ControlMethod_e control_method_ = POSITION_SPEED;

public:

    //初始化
    void init(float sKp,float sKi,float sKd,
          float pKp,float pKi,float pKd,
          ControlMethod_e controlMethod);

    //上层的一些操作
    void handle();
    void stop();
    void setPosition(float target_position,float feedforward_speed,float feedforward_intensity);
    void setSpeed(float target_speed,float feedforward_intensity);
    void setIntensity(float intensity);

    //入口：can接收中断调用
    void canRxMsgCallback(const uint8_t rx_data[8]);

    //出口
    float angle() const;
    float speedRpm() const;
    float currentAmps() const;
    bool hasFeedback() const;

    uint8_t* getTxData();
    void setTxCurrent(float current);
};
#endif //RM5_MOTOR_H
