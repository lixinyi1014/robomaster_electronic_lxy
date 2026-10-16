//
// Created by lixin on 2026/10/11.
//

#include "motor.h"
#include "pid.h"
#include <math.h>
volatile float g_ff_gain = 0.36f;//0.1*9.8*0.065/0.18 = 0.354
volatile float speed = 0.0f;
void Motor::init(float sKp,float sKi,float sKd,
             float pKp,float pKi,float pKd,
             ControlMethod_e controlMethod) {
    spid_.init(sKp,sKi,sKd,1.0f,10.0f,0.1f);//(-10,10)
    ppid_.init(pKp,pKi,pKd,3.0f,3000.0,1.0f);
    control_method_= controlMethod;
    stop();
}

void Motor::handle() {
    if (!received_) { stop(); return; }
    fdb_angle_ = angle_;
    fdb_speed_ = speedRpm_;
    feedforward_intensity_ = g_ff_gain * sinf((angle_) * 3.14f/180.0f);
    intensity_ = feedforward_intensity_;
    if (control_method_ == POSITION_SPEED)
    {
        target_speed_ = ppid_.calc(target_angle_,fdb_angle_) + feedforward_speed_;
    }

    if (control_method_ == POSITION_SPEED || control_method_ == SPEED)
    {
        intensity_ = spid_.calc(target_speed_,fdb_speed_) + feedforward_intensity_;
    }


    setTxCurrent(intensity_);
}
void Motor::stop() {
    spid_.reset();
    ppid_.reset();
    intensity_ = 0.0f;
    setTxCurrent(0.0f);
}
void Motor::setPosition(float target_position,float feedforward_speed = 0.0f,float feedforward_intensity = 0.0f) {
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;;
}
void Motor::setSpeed(float target_speed,float feedforward_intensity) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void Motor::setIntensity(float intensity) {
    control_method_ = TORQUE;
    intensity_ = intensity;
}

void Motor::canRxMsgCallback(const uint8_t data[8])
{
    uint16_t encoder = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
    if (encoder >= 8192) return;

    if (received_)
    {
        int32_t delta = (int32_t)encoder - (int32_t)last_encoder_;
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        angle_ += (float)delta * 360.0f / 8192.0f / 36.0f;
    }

    last_encoder_ = encoder;
    speedRpm_ = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    currentA_ = (int16_t)((uint16_t)data[4] << 8 | data[5]);

    received_ = true;
}

void Motor::setTxCurrent(float current)
{
    if (current > 10.0f) current = 10.0f;
    if (current < -10.0f) current = -10.0f;
    int16_t command = (int16_t)(current * 1000.0f);
    tx_data_[0] = (uint8_t)((uint16_t)command >> 8);
    tx_data_[1] = (uint8_t)((uint16_t)command & 0xFF);
}

uint8_t* Motor::getTxData()
{
    return tx_data_;
}