//
// Motor 类的实现。头文件里只放声明，函数体都放这里，
// 否则 Motor.hpp 被多个 .cpp include 时会报 "multiple definition"。
//
#include "Motor.hpp"
#include <cstring>

Motor::Motor(float ratio) : ratio_(ratio) {}

void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    std::memcpy(rx_data_, rx_data, sizeof(rx_data_));
    rxdataTOmsg();
}

void Motor::rxdataTOmsg() {
    uint16_t ecd = (uint16_t)((rx_data_[0] << 8) | rx_data_[1]);
    int16_t rpm = (int16_t)((rx_data_[2]<<8) | rx_data_[3]);
    int16_t cur = (int16_t)((rx_data_[4]<<8) | rx_data_[5]);
    uint8_t temp = rx_data_[6];

    ecd_angle_ = (float)(ecd * 360.0f / kEncoderRange);
    speedRpm_ = (float)(rpm);
    currentA_ = (float)(cur*20.0f/16384.0f);
    tempC_ = (float)temp;

    if (!received_) {
        last_ecd_ = ecd;
        received_ = true;
        return;
    }
    float delta = ecd - last_ecd_;
    if (delta> kEncoderRange/2) delta -= kEncoderRange;
    if (delta<-kEncoderRange/2) delta += kEncoderRange;
    angle_ +=delta * 360.0f/kEncoderRange/ratio_;


    last_ecd_ = ecd;

}


void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    if (motor_id < 1 || motor_id > 4) return;          // 0x200 帧只管 1~4 号

    if (amperes >  kMaxCurrentA) amperes =  kMaxCurrentA;   // 限幅，防止溢出
    if (amperes < -kMaxCurrentA) amperes = -kMaxCurrentA;

    int16_t raw = (int16_t)(amperes * kMaxCurrentRaw / kMaxCurrentA);
    uint8_t idx = (uint8_t)((motor_id - 1) * 2);
    tx_data_[idx]     = (uint8_t)((uint16_t)raw >> 8);
    tx_data_[idx + 1] = (uint8_t)((uint16_t)raw & 0xFF);
}

uint8_t* Motor::getTxData() { return tx_data_; }
