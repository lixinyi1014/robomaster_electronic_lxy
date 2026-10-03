
#include "main.h"
#include <cstdint>

class Motor {
public:
    explicit Motor(float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);

    void setTxCurrent(float amperes,uint8_t motor_id);
    uint8_t* getTxData();

private:
    const float ratio_;
    uint8_t rx_data_[8] = {};

    //这一帧解出来的
    float ecd_angle_ = 0;
    float speedRpm_ = 0;
    float currentA_ = 0;
    float tempC_ = 0;

    //连续角度
    float angle_ = 0;
    float last_ecd_ = 0;
    bool received_ = false;

    //待发送
    uint8_t tx_data_[8] = {};

    //函数实现
    void rxdataTOmsg();
    static constexpr uint16_t kEncoderRange = 8192;
    static constexpr float kMaxCurrentA = 20.0f;
    static constexpr int16_t kMaxCurrentRaw = 16384;
};
