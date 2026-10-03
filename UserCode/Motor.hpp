
#include "main.h"
#include <cstdint>

class Motor {
public:
    explicit Motor(float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);

    // 出口：控制逻辑读取
    float angle() const;          // 输出轴角度（度）
    float speedRpm() const;       // 转子转速（RPM）
    float currentAmps() const;    // 转矩电流（A）
    float temperatureC() const;   // 温度（℃）
    bool  hasFeedback() const;    // 收到过帧？

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
