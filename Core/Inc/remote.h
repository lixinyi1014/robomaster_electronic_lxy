/**
******************************************************************************
 * @file    remote.cpp/h
 * @brief   Remote control. 遥控器
 ******************************************************************************
 * Copyright (c) 2026 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */
#ifndef UART_REMOTE_H
#define UART_REMOTE_H

#include "connect.hpp"
#include "usart.h"

constexpr uint16_t RC_RX_BUF_SIZE = 36u;
constexpr uint16_t RC_FRAME_LEN = 18u;

class Remote {
    enum class RCSwitchState_e {
        UP = 1,
        DOWN = 2,
        MID = 3
    };
    
    UART_HandleTypeDef *huart_;
    uint8_t rx_buf[RC_RX_BUF_SIZE], rx_data_[RC_FRAME_LEN];
    volatile uint8_t rx_len_;

public:
    // connect state 遥控器连接状态
    Connect connect_;
    // remote channel 遥控器通道
    struct {
        uint16_t l_row;
        uint16_t l_col;
        uint16_t r_row;
        uint16_t r_col;
        uint16_t dial_wheel;
    }__packed channel_;
    // remote switch 遥控器拨挡
    struct {
        RCSwitchState_e l;
        RCSwitchState_e r;
    }__packed switch_;
    
    
    explicit Remote(UART_HandleTypeDef *huart);
    ~Remote() = default;
    
    void init(void);
    void reset(void);
    void rxMsgCallback(uint8_t* rx_data_);
    bool rxMsgCheck(UART_HandleTypeDef* huart) const;
    void handle(void);
    
};
extern Remote remote;
#endif //UART_REMOTE_H