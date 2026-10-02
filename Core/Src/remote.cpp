/**
******************************************************************************
 * @file    remote.cpp/h
 * @brief   Remote control. 遥控器
 ******************************************************************************
 * Copyright (c) 2026 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "remote.h"
#include <cstring>
constexpr uint16_t REMOTE_CONNECT_TIMEOUT = 500u; 
// Constructor 构造函数
Remote::Remote(UART_HandleTypeDef *huart): huart_(huart), connect_(REMOTE_CONNECT_TIMEOUT){
    switch_.l = RCSwitchState_e::DOWN;
    switch_.r = RCSwitchState_e::DOWN;
}

// Start UART(SBUS) receive. 打开UART接收
void Remote::init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rx_buf,36);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
}

// Reset RC data. 重置遥控器数据
void Remote::reset() {
    // Your code here
    channel_.l_col = 1024;
    channel_.r_col = 1024;
    channel_.l_row = 1024;
    channel_.r_row = 1024;
    channel_.dial_wheel = 1024;
    switch_.l = RCSwitchState_e::DOWN;
    switch_.r = RCSwitchState_e::DOWN;
}

// Check for uart correspondence. 检查串口是否匹配
bool Remote::rxMsgCheck(UART_HandleTypeDef *huart) const {
    // Your code here 
    return huart == huart_;
}

// Update connect status, restart UART(SBUS) receive.
// 更新连接状态，重新打开UART(SBUS)接收
void Remote::rxMsgCallback(uint8_t* rx_data_){
    // Your code here 
    memcpy(this->rx_data_, rx_data_,RC_FRAME_LEN);
    connect_.refresh();
    init();
}

// Unpack data. 数据解包
void Remote::handle() {
    // Your code here 
    if (!connect_.check()) {
        reset();
        return;
    }
    const uint8_t *d = rx_data_;
    uint16_t ch0  = ( d[0]| (d[1] << 8)) & 0x07FF;
    uint16_t ch1  = ((d[1] >> 3) | (d[2] << 5)) & 0x07FF;
    uint16_t ch2  = ((d[2] >> 6) | (d[3] << 2) | (d[4] << 10)) & 0x07FF;
    uint16_t ch3  = ((d[4] >> 1) | (d[5] << 7)) & 0x07FF;
    uint8_t  sw_r = (d[5] >> 4) & 0x03;
    uint8_t  sw_l = (d[5] >> 6) & 0x03;
    uint16_t dial = d[16] | (d[17] << 8);
    channel_.r_row      = ch0;
    channel_.r_col      = ch1;
    channel_.l_row      = ch2;
    channel_.l_col      = ch3;
    channel_.dial_wheel = dial;
    switch_.r = static_cast<RCSwitchState_e>(sw_r);
    switch_.l = static_cast<RCSwitchState_e>(sw_l);
}
Remote remote(&huart3);
extern "C" void remote_init(void){ remote.init(); }
extern "C" void remote_handle(void){remote.handle();}