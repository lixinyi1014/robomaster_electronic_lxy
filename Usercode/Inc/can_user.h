//
// Created by lixin on 2026/10/11.
//

#ifndef RM5_CAN_USER_H
#define RM5_CAN_USER_H

#include "can.h"

//收发帧头 + 邮箱号 + 滤波器（这些是cubeMX不生成的全局量）
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;

extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;

#endif //RM5_CAN_USER_H
