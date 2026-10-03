//
// Created by lixin on 2026/10/3.
//

#ifndef RM4_CAN_USER_H
#define RM4_CAN_USER_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;

#ifdef __cplusplus
}
#endif

#endif //RM4_CAN_USER_H
