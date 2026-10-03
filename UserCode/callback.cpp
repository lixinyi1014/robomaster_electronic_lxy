//
// Created by lixin on 2026/10/3.
//
#include "tim.h"
#include "can_user.h"
#include "Motor.hpp"

Motor motor(19.2f);
constexpr uint32_t kMotor1FeedbackId = 0x201;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan){
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,data)!=HAL_OK) return;
    //现在我们拿到了data
    if (rx_header.IDE==CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA
        && rx_header.DLC == 8 && rx_header.StdId == kMotor1FeedbackId){
        motor.canRxMsgCallback(data);
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,motor.getTxData(),&can_tx_mailbox);
        motor.setTxCurrent(1.5f, 1);//电机转动
    }
}

