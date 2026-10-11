//
// Created by lixin on 2026/10/3.
//

//在callback进行调参！！！！



#include "tim.h"
extern "C"{
    #include "can_user.h"
}
#include "motor.h"

Motor motor;
volatile uint8_t stop_flag = 0;
extern "C" void INIT(void) {
    motor.init(0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Motor::POSITION_SPEED);
    motor.setPosition(0.0f,0.0f,0.0f);
    if ((HAL_CAN_ConfigFilter(&hcan1,&can_filter_config) == HAL_OK)&&(
        HAL_CAN_Start(&hcan1) == HAL_OK)&&(
        HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)) {
        __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim6);
    }
}

extern "C" void HANDLE(void) {
    if (stop_flag != 0)
    {
        motor.stop();
    }
    else
    {
        motor.handle();
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return;
    if (HAL_CAN_AddTxMessage(&hcan1,&tx_header,motor.getTxData(),&can_tx_mailbox) != HAL_OK) stop_flag = 1;
}

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
       // motor.setTxCurrent(1.06f, 1);//电机转动
    }
}

