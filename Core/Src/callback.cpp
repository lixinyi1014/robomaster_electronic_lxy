//
// Created by lixin on 2026/10/2.
//
#include <cstring>
#include "main.h"
#include "usart.h"
#include "remote.h"
extern uint8_t rx_msg[10];
static uint8_t tx_msg[10];

void  HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
   /* if (huart == &huart1) {
        memcpy(tx_msg, rx_msg, 10);
        if (rx_msg[0] == 'R') HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        else if (rx_msg[0] == 'M') HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
       // HAL_UART_Receive_IT(&huart1, rx_msg, 1);
        HAL_UART_Receive_DMA(&huart1, rx_msg, 10);
        HAL_UART_Transmit_DMA(&huart1, tx_msg, 10);
    }*/
    if (huart == &huart1 &&
        (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE
            || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC)) {
        HAL_UART_Transmit_IT(&huart1, rx_msg, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_msg,10);
    }
    if (remote.rxMsgCheck(huart)) {
        if (Size == RC_FRAME_LEN) {
            remote.rxMsgCallback(huart->pRxBuffPtr);
        }
        else {
            remote.init();
        }
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        HAL_UART_AbortReceive(huart);
        remote.init();
    }
}