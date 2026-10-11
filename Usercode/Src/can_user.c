//
// Created by lixin on 2026/10/11.
//
#include "can_user.h"
CAN_RxHeaderTypeDef rx_header;

CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x200,
    .ExtId = 0x0,//扩展ID
    .IDE = CAN_ID_STD,//标识符类型，选择标准帧
    .RTR = CAN_RTR_DATA,//远程发送请求，选择数据帧代表我们传的是真实数据
    .DLC = 8,//Data Length Code
    .TransmitGlobalTime = DISABLE//禁用全局时间戳
};

uint32_t can_tx_mailbox;

CAN_FilterTypeDef can_filter_config = {
    .FilterBank =       0,
    .FilterMode =       CAN_FILTERMODE_IDMASK,//掩码模式
    .FilterScale =      CAN_FILTERSCALE_32BIT,//滤波器尺度
    .FilterIdHigh =     0x0000,
    .FilterIdLow =      0x0000,
    .FilterMaskIdHigh =  0x0000,
    .FilterMaskIdLow =  0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14
};