//
// Created by lixin on 2026/10/18.
//

#include "imu.h"
#include "tim.h"
volatile uint32_t tick = 0;
volatile uint32_t last_tick = 0;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM6) return;
    tick++;
}
extern "C" void HANDLE(void) {
    uint32_t now = tick;
    if (now == last_tick) return;


    imu_update(now, last_tick);
    last_tick = now;
}