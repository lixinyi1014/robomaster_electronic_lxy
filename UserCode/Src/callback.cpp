//
// Created by lixin on 2026/10/18.
//

#include "imu.h"
#include "tim.h"
#include "main.h"


volatile uint32_t tick = 0;
volatile uint32_t last_tick = 0;
volatile uint32_t acc_int_cnt = 0;
volatile uint32_t gyro_int_cnt = 0;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM6) return;
    tick++;
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    bmi088_exti_handler(GPIO_Pin);
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) bmi088_spi_done_handler();
}
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) bmi088_spi_error_handler();
}
extern "C" void HANDLE(void) {
    int16_t a[3], g[3];
    if (!bmi088_get_raw(a, g)) return;    // 没有新数据就直接返回
    imu.dmaUpdate(a, g);
}