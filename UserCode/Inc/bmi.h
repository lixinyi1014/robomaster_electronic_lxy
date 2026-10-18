//
// Created by lixin on 2026/10/18.
//
#ifndef RM_6_BMI_H
#define RM_6_BMI_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t bmi088_init(void);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif //RM_6_BMI_H
