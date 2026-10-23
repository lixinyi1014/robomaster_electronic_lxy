//
// Created by lixin on 2026/10/18.
//
#include "main.h"
#include "bmi.h"
#include "spi.h"


void bmi088_write_byte(uint8_t tx_data) {
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&tx_data, 1, 1000);
}

void bmi088_read_byte(uint8_t *rx_data, uint8_t length) {
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}
// 向寄存器中写入数据，可参考5.3，5.4寄存器写入方法，先写首位判别符+地址，再写data
void bmi088_write_reg(uint8_t reg, uint8_t data){
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);
}

void BMI088_ACCEL_NS_L(void) {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void) {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void) {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void) {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

// 参考: acc写入，相当于加上片选的 bmi088_write_reg 函数
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);

    BMI088_ACCEL_NS_H();
}

// 加速度计读取，注意需要忽略第一个字节dummy byte
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length){
    uint8_t dummy;
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(&dummy,1);
    bmi088_read_byte(rx_data,length);

    BMI088_ACCEL_NS_H();
}
// 陀螺仪读取
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(rx_data,length);

    BMI088_GYRO_NS_H();

}
// gyro写入
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data)
{
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_reg(reg, tx_data);

    BMI088_GYRO_NS_H();
}

uint8_t bmi088_init(void) {
    uint8_t dummy = 0;
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t acc_power[2] = {0};
    uint8_t acc_config[2] = {0};
    uint8_t gyro_config[3] = {0};

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    HAL_Delay(1);

    // Switch ACCEL to SPI，第一次读到的值不用
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Soft Reset ACCEL
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(1);

    // Soft Reset GYRO
    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(30);

    // ACCEL 软复位后再做一次虚拟读取
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Check Chip ID
    bmi088_accel_read_reg(0x00, &acc_id, 1);
    if (acc_id != 0x1E) {
        return 0;
    }
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) {
        return 0;
    }
    bmi088_accel_write_single_reg(0x53,0x08);//INT1_IO_CTRL输出使能
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x58,0x04);//数据就绪
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x15,0x80);
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x16,0x00);
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x18,0x01);
    HAL_Delay(1);


    // Switch ACCEL to Active Mode
    bmi088_accel_write_single_reg(0x7C, 0x00); // ACC_PWR_CONF
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7D, 0x04); // ACC_PWR_CTRL
    HAL_Delay(5);

    // ACCEL Range / ODR / BW
    bmi088_accel_write_single_reg(0x41, 0x01); // ACC_RANGE
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x40, 0xAB); // ACC_CONF
    HAL_Delay(40);

    // GYRO Power / Range / ODR / BW
    bmi088_gyro_write_single_reg(0x11, 0x00); // GYRO_LPM1
    HAL_Delay(30);
    bmi088_gyro_write_single_reg(0x0F, 0x00); // GYRO_RANGE
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x10, 0x02); // GYRO_BANDWIDTH
    HAL_Delay(10);

    // Read back，连续读取相邻的配置寄存器
    bmi088_accel_read_reg(0x7C, acc_power, 2);
    if (acc_power[0] != 0x00 || acc_power[1] != 0x04) {
        return 0;
    }
    bmi088_accel_read_reg(0x40, acc_config, 2);
    if (acc_config[0] != 0xAB || (acc_config[1] & 0x03) != 0x01) {
        return 0;
    }
    bmi088_gyro_read_reg(0x0F, gyro_config, 3);
    if ((gyro_config[0] & 0x07) != 0x00 ||
        (gyro_config[1] & 0x07) != 0x02 || gyro_config[2] != 0x00) {
        return 0;
        }

    return 1;
}

#define DEV_NONE 0
#define DEV_ACC 1
#define DEV_GYRO 2

static uint8_t acc_tx[8] = {0x12 | 0x80};
static uint8_t acc_rx[8];
static uint8_t gyro_tx[7] = {0x02 | 0x80};
static uint8_t gyro_rx[7];

static volatile uint8_t dma_ready = 0;
static volatile uint8_t spi_busy = 0;
static volatile uint8_t spi_device = DEV_NONE;
static volatile uint8_t acc_pending = 0;
static volatile uint8_t gyro_pending = 0;
static volatile uint8_t gyro_new = 0;

static volatile int16_t acc_raw[3];
static volatile int16_t gyro_raw[3];

volatile uint32_t dma_cnt = 0;

static void dma_try_start(void) {
    if (spi_busy) return;

    if (gyro_pending) {
        gyro_pending = 0;
        spi_busy = 1;
        spi_device = DEV_GYRO;
        BMI088_ACCEL_NS_H();
        BMI088_GYRO_NS_L();
        if (HAL_SPI_TransmitReceive_DMA(&hspi1,gyro_tx,gyro_rx,7) != HAL_OK) {
            BMI088_GYRO_NS_H();
            spi_busy = 0;
            spi_device = DEV_NONE;
        }
    } else if (acc_pending) {
        acc_pending = 0;
        spi_busy = 1;
        spi_device = DEV_ACC;
        BMI088_GYRO_NS_H();
        BMI088_ACCEL_NS_L();
        if (HAL_SPI_TransmitReceive_DMA(&hspi1,acc_tx,acc_rx,8) != HAL_OK) {
            BMI088_ACCEL_NS_H();
            spi_busy = 0;
            spi_device = DEV_NONE;
        }
    }
}

void bmi088_exti_handler(uint16_t pin) {
    if (!dma_ready) return;
    if (pin == INT1_ACCEL_Pin) acc_pending = 1;
    else if (pin == INT1_GYRO_Pin) gyro_pending = 1;
    else return;
    dma_try_start();
}

void bmi088_spi_done_handler(void) {
    if (spi_device == DEV_GYRO) {
        BMI088_GYRO_NS_H();
        for (int i = 0; i < 3; i++) {
            gyro_raw[i] = (int16_t)((gyro_rx[2*(i+1)]<<8|gyro_rx[2*i+1]));
        }
        gyro_new =1;
    }
    else if (spi_device == DEV_ACC) {
        BMI088_ACCEL_NS_H();
        for (int i = 0; i < 3; i++) {
            acc_raw[i] = (int16_t)((acc_rx[2*(i+1)+1]<<8|acc_rx[2*(i+1)]));
        }
    }
    dma_cnt++;
    spi_device = DEV_NONE;
    spi_busy = 0;
    dma_try_start();
}

void bmi088_spi_error_handler(void) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    spi_busy = 0;
    spi_device = DEV_NONE;
}

void bmi088_dma_start(void) {
    __disable_irq();
    dma_ready = 1;
    acc_pending = 1;
    gyro_pending = 1;
    dma_try_start();
    __enable_irq();
}

uint8_t bmi088_get_raw(int16_t acc[3], int16_t gyro[3]) {
    if (!gyro_new) return 0;
    __disable_irq();
    for (int i = 0;i<3;i++) {
        acc[i] = acc_raw[i];
        gyro[i] = gyro_raw[i];
    }
    gyro_new = 0;
    __enable_irq();
    return 1;

}