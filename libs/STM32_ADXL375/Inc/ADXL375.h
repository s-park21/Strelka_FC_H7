/*
 * ADXL375.h
 *
 *  Created on: 10 May 2024
 *  Author: Angus McLennan
 *
 */

#ifndef INC_ADXL375_H_
#define INC_ADXL375_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define DEVICE_ID 0x00
#define THRESH_SHOCK 0x1D
#define OFSX 0x1E
#define OFSY 0x1F
#define OFSZ 0x20
#define DUR 0x21
#define LATENT 0x22
#define WINDOW 0x23
#define THRESH_ACT 0x24
#define THRESH_INACT 0x25
#define TIME_INACT 0x26
#define ACT_INACT_CTL 0x27
#define SHOCK_AXES 0x2A
#define ACT_SHOCK_STATUS 0x2B
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x35
#define DATAZ1 0x37
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39

#define ADXL375_DEVICE_ID 0xE5

typedef enum
{
    ADXL375_OK,
    ADXL375_ERR,
    ADXL375_NOT_FOUND,
} ADXL375_state_t;

typedef enum
{
    ADXL375_RATE_3200Hz = 0b1111, // BW 1600 Idd 145
    ADXL375_RATE_1600Hz = 0b1110, // BW 800 Idd 90
    ADXL375_RATE_800Hz = 0b1101,  // BW 400 Idd 140
    ADXL375_RATE_400Hz = 0b1100,  // BW 200 Idd 140
    ADXL375_RATE_200Hz = 0b1011,  // BW 100 Idd 140
    ADXL375_RATE_100Hz = 0b1010,  // BW 50 Idd 140
    ADXL375_RATE_50Hz = 0b1001,   // BW 25 Idd 90
    ADXL375_RATE_25Hz = 0b1000,   // BW 12.5 Idd 60
    ADXL375_RATE_12_5Hz = 0b0111, // BW 6.25 Idd 50
    ADXL375_RATE_6_25Hz = 0b0110, // BW 3.13 Idd 40
    ADXL375_RATE_3_13Hz = 0b0101, // BW 1.56 Idd 35
    ADXL375_RATE_1_56Hz = 0b0100, // BW 0.78 Idd 35
    ADXL375_RATE_0_78Hz = 0b0011, // BW 0.39 Idd 35
    ADXL375_RATE_0_39Hz = 0b0010, // BW 0.20 Idd 35
    ADXL375_RATE_0_20Hz = 0b0001, // BW 0.10 Idd 35
    ADXL375_RATE_0_10Hz = 0b0000, // BW 0.05 Idd 35
} ADXL375_bandwidth_t;


typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *CS_port;
    uint16_t CS_pin;
    ADXL375_bandwidth_t sample_rate;
    bool acc_good;
} ADXL375_t;

#define ADXL375_MG2G_MULTIPLIER (0.049) /**< 49mg per lsb */

ADXL375_state_t ADXL375_init(ADXL375_t *adxl, float offsetX, float offsetY, float offsetZ);
ADXL375_state_t ADXL375_readSensor(ADXL375_t *adxl, float *accel);
void ADXL375_writeSPI(ADXL375_t *adxl, uint8_t register_addr, uint8_t *data, size_t len);
void ADXL375_readSPI(ADXL375_t *adxl, uint8_t register_addr, uint8_t *data, size_t len);

#endif // INC_ADXL375_H_
