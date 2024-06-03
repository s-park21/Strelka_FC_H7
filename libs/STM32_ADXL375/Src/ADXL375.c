#include "ADXL375.h"

/*
    Function: ADXL375_init
    Initialises the ADXL375 accelerometer
    Inputs:
    ADXL375_t *adxl: Device struct
    float32_t offsetX: Two's complement offset measured in units of G (9.81m/s2)
    float32_t offsetY: Two's complement offset measured in units of G (9.81m/s2)
    float32_t offsetZ: Two's complement offset measured in units of G (9.81m/s2)
    returns: ADXL375_state_t
*/
ADXL375_state_t ADXL375_init(ADXL375_t *adxl, float offsetX, float offsetY, float offsetZ)
{
	HAL_GPIO_WritePin(adxl->CS_port, adxl->CS_pin, GPIO_PIN_SET);
    // Read device ID
    uint8_t dev_id;
    ADXL375_readSPI(adxl, (uint8_t)DEVICE_ID, &dev_id, sizeof(dev_id));
    if (dev_id != ADXL375_DEVICE_ID)
    {
    	adxl->acc_good = false;
        return ADXL375_NOT_FOUND;
    }

    // Disable interrupts
    uint8_t data = 0;
    ADXL375_writeSPI(adxl, (uint8_t)INT_ENABLE, &data, sizeof(data));

    // Add calibration offsets
    // Offsets are added to the readings each cycle
    uint8_t offset_x = (uint8_t)(offset_x / 0.196);
    uint8_t offset_y = (uint8_t)(offset_y / 0.196);
    uint8_t offset_z = (uint8_t)(offset_z / 0.196);
    ADXL375_writeSPI(adxl, (uint8_t)OFSX, &offset_x, sizeof(offset_x));
    ADXL375_writeSPI(adxl, (uint8_t)OFSY, &offset_y, sizeof(offset_y));
    ADXL375_writeSPI(adxl, (uint8_t)OFSZ, &offset_z, sizeof(offset_z));

    // Define sensor sample rate
    data = 0 | adxl->sample_rate;
    ADXL375_writeSPI(adxl, (uint8_t)BW_RATE, &data, sizeof(data));

    // Configure data format
    data = 0b00101011;
    ADXL375_writeSPI(adxl, (uint8_t)DATA_FORMAT, &data, sizeof(data));

    // Set FIFO buffer to stream mode and set ddry watermark to 0 bytes before triggering
    data = 0b10000000;
    ADXL375_writeSPI(adxl, (uint8_t)FIFO_CTL, &data, sizeof(data));

    // Set data ready interrupt to INT1
    data = 0b01111111;
    ADXL375_writeSPI(adxl, (uint8_t)INT_MAP, &data, sizeof(data));

    // Enable data ready interrupt
    data = 0b10000000;
    ADXL375_writeSPI(adxl, (uint8_t)INT_MAP, &data, sizeof(data));

    adxl->acc_good = true;
    return ADXL375_OK;
}

/*
    Function: ADXL375_readSensor
    Reads the sensor values from the ADXL375 FIFO
    Inputs:
    ADXL375_t *adxl: Device struct
    float *accel (output): A pointer to an array of data points in the form { accX, accY, accZ }
*/
ADXL375_state_t ADXL375_readSensor(ADXL375_t *adxl, float *accel)
{
    // Read all available data from FIFO
    uint8_t FIFO_data[6];
    ADXL375_readSPI(adxl, (uint8_t)DATAX0, FIFO_data, sizeof(FIFO_data));

    float scale_factor = 0.049; // g/LSB
    switch (adxl->sample_rate)
    {
    case ADXL375_RATE_3200Hz:
        // TODO Find scale factor for this rate
        scale_factor = 0.049;
        break;
    case ADXL375_RATE_1600Hz:
        // TODO Find scale factor for this rate
        scale_factor = 0.049;
        break;
    default:
        scale_factor = 0.049;
        break;
    }
    accel[0] = (float)((FIFO_data[0] | ((FIFO_data[1] << 8) & 0xFF00)) * scale_factor);
    accel[1] = (float)((FIFO_data[2] | ((FIFO_data[3] << 8) & 0xFF00)) * scale_factor);
    accel[2] = (float)((FIFO_data[4] | ((FIFO_data[5] << 8) & 0xFF00)) * scale_factor);

    return ADXL375_OK;
}

/**
 * @brief Write SPI Data
 * @param [in] CS_Port
 * @param [in] CS_Pin
 * @param [in] register_addr
 * @param [in] data
 * @param [in] len
 */
void ADXL375_writeSPI(ADXL375_t *adxl, uint8_t register_addr, uint8_t *data, size_t len)
{
    HAL_GPIO_WritePin(adxl->CS_port, adxl->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adxl->hspi, &register_addr, 1, 1000);
    HAL_SPI_Transmit(adxl->hspi, data, len, 1000);
    HAL_GPIO_WritePin(adxl->CS_port, adxl->CS_pin, GPIO_PIN_SET);
}

/**
 * @brief Read SPI Data
 * @param [in] device Device type (gyro, accel or mag)
 * @param [in] register_addr Register Address
 * @param [in] num Data Length
 * @param [out] *buf Read Data
 */
void ADXL375_readSPI(ADXL375_t *adxl, uint8_t register_addr, uint8_t *data, size_t len)
{
    // Add RW bit to start of register
    register_addr = register_addr | 0x80;
    uint8_t packet[20];

    HAL_GPIO_WritePin(adxl->CS_port, adxl->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adxl->hspi, &register_addr, 1, 1000);
    HAL_SPI_Receive(adxl->hspi, packet, len, 1000);
    HAL_GPIO_WritePin(adxl->CS_port, adxl->CS_pin, GPIO_PIN_SET);

    // Copy data into "data" spot in memory
    memcpy(data, packet, len);
}
