/*
 * ASM330.c
 *
 *  Created on: Nov 25, 2023
 *      Author: thean
 */

#include "ASM330.h"

GPIO_TypeDef *CS_up_GPIO_Port;
uint16_t CS_up_Pin;

stmdev_ctx_t dev_ctx;
asm330lhhx_pin_int1_route_t int1_route;
asm330lhhx_pin_int2_route_t int2_route;

/* Platform specific functions */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

uint8_t ASM330_Init(ASM330_handle *asm330) {
	static uint8_t whoamI, rst;

	CS_up_GPIO_Port = asm330->CS_GPIO_Port;
	CS_up_Pin = asm330->CS_Pin;
//	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

	asm330->dev_ctx.write_reg = platform_write;
	asm330->dev_ctx.read_reg = platform_read;
	asm330->dev_ctx.handle = asm330->hspi;

	osDelay(pdMS_TO_TICKS(BOOT_TIME));

	/* Check device ID. */
	asm330lhhx_device_id_get(&asm330->dev_ctx, &whoamI);
	if (whoamI != ASM330LHHX_ID) {
		asm330->acc_good = false;
		asm330->gyro_good = false;
		return 1;
	}
	asm330->acc_good = true;
	asm330->gyro_good = true;

	/* Restore default configuration. */
	asm330lhhx_reset_set(&asm330->dev_ctx, PROPERTY_ENABLE);
	do {
		asm330lhhx_reset_get(&asm330->dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface. */
	asm330lhhx_i3c_disable_set(&asm330->dev_ctx, ASM330LHHX_I3C_DISABLE);

	/* Enable Block Data Update. */
	asm330lhhx_block_data_update_set(&asm330->dev_ctx, PROPERTY_DISABLE);

	/* Set Output Data Rate. */
	asm330lhhx_xl_data_rate_set(&asm330->dev_ctx, asm330->accel_odr);
	asm330lhhx_gy_data_rate_set(&asm330->dev_ctx, asm330->gyro_odr);

	/* Set full scale. */
	asm330lhhx_xl_full_scale_set(&asm330->dev_ctx, asm330->accel_scale);
	asm330lhhx_gy_full_scale_set(&asm330->dev_ctx, asm330->gyro_scale);

	/* Enable accelerometer internal low pass filter */
	asm330lhhx_xl_filter_lp2_set(&asm330->dev_ctx, 1);
	// Set filter bandwidth to 66.67Hz at 6.67kHz sample rate
	asm330lhhx_xl_hp_path_on_out_set(&asm330->dev_ctx, ASM330LHHX_LP_ODR_DIV_100);

	/* Enable gyroscope internal low pass filter */
	asm330lhhx_gy_filter_lp1_set(&asm330->dev_ctx, 1);
	// Set bandwidth to 154Hz at 6.67kHz sample rate
	asm330lhhx_gy_lp1_bandwidth_set(&asm330->dev_ctx, 0b010);

	/* Set INT1 to accelerometer data ready */
	asm330lhhx_pin_int1_route_get(&asm330->dev_ctx, &int1_route);
	int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
	asm330lhhx_pin_int1_route_set(&asm330->dev_ctx, &int1_route);

	/* Set INT2 to gyroscope data ready */
	asm330lhhx_pin_int2_route_get(&asm330->dev_ctx, &int2_route);
	int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
	asm330lhhx_pin_int2_route_set(&asm330->dev_ctx, &int2_route);

	return 0;
}

float asm330_previous_acc_value;
uint8_t asm330_previous_acc_look_back_counter;
uint8_t asm330_invalid_acc_counter;
float asm330_previous_gyro_value;
uint8_t asm330_previous_gyro_look_back_counter;
uint8_t asm330_invalid_gyro_counter;

uint8_t ASM330_readAccel(ASM330_handle *asm330, float *accel) {
	static int16_t data_raw_acceleration[3];
	asm330lhhx_acceleration_raw_get(&asm330->dev_ctx, data_raw_acceleration);
	uint8_t ret = 1;
	switch (asm330->accel_scale) {
	case ASM330LHHX_2g:
		accel[0] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[2]) / 1000.0;
		ret = 0;
	case ASM330LHHX_4g:
		accel[0] = asm330lhhx_from_fs4g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs4g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs4g_to_mg(data_raw_acceleration[2]) / 1000.0;
		ret = 0;
	case ASM330LHHX_8g:
		accel[0] = asm330lhhx_from_fs8g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs8g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs8g_to_mg(data_raw_acceleration[2]) / 1000.0;
		ret = 0;
	case ASM330LHHX_16g:
		accel[0] = asm330lhhx_from_fs16g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs16g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs16g_to_mg(data_raw_acceleration[2]) / 1000.0;
		ret = 0;
	}

	/* Check accelerometer is functioning correctly */
	if(accel[0] == asm330_previous_acc_value) {
		asm330_invalid_acc_counter++;
	}
	else {
		asm330_invalid_acc_counter = 0;
	}
	asm330_previous_acc_look_back_counter++;

	if(asm330_previous_acc_look_back_counter > NUM_LOOK_BACK_CYCLES) {
		asm330_previous_acc_value = accel[0];
		asm330_invalid_acc_counter = 0;
	}
	if(asm330_invalid_acc_counter > NUM_LOOK_BACK_CYCLES) {
		// Last NUM_LOOK_BACK_CYCLES readings have been identical. Sensor is not updating
		asm330->acc_good = false;
	}
	else {
		asm330->acc_good = true;
	}
	return ret;
}

uint8_t ASM330_readGyro(ASM330_handle *asm330, float *gyro) {
	static int16_t data_raw_angular_rate[3];
	asm330lhhx_angular_rate_raw_get(&asm330->dev_ctx, data_raw_angular_rate);
	uint8_t ret = 1;
	// Convert from md/s to rad/s
	switch (asm330->gyro_scale) {
	case ASM330LHHX_125dps:
		gyro[0] = asm330lhhx_from_fs125dps_to_mdps(data_raw_angular_rate[0]) * 0.0000174533;
		gyro[1] = asm330lhhx_from_fs125dps_to_mdps(data_raw_angular_rate[1]) * 0.0000174533;
		gyro[2] = asm330lhhx_from_fs125dps_to_mdps(data_raw_angular_rate[2]) * 0.0000174533;
		ret = 0;
	case ASM330LHHX_250dps:
		gyro[0] = asm330lhhx_from_fs250dps_to_mdps(data_raw_angular_rate[0]) * 0.0000174533;
		gyro[1] = asm330lhhx_from_fs250dps_to_mdps(data_raw_angular_rate[1]) * 0.0000174533;
		gyro[2] = asm330lhhx_from_fs250dps_to_mdps(data_raw_angular_rate[2]) * 0.0000174533;
		ret = 0;
	case ASM330LHHX_500dps:
		gyro[0] = asm330lhhx_from_fs500dps_to_mdps(data_raw_angular_rate[0]) * 0.0000174533;
		gyro[1] = asm330lhhx_from_fs500dps_to_mdps(data_raw_angular_rate[1]) * 0.0000174533;
		gyro[2] = asm330lhhx_from_fs500dps_to_mdps(data_raw_angular_rate[2]) * 0.0000174533;
		ret = 0;
	case ASM330LHHX_1000dps:
		gyro[0] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[0]) * 0.0000174533;
		gyro[1] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[1]) * 0.0000174533;
		gyro[2] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[2]) * 0.0000174533;
		ret = 0;
	case ASM330LHHX_2000dps:
		gyro[0] = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) * 0.0000174533;
		gyro[1] = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) * 0.0000174533;
		gyro[2] = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) * 0.0000174533;
		ret = 0;
	case ASM330LHHX_4000dps:
		gyro[0] = asm330lhhx_from_fs4000dps_to_mdps(data_raw_angular_rate[0]) * 0.0000174533;
		gyro[1] = asm330lhhx_from_fs4000dps_to_mdps(data_raw_angular_rate[1]) * 0.0000174533;
		gyro[2] = asm330lhhx_from_fs4000dps_to_mdps(data_raw_angular_rate[2]) * 0.0000174533;
		ret = 0;
	}
	/* Check gyroscope is functioning correctly */
	if(gyro[0] == asm330_previous_gyro_value) {
		asm330_invalid_gyro_counter++;
	}
	else {
		asm330_invalid_gyro_counter = 0;
	}
	asm330_previous_gyro_look_back_counter++;

	if(asm330_previous_gyro_look_back_counter > NUM_LOOK_BACK_CYCLES) {
		asm330_previous_gyro_value = gyro[0];
		asm330_invalid_gyro_counter = 0;
	}
	if(asm330_invalid_gyro_counter > NUM_LOOK_BACK_CYCLES) {
		// Last NUM_LOOK_BACK_CYCLES readings have been identical. Sensor is not updating
		asm330->gyro_good = false;
	}
	else {
		asm330->gyro_good = true;
	}
	return ret;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, 1000);
	HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef res = HAL_SPI_Transmit(handle, &reg, 1, 1000);
	res = HAL_SPI_Receive(handle, bufp, len, 1000);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
	return res;
}
