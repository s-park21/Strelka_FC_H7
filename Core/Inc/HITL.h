/*
 * HITL.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Angus McLennan
 */

#ifndef INC_HITL_H_
#define INC_HITL_H_

#include <stdint.h>
#include "main.h"
#include "BMX055.h"
#include "MS5611.h"
#include "Sensors.h"
#include "ASM330.h"
#include "fatfs.h"
#include "stdio.h"
#include "string.h"
#include "State_Controller.h"

extern uint32_t micros();
extern BMX055_Handle bmx055;
extern ASM330_handle asm330;
extern MS5611_Handle ms5611;
extern System_State_t system_state;

extern BMX055_Data_Handle bmx055_data;
extern ASM330_Data_Handle asm330_data;
extern MS5611_Data_Handle ms5611_data;
extern ADXL375_Data_Handle adxl375_data;
extern bool sensors_initialised;
extern osSemaphoreId_t SDMMCSemaphoreHandle;

uint8_t HITL_init();
void HITL_read_sensors();

#endif /* INC_HITL_H_ */
