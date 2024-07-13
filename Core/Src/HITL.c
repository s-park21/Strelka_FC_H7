/*
 * HITL.c
 *
 *  Created on: Jun 22, 2024
 *      Author: Angus McLennan
 */

#include "HITL.h"

bool do_hitl;
FSIZE_t lastReadPos = 0;
bool first_run = true;
uint32_t HITL_timestamp;

uint8_t HITL_init() {
	do_hitl = false;
	HITL_timestamp = 0;
	bool first_run = true;
	FSIZE_t lastReadPos = 0;
	FILINFO dir_info;
	FRESULT fr;
	fr = f_stat("/HITL", &dir_info);
	if (fr == FR_OK) {
		do_hitl = true;
	}
	else {
		return 1;
	}
	sensors_initialised = true;
	bmx055.acc_good = true;
	bmx055.gyro_good = true;
	bmx055.mag_good = true;
	asm330.acc_good = true;
	asm330.gyro_good = true;
	ms5611.baro_good = true;
	return 0;
}

void HITL_read_sensors() {
	if (do_hitl) {
				// Open file
				UINT bytes_read;
				if (osSemaphoreAcquire(SDMMCSemaphoreHandle, 2000) == osOK) {
					FRESULT res = f_open(&SDFile, "/HITL/data.csv", FA_READ);
					// Return file pointer to last read location
					res = f_lseek(&SDFile, lastReadPos);
					if (f_eof(&SDFile)) {
						// If end of file is reached, loop forever
						Non_Blocking_Error_Handler();
					}
					if (res == FR_OK) {
						// Loop to find closest data point to current system time
						while (HITL_timestamp < micros()) {
							char read_chr = '\0';
							while (read_chr != '\n') {
								res = f_read(&SDFile, &read_chr, sizeof(read_chr), &bytes_read);
							}
							// Read time stamp
							char read_line[28];			// Assume that a timestamp cannot be longer than 28 characters
							uint8_t num_chars = 0;		// Number of characters in the timestamp string
							read_chr = '\0';
							while (read_chr != ',') {
								res = f_read(&SDFile, &read_chr, sizeof(read_chr), &bytes_read);
								read_line[num_chars] = read_chr;
								num_chars++;
							}
							// Convert timestamp string to number
							char *endptr;
							HITL_timestamp = strtol(read_line, &endptr, 10);
						}
						// Wait for the real time to catch up to within 1ms
						if (HITL_timestamp > micros()) {
							while ((int)HITL_timestamp - (int)micros() > 100) {
								osDelay(10);
							}
						}

						// Most up to date data has been found. Read a line.
						// timestamp,accel1X,accel1Z,accel1Z,accel2X,accel2Z,accel2Z,accel3X,accel3Z,accel3Z,gyro1X,gyro1Y,gyro1Z,gyro2X,gyro2Y,gyro2Z,mag1X,mag1Y,mag1Z,baro_altitude,baro_pressure,baro_temperature,gps_latitude,gps_longitude,gps_altitude\n
						char read_line[512];			// Assume that a line cannot be longer than 512 characters
						uint32_t num_chars = 0;			// Number of characters in the timestamp string
						char read_chr = '\0';
						while (read_chr != '\n') {
							res = f_read(&SDFile, &read_chr, sizeof(read_chr), &bytes_read);
							read_line[num_chars] = read_chr;
							num_chars++;
						}
						// Add end of string character
						read_line[num_chars + 1] = '\0';

						// Convert read line to numbers
						float data_numbers[21];			// Array to hold all 21 floats in a single data line
						char *endptr = read_line;
						bool decode_good = true;
						for (int i = 0; i < sizeof(data_numbers) / sizeof(float); i++) {
							char *prevptr = endptr;
							float read_num = strtod(prevptr, &endptr);
							if (*endptr == ',') {
								// Skip to next character
								endptr++;
							}
							if (endptr != prevptr && read_num != HUGE_VAL && read_num != -HUGE_VAL) {
								// A number was decoded successfully
								data_numbers[i] = read_num;
							} else {
								decode_good = false;
								break;
							}
							if (*endptr == '\0' || *endptr == '\r' || *endptr == '\n') {
								// End of string has been reached
								break;
								decode_good = true;
							}
						}
						if (decode_good) {
							// Copy the converted floats into the relevant data structures
							asm330_data.accel[0] = data_numbers[0];
							asm330_data.accel[1] = data_numbers[1];
							asm330_data.accel[2] = data_numbers[2];
							bmx055_data.accel[0] = data_numbers[3];
							bmx055_data.accel[1] = data_numbers[4];
							bmx055_data.accel[2] = data_numbers[5];
							adxl375_data.accel[0] = data_numbers[6];
							adxl375_data.accel[1] = data_numbers[7];
							adxl375_data.accel[2] = data_numbers[8];
							asm330_data.gyro[0] = data_numbers[9];
							asm330_data.gyro[1] = data_numbers[10];
							asm330_data.gyro[2] = data_numbers[11];
							bmx055_data.gyro[0] = data_numbers[12];
							bmx055_data.gyro[1] = data_numbers[13];
							bmx055_data.gyro[2] = data_numbers[14];
							bmx055_data.mag[0] = data_numbers[15];
							bmx055_data.mag[1] = data_numbers[16];
							bmx055_data.mag[2] = data_numbers[17];
							ms5611_data.altitude = data_numbers[18];
							ms5611_data.pressure = data_numbers[19];
							ms5611_data.temperature = data_numbers[20];
							// TODO: Find a way to put GPS data into data structures

							if(first_run) {
								first_run = false;
								system_state.starting_altitude = ms5611_data.altitude;
								system_state.starting_pressure = ms5611_data.pressure;
								system_state.starting_temperature = ms5611_data.temperature;
							}
						}
						lastReadPos = f_tell(&SDFile);
						res = f_close(&SDFile);
					}
					osSemaphoreRelease(SDMMCSemaphoreHandle);
				}
			}

}
