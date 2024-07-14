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
        UINT bytes_read;
        if (osSemaphoreAcquire(SDMMCSemaphoreHandle, 2000) == osOK) {
            FRESULT res = f_open(&SDFile, "/HITL/data.csv", FA_READ);
            res = f_lseek(&SDFile, lastReadPos);
            if (f_eof(&SDFile)) {
                Non_Blocking_Error_Handler();
            }
            if (res == FR_OK) {
                while (HITL_timestamp < micros()) {
                    char read_chr = '\0';
                    while (read_chr != '\n') {
                        res = f_read(&SDFile, &read_chr, sizeof(read_chr), &bytes_read);
                    }

                    char *read_line = (char *)malloc(28 * sizeof(char));
                    uint8_t num_chars = 0;
                    read_chr = '\0';
                    while (read_chr != ',') {
                        res = f_read(&SDFile, &read_chr, sizeof(read_chr), &bytes_read);
                        if (num_chars < 28) {
                            read_line[num_chars] = read_chr;
                            num_chars++;
                        }
                    }
                    char *endptr;
                    HITL_timestamp = strtol(read_line, &endptr, 10);
                    free(read_line);
                }

                if (HITL_timestamp > micros()) {
                    while ((int)HITL_timestamp - (int)micros() > 100) {
                        osDelay(10);
                    }
                }

                char *read_line = (char *)malloc(512 * sizeof(char));
                uint32_t num_chars = 0;
                char read_chr = '\0';
                while (read_chr != '\n') {
                    res = f_read(&SDFile, &read_chr, sizeof(read_chr), &bytes_read);
                    if (num_chars < 512) {
                        read_line[num_chars] = read_chr;
                        num_chars++;
                    }
                }
                read_line[num_chars + 1] = '\0';

                float *data_numbers = (float *)malloc(21 * sizeof(float));
                char *endptr = read_line;
                bool decode_good = true;
                for (int i = 0; i < 21; i++) {
                    char *prevptr = endptr;
                    float read_num = strtod(prevptr, &endptr);
                    if (*endptr == ',') {
                        endptr++;
                    }
                    if (endptr != prevptr && read_num != HUGE_VAL && read_num != -HUGE_VAL) {
                        data_numbers[i] = read_num;
                    } else {
                        decode_good = false;
                        break;
                    }
                    if (*endptr == '\0' || *endptr == '\r' || *endptr == '\n') {
                        decode_good = true;
                        break;
                    }
                }
                if (decode_good) {
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
                    if(first_run) {
                        first_run = false;
                        system_state.starting_altitude = ms5611_data.altitude;
                        system_state.starting_pressure = ms5611_data.pressure;
                        system_state.starting_temperature = ms5611_data.temperature;
                    }
                }
                free(data_numbers);
                free(read_line);
                lastReadPos = f_tell(&SDFile);
                res = f_close(&SDFile);
            }
            osSemaphoreRelease(SDMMCSemaphoreHandle);
        }
    }
}
