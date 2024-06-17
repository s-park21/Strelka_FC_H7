/*
 * debug.c
 *
 *  Created on: Apr 20, 2023
 *      Author: Angus McLennan
 */

#include "debug.h"

extern micros();

int _write(int file, char *ptr, int len) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

/* Debug print function */
void debug_print(char *msg, size_t len, enum debug_level dbl) {
#ifdef DEBUG_ENABLED
	if (dbl >= dbg_level) {
		char *log_levels[] = { "DEBUG:   ", "INFO:    ", "WARNING: ", "ERROR:   ", "CRITICAL:" };
//		CDC_Transmit_FS((uint8_t*)msg, len);
	}
#elif defined(USB_LOGGING)
	// Print statement must have greater than or equal level to the global debug level set
	if(dbl == LOG_OUTPUT) {
		// Print data to USB
//		CDC_Transmit_HS((uint8_t*)msg, len);
		return;
	}
#endif
}

char sys_logs_buff[128];		// Buffer to hold the system logs that need to be written to SD
uint32_t sys_logs_pos_idx = 0;		// Varible to hold the current offset position within the buffer
bool sys_logs_rdy = false;				// Flag to indicate when sys logs are ready to be written to SD

void store_sys_log(char *log_msg, ...) {
	va_list args;
	va_start(args, log_msg);

	// Create a buffer to hold the formatted log message
	char formatted_log_msg[160];
	vsnprintf(formatted_log_msg, sizeof(formatted_log_msg), log_msg, args);

	va_end(args);

	// Add system time and new line character to the formatted log message
	char msg[160];
	size_t sz = snprintf(msg, sizeof(msg), "%lu:%s\r\n", micros(), formatted_log_msg);

	taskENTER_CRITICAL();
	if (sys_logs_pos_idx + sz + 1 < sizeof(sys_logs_buff)) {
		memcpy(&sys_logs_buff[sys_logs_pos_idx], msg, sz);
		sys_logs_pos_idx += sz;
	}
	taskEXIT_CRITICAL();

	sys_logs_rdy = true;
}

void write_sys_logs() {
	SD_write_sys_logs_batch((uint8_t*) sys_logs_buff, sys_logs_pos_idx);
	sys_logs_pos_idx = 0;
	sys_logs_rdy = false;
}
