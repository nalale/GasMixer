/*
 * tools.c
 *
 *  Created on: May 30, 2024
 *      Author: SoftwareEngineer_01
 */
#include "main.h"

uint32_t msTimer_GetStamp(void) {

	return HAL_GetTick();
}

uint32_t msTimer_GetFrom(uint32_t last_ms) {

	uint32_t ms_now = msTimer_GetStamp();

	return (ms_now >= last_ms)?
		 ms_now - last_ms:
		(0xFFFFFFFF - last_ms) + ms_now;
	}

