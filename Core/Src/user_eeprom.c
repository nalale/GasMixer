/*
 * user_eeprom.c
 *
 *  Created on: Jun 11, 2024
 *      Author: SoftwareEngineer_01
 */

#include <string.h>
#include <stdint.h>
#include "main.h"

static I2C_HandleTypeDef *hi2c1;

void InitEEPROM(I2C_HandleTypeDef *_hi2c1) {

	hi2c1 = _hi2c1;
}

EEPROM_Statuses_t LoadFromEEPROM(void)
{
	unsigned char Buf[64];
	unsigned short Segment = 0;
	unsigned short Offset = 0;
	unsigned short ush = 0xFFFF;
	HAL_StatusTypeDef EEPROM_Status;
	//read EEPROM header
	EEPROM_Status = HAL_I2C_Mem_Read(hi2c1, EEPROM_Address, 0, I2C_MEMADD_SIZE_16BIT, (unsigned char*)&ush, 2, EEPROM_TimeOut);

  //read first data block
	if ((EEPROM_Status == HAL_OK) && (ush == EEPROM_OK)) {
		Segment = 64;
		Offset = sizeof(k1) + sizeof(b1) + sizeof(k2) + sizeof(b2) + sizeof(k3) + sizeof(b3) + sizeof(k4) + sizeof(b4);
		EEPROM_Status = HAL_I2C_Mem_Read(hi2c1, EEPROM_Address, Segment, I2C_MEMADD_SIZE_16BIT, Buf, Offset, EEPROM_TimeOut);
			if (EEPROM_Status == HAL_OK) {
				Offset = 0;
				memcpy(&k1, &Buf[Offset], sizeof(k1));
				Offset += sizeof(k1);
				memcpy(&b1, &Buf[Offset], sizeof(b1));
				Offset += sizeof(b1);
				memcpy(&k2, &Buf[Offset], sizeof(k2));
				Offset += sizeof(k2);
				memcpy(&b2, &Buf[Offset], sizeof(b2));
				Offset += sizeof(b2);
				memcpy(&k3, &Buf[Offset], sizeof(k3));
				Offset += sizeof(k3);
				memcpy(&b3, &Buf[Offset], sizeof(b3));
				Offset += sizeof(b3);
				memcpy(&k4, &Buf[Offset], sizeof(k4));
				Offset += sizeof(b4);
				memcpy(&b4, &Buf[Offset], sizeof(b4));
			}
	}
	//read second data block
	if ((EEPROM_Status == HAL_OK) && (ush == EEPROM_OK)) {
		Segment = 128;
		Offset = sizeof(TargetO2) + sizeof(TargetCO2) + sizeof(Valve1) + sizeof(Valve2) + sizeof(KoefN2) +
		 sizeof(KoefCO2) + sizeof(Volume) + sizeof(SValve_N2) + sizeof(SValve_CO2);
		EEPROM_Status = HAL_I2C_Mem_Read(hi2c1, EEPROM_Address, Segment, I2C_MEMADD_SIZE_16BIT, Buf, Offset, EEPROM_TimeOut);
		if (EEPROM_Status == HAL_OK) {
			Offset = 0;
			memcpy(&TargetO2, &Buf[Offset], sizeof(TargetO2));
			Offset += sizeof(TargetO2);
			memcpy(&TargetCO2, &Buf[Offset], sizeof(TargetCO2));
			Offset += sizeof(TargetCO2);
			memcpy(&Valve1, &Buf[Offset], sizeof(Valve1));
			Offset += sizeof(Valve1);
			memcpy(&Valve2, &Buf[Offset], sizeof(Valve2));
			Offset += sizeof(Valve2);
			memcpy(&KoefN2, &Buf[Offset], sizeof(KoefN2));
			Offset += sizeof(KoefN2);
			memcpy(&KoefCO2, &Buf[Offset], sizeof(KoefCO2));
			Offset += sizeof(KoefCO2);
			memcpy(&Volume, &Buf[Offset], sizeof(Volume));
			Offset += sizeof(Volume);
			memcpy(&SValve_N2, &Buf[Offset], sizeof(SValve_N2));
			Offset += sizeof(SValve_N2);
			memcpy(&SValve_CO2, &Buf[Offset], sizeof(SValve_CO2));
		}
	}
	if (EEPROM_Status == HAL_OK)
		return EEPROM_OK;
	else
		return EEPROM_FREE;
}

EEPROM_Statuses_t SaveToEEPROM(void)
{
	unsigned char Buf[64];
	unsigned short Segment = 0;
	unsigned short Offset = 0;
	unsigned short ush;
	HAL_StatusTypeDef EEPROM_Status;
	//write Block EEPROM header
	ush = EEPROM_FREE;
	EEPROM_Status = HAL_I2C_Mem_Write(hi2c1, EEPROM_Address, 0, I2C_MEMADD_SIZE_16BIT, (unsigned char*)&ush, 2, EEPROM_TimeOut);
	HAL_Delay(5);
	//write first data block
	if (EEPROM_Status == HAL_OK) {
		Segment = 64;
		Offset = 0;
		memcpy(&Buf[Offset], &k1, sizeof(k1));
		Offset += sizeof(k1);
		memcpy(&Buf[Offset], &b1, sizeof(b1));
		Offset += sizeof(b1);
		memcpy(&Buf[Offset], &k2, sizeof(k2));
		Offset += sizeof(k2);
		memcpy(&Buf[Offset], &b2, sizeof(b2));
		Offset += sizeof(b2);
		memcpy(&Buf[Offset], &k3, sizeof(k3));
		Offset += sizeof(k3);
		memcpy(&Buf[Offset], &b3, sizeof(b3));
		Offset += sizeof(b3);
		memcpy(&Buf[Offset], &k4, sizeof(k4));
		Offset += sizeof(b4);
		memcpy(&Buf[Offset], &b4, sizeof(b4));
		Offset += sizeof(b4);
		EEPROM_Status = HAL_I2C_Mem_Write(hi2c1, EEPROM_Address, Segment, I2C_MEMADD_SIZE_16BIT, Buf, Offset, EEPROM_TimeOut);
		HAL_Delay(5);
	}
 //write second data block
	if (EEPROM_Status == HAL_OK) {
		Segment = 128;
		Offset = 0;
		memcpy(&Buf[Offset], &TargetO2, sizeof(TargetO2));
		Offset += sizeof(TargetO2);
		memcpy(&Buf[Offset], &TargetCO2, sizeof(TargetCO2));
		Offset += sizeof(TargetCO2);
		memcpy(&Buf[Offset], &Valve1, sizeof(Valve1));
		Offset += sizeof(Valve1);
		memcpy(&Buf[Offset], &Valve2, sizeof(Valve2));
		Offset += sizeof(Valve2);
		memcpy(&Buf[Offset], &KoefN2, sizeof(KoefN2));
		Offset += sizeof(KoefN2);
		memcpy(&Buf[Offset], &KoefCO2, sizeof(KoefCO2));
		Offset += sizeof(KoefCO2);
		memcpy(&Buf[Offset], &Volume, sizeof(Volume));
		Offset += sizeof(Volume);
		memcpy(&Buf[Offset], &SValve_N2, sizeof(SValve_N2));
		Offset += sizeof(SValve_N2);
		memcpy(&Buf[Offset], &SValve_CO2, sizeof(SValve_CO2));
		Offset += sizeof(SValve_CO2);
		EEPROM_Status = HAL_I2C_Mem_Write(hi2c1, EEPROM_Address, Segment, I2C_MEMADD_SIZE_16BIT, Buf, Offset, EEPROM_TimeOut);
		HAL_Delay(5);
	}
	//write normal EEPROM header
	if (EEPROM_Status == HAL_OK) {
		ush = EEPROM_OK;
		EEPROM_Status = HAL_I2C_Mem_Write(hi2c1, EEPROM_Address, 0, I2C_MEMADD_SIZE_16BIT, (unsigned char*)&ush, 2, EEPROM_TimeOut);
		HAL_Delay(5);
	}

	if (EEPROM_Status == HAL_OK)
		return EEPROM_OK;
	else
		return EEPROM_FREE;
}
