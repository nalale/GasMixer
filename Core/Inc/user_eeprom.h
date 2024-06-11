/*
 * user_eeprom.h
 *
 *  Created on: Jun 11, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_USER_EEPROM_H_
#define INC_USER_EEPROM_H_

void InitEEPROM(I2C_HandleTypeDef *_hi2c1);
EEPROM_Statuses_t SaveToEEPROM(void);
EEPROM_Statuses_t LoadFromEEPROM(void);

#endif /* INC_USER_EEPROM_H_ */
