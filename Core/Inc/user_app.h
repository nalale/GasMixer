/*
 * user.app.h
 *
 *  Created on: Jun 11, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_USER_APP_H_
#define INC_USER_APP_H_

void InitRelays(TIM_HandleTypeDef *hpwmO2, TIM_HandleTypeDef *hpwmN2);
void UpdateMesure(void);
uint8_t CreateSchedule(void);
void ProcSchedule(void);

void OpenN2(void);
void OpenCO2(void);
void CloseN2(void);
void CloseCO2(void);



#endif /* INC_USER_APP_H_ */
