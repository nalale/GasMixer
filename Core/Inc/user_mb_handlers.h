/*
 * user_mb_handlers.h
 *
 *  Created on: Jun 12, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_USER_MB_HANDLERS_H_
#define INC_USER_MB_HANDLERS_H_


#define UART_BUF_Size 		256

void userMBInit(uint8_t ucSlaveAddress, void *dHUART, void *dHTIM);
void userMBEnable(void);
void userMBPoll(void);

void userMBRxCallBack(UART_HandleTypeDef *huart);
void userMBTxCallBack(UART_HandleTypeDef *huart);

#endif /* INC_USER_MB_HANDLERS_H_ */
