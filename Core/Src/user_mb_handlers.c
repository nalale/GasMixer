/*
 * mb_user_handlers.c
 *
 *  Created on: Jun 11, 2024
 *      Author: SoftwareEngineer_01
 */

#include <string.h>
#include "main.h"
//#include "CRC.h"
#include "tools.h"
#include "mb.h"
#include "mbfunc.h"
#include "mbcrc.h"

#include "user_mb_handlers.h"

#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

#define MB_PDU_FUNC_OFF     0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF     1   /*!< Offset for response data in PDU. */

typedef enum {
	userMB_Wait = 0,
	userMB_Receive,
	userMB_Execute,
} userMBStates_t;

static UART_HandleTypeDef *pUart;
static unsigned char UART2_RX_Char;
static unsigned char UART2_RX_Pos = 0;
static unsigned char UART2_TX_Pos = 0;

static uint32_t WaitFrameMs_Cnt = 0;

static unsigned char UART2_RX_Buf[UART_BUF_Size];
//static unsigned char UART2_TX_Buf[UART_BUF_Size];

static userMBStates_t userMBState;
static uint8_t _SlaveID;

// --- Func Prototype
unsigned char IsMBMessage(unsigned char* Address, unsigned char** Frame, unsigned char* Len); //return 0 on false and message length on true
unsigned char ProcessMBMessage(unsigned char* Data, unsigned char *MesLen); //return answer length
void ShiftBuffer(unsigned char *Buf, unsigned char *Len, unsigned char N);
static uint8_t userMBFrameSend(UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength);

 /* //void ProcessMessage(unsigned char *Buf, unsigned char *Len);
 * 	//void ExecCommand(int Channel, int Code, int Param);
 * 	//void SendAnswer(int Channel, int Code, int Result);
 * 	//void SendError(int Channel, int Code);
 */

void userMBInit(uint8_t ucSlaveAddress, void *dHUART, void *dHTIM) {
	pUart = dHUART;
	_SlaveID = ucSlaveAddress;
}

void userMBEnable() {
	HAL_UART_Receive_IT(pUart, &UART2_RX_Char, 1);
}

void userMBPoll() {
	static UCHAR   *ucMBFrame;
	static UCHAR   usLength;
	UCHAR ucFunctionCode;
	static UCHAR mbAddress;

	switch(userMBState) {
	case userMB_Wait:

		if(UART2_RX_Pos > 0) {
			userMBState = userMB_Receive;
			WaitFrameMs_Cnt = msTimer_GetStamp();
		}
		break;
	case userMB_Receive:
		uint8_t st = IsMBMessage(&mbAddress, &ucMBFrame, &usLength);

		if(st)
			userMBState = userMB_Execute;
		else
			if(msTimer_GetFrom(WaitFrameMs_Cnt) >= 5) {
				userMBState = userMB_Wait;
				ShiftBuffer(UART2_RX_Buf, &UART2_RX_Pos, UART2_RX_Pos);
				HAL_UART_Receive_IT(pUart, &UART2_RX_Char, 1);
			}
		break;

	case userMB_Execute:
		ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
		eMBException eException = ProcessMBMessage(ucMBFrame, &usLength);

		if( mbAddress != MB_ADDRESS_BROADCAST )
		{
			if( eException != MB_EX_NONE )
			{
				/* An exception occured. Build an error frame. */
				usLength = 0;
				ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
				ucMBFrame[usLength++] = eException;
			}

			UART2_TX_Pos = userMBFrameSend( mbAddress, ucMBFrame, usLength );
		}

		if (UART2_TX_Pos) { //Answer for modbus
			HAL_UART_Transmit_IT(pUart, (unsigned char*)UART2_RX_Buf, UART2_TX_Pos);
		}

		ShiftBuffer(UART2_RX_Buf, &UART2_RX_Pos, UART2_RX_Pos);
		HAL_UART_Receive_IT(pUart, &UART2_RX_Char, 1);
		userMBState = userMB_Wait;
		break;
	}
}



unsigned char IsMBMessage(unsigned char* Address, unsigned char** Data, unsigned char* Len)
{
	unsigned char res = 0;
//	unsigned char FuncCode = 255;
//	unsigned char MesLength = 253; //max message length before CRC

	uint8_t *ucRTUBuf = UART2_RX_Buf;
	uint8_t usRcvBufferPos = UART2_RX_Pos;
	*Address = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

	if ((*Address == _SlaveID) || (*Address == MB_BroadCast)) {
		if (usRcvBufferPos >= MB_SER_PDU_SIZE_MIN) { //min {ID,FuncCode,CRC_Lo,CRC_Hi}
/*			FuncCode = ucRTUBuf[1];
			switch (FuncCode) {
				case MB_FUNC_READ_HOLDING_REGISTER: //MesLength = 6; break;
				case MB_FUNC_READ_INPUT_REGISTER: //MesLength = 6; break;
				case MB_FUNC_WRITE_REGISTER:
					MesLength = 6;
				break;

				case MB_FUNC_WRITE_MULTIPLE_REGISTERS:
					if (usRcvBufferPos > 6)
						MesLength = 7 + ucRTUBuf[6];
				break;

				default:
					MesLength = 2; // {ID, Funccode}
					break;

			} //switch
*/
//			if (usRcvBufferPos >= MesLength + 2) {
				//check CRC
				//Crc = CRC16(Data, MesLength);
				if (usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos ) == 0 ) //((Data[MesLength] == (Crc & 0xFF)) && (Data[MesLength+1] == ((Crc >> 8) & 0xFF)))
				{
					*Len = usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC; //MesLength + 2;
					*Data = ( UCHAR * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
					res = 1;
				}
//			}
			// else wait for other bytes
		} //else Wait for other bytes
	} else {
		//Wrong SlaveID --> Erase first char
		ShiftBuffer(ucRTUBuf, &usRcvBufferPos, 1);
	}

	return res;
}

unsigned char ProcessMBMessage(unsigned char* Data, unsigned char *MesLen)
{
	unsigned char res = 0;
	uint8_t *ucMBFrame = Data;
	uint8_t *DataLen = MesLen;
	unsigned char FuncCode = ucMBFrame[MB_PDU_FUNC_OFF];
	eMBException eException = MB_EX_ILLEGAL_FUNCTION;

	switch (FuncCode) {
	case MB_FUNC_READ_HOLDING_REGISTER: //read registers 4*
		eException = eMBFuncReadHoldingRegister(ucMBFrame, (USHORT*)DataLen);
	break;

	case MB_FUNC_READ_INPUT_REGISTER: //read registers 3*
		eException = eMBFuncReadInputRegister(ucMBFrame, (USHORT*)DataLen);

	break;

	case MB_FUNC_WRITE_REGISTER: //write single reg 4*
		eException = eMBFuncWriteHoldingRegister(ucMBFrame, (USHORT*)DataLen);
	break; //command 0x06

	case MB_FUNC_WRITE_MULTIPLE_REGISTERS:
		eException = eMBFuncWriteMultipleHoldingRegister(ucMBFrame, (USHORT*)DataLen);
	break; //command 0x10

	default:  //Error - function not supported
		eException = MB_EX_ILLEGAL_FUNCTION;
	break;
	} //switch

	return res;
}

static uint8_t userMBFrameSend(UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength) {
	/* First byte before the Modbus-PDU is the slave address. */
	volatile UCHAR *pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
	volatile USHORT usSndBufferCount = 1;

	/* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
	pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
	usSndBufferCount += usLength;

	/* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
	USHORT usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
	UART2_RX_Buf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
	UART2_RX_Buf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

	return usSndBufferCount;
}


void ShiftBuffer(unsigned char *Buf, unsigned char *Len, unsigned char N)
{
	unsigned char i;

	for (i = N; i < *Len; i++)
		Buf[i-N] = Buf[N];

	if (N < *Len)
		*Len -= N;
	else
		*Len = 0;

//	Buf[*Len] = 0;
}

void userMBRxCallBack(UART_HandleTypeDef *huart)
{
	if(huart->Instance == pUart->Instance)
	{
		UART2_RX_Buf[UART2_RX_Pos] = UART2_RX_Char;
		UART2_RX_Pos++;

		HAL_UART_Receive_IT(pUart, &UART2_RX_Char, 1);
	}
}

void userMBTxCallBack(UART_HandleTypeDef *huart)
{
	if(huart->Instance == pUart->Instance)
	{

	}
}

