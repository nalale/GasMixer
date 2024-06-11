/*
 * mb_user_handlers.c
 *
 *  Created on: Jun 11, 2024
 *      Author: SoftwareEngineer_01
 */

#include <stdlib.h>
#include "main.h"

/*// --- Func Prototype
 * unsigned char IsMBMessage(unsigned char* Data, unsigned char* Len); //return 0 on false and message length on true
 * unsigned char ProcessMBMessage(unsigned char* Data, unsigned char MesLen, unsigned char* Answer); //return answer length
 * void ShiftBuffer(unsigned char *Buf, unsigned char *Len, unsigned char N);
 * //void ProcessMessage(unsigned char *Buf, unsigned char *Len);
 * //void ExecCommand(int Channel, int Code, int Param);
 * //void SendAnswer(int Channel, int Code, int Result);
 * //void SendError(int Channel, int Code);
 */



/*
unsigned char IsMBMessage(unsigned char* Data, unsigned char* Len)
{ unsigned char res = 0;
	unsigned char FuncCode = 255;
	unsigned char MesLength = 253; //max message length before CRC
	unsigned short Crc;
	if ((*Len) && ((Data[0] == SlaveID) || (Data[0] == MB_BroadCast))) {
			if (*Len > 3) { //min {ID,FuncCode,CRC_Lo,CRC_Hi}
				FuncCode = Data[1];
				switch (FuncCode) {
					case 0x03: //MesLength = 6; break;
					case 0x04: //MesLength = 6; break;
					case 0x06: MesLength = 6; break;
					case 0x10: if (*Len > 6) MesLength = 7 + Data[6]; break;
					default: MesLength = 2; // {ID, Funccode}
				} //switch
				if (*Len >= MesLength + 2) { //check CRC
					Crc = CRC16(Data, MesLength);
					if ((Data[MesLength] == (Crc & 0xFF)) && (Data[MesLength+1] == ((Crc >> 8) & 0xFF)))
						res = MesLength+2;
					else ShiftBuffer(Data, Len, MesLength+2); //erase all message on wrong CRC
				}
			} //Len > 3
	} else { //Wrong SlaveID --> Erase first char
		ShiftBuffer(Data, Len, 1);
	}
	return res;
}

unsigned char ProcessMBMessage(unsigned char* Data, unsigned char MesLen, unsigned char* Answer)
{
	unsigned char res = 0;
	unsigned short crc, AddrReg, NReg, T;
	unsigned char FuncCode = Data[1];
	FourByte_t FB;
	unsigned char ParseError = 0;
	int i;
	switch (FuncCode) {
	  case 0x03: //read registers 4*
		AddrReg = (Data[2] << 8) | Data[3];
		NReg = (Data[4] << 8) | Data[5];
		AddrReg -= 40000;
		if ((AddrReg + NReg) <= MB_MAX4regs)
				{
					Answer[0] = SlaveID;
					Answer[1] = Data[1];
					Answer[2] = NReg * 2;
					for (i = 0; i < NReg; i++) {
						T = *MB4Regs[AddrReg+i];
						Answer[3 + 2*i] = (T >> 8) & 0xFF;
						Answer[4 + 2*i] = T & 0xFF;
					}
					T = 3 + NReg * 2;
					crc = CRC16(Answer, T);
					Answer[T] = crc & 0xFF;
					Answer[T+1] = (crc >> 8) & 0xFF;
					res = T+2;
				} else { //Error - out of registers
						Answer[0] = SlaveID;
						Answer[1] = Data[1] | 128;
						Answer[2] = MB_Err_NoAddr;
						crc = CRC16(Answer, 3);
						Answer[3] = crc & 0xFF;
						Answer[4] = (crc >> 8) & 0xFF;
						res = 5;
				}
		break;
		case 0x04: //read registers 3*
		AddrReg = (Data[2] << 8) | Data[3];
		NReg = (Data[4] << 8) | Data[5];
		AddrReg -= 30000;
		if ((AddrReg + NReg) <= MB_MAX3regs)
				{
					Answer[0] = SlaveID;
					Answer[1] = Data[1];
					Answer[2] = NReg * 2;
					for (i = 0; i < NReg; i++) {
						T = *MB3Regs[AddrReg+i];
						Answer[3 + 2*i] = (T >> 8) & 0xFF;
						Answer[4 + 2*i] = T & 0xFF;
					}
					T = 3 + NReg * 2;
					crc = CRC16(Answer, T);
					Answer[T] = crc & 0xFF;
					Answer[T+1] = (crc >> 8) & 0xFF;
					res = T+2;
				} else { //Error - out of registers
						Answer[0] = SlaveID;
						Answer[1] = Data[1] | 128;
						Answer[2] = MB_Err_NoAddr;
						crc = CRC16(Answer, 3);
						Answer[3] = crc & 0xFF;
						Answer[4] = (crc >> 8) & 0xFF;
						res = 5;
				}
		break;
		case 0x06: //write single reg 4*
			AddrReg = (Data[2] << 8) | Data[3];
			AddrReg -= 40000;
			T = (Data[4] << 8) | Data[5]; //value

			if (AddrReg < MB_MAX4regs) {
				switch (AddrReg) {
					case 1: //no control for ID
								*MB4Regs[AddrReg] = T;
								memcpy(Answer, Data, 8);
								Answer[0] = SlaveID;
								res = 8;
								break;
					case 2: //4 bytes values should set as pair of registers
					case 3: //set error to unservised command
					case 4:
					case 5:
					case 10:
					case 11:
					case 13:
					case 14:
					case 15:
					case 16:
					case 17:
					case 18:
					case 19:
					case 20:
					case 21:
					case 22:
					case 23:
					case 24:
					case 25:
					case 26:
					case 27:
					case 28:
								Answer[0] = SlaveID;
								Answer[1] = Data[1] | 128;
								Answer[2] = MB_Err_NoServ;
								crc = CRC16(Answer, 3);
								Answer[3] = crc & 0xFF;
								Answer[4] = (crc >> 8) & 0xFF;
								res = 5;
								break;
					case 6: //% values <=100
					case 7:
					case 8:
					case 9:
						 if (!(Sost & SOST_Running)) {
							if (T <= 100) {
								*MB4Regs[AddrReg] = T;
								htim1.Instance->CCR1 = 30 * Valve1; //reset on all change in (6-9) - for small code
								htim2.Instance->CCR1 = 30 * Valve2;
								memcpy(Answer, Data, 8);
								Answer[0] = SlaveID;
								res = 8;
								Sost &= ~(SOST_PlanReady | SOST_PlanStarted);
							} else { //set error to not valid value
								Answer[0] = SlaveID;
								Answer[1] = Data[1] | 128;
								Answer[2] = MB_Err_NotVal;
								crc = CRC16(Answer, 3);
								Answer[3] = crc & 0xFF;
								Answer[4] = (crc >> 8) & 0xFF;
								res = 5;
							}
							} else { //running : set error to refuse
								Answer[0] = SlaveID;
								Answer[1] = Data[1] | 128;
								Answer[2] = MB_Err_Refuse;
								crc = CRC16(Answer, 3);
								Answer[3] = crc & 0xFF;
								Answer[4] = (crc >> 8) & 0xFF;
								res = 5;
					}
					break;
			case 12:
						if ((T == CMD_Start) || (T == CMD_Stop)) {
							LastCommand = T;

							if (LastCommand == CMD_Start) {
								if (Sost & SOST_PlanReady)
								{
									Sost |= SOST_Running;
									CurrentPlan = 0;
									Ms10Counter = 0;
									memcpy(Answer, Data, 8);
									Answer[0] = SlaveID;
									res = 8;
								} else
								{
									Answer[0] = SlaveID;
									Answer[1] = Data[1] | 128;
									Answer[2] = MB_Err_Refuse;
									crc = CRC16(Answer, 3);
									Answer[3] = crc & 0xFF;
									Answer[4] = (crc >> 8) & 0xFF;
									res = 5;
								}
							}
							if (LastCommand == CMD_Stop) {
								Sost &= ~(SOST_OpenCO2 | SOST_OpenN2 | SOST_Running | SOST_PlanStarted);
								CloseCO2();
								CloseN2();
								memcpy(Answer, Data, 8);
								Answer[0] = SlaveID;
								res = 8;
							}


						} else { //set error to not valid value
						Answer[0] = SlaveID;
						Answer[1] = Data[1] | 128;
						Answer[2] = MB_Err_NotVal;
						crc = CRC16(Answer, 3);
						Answer[3] = crc & 0xFF;
						Answer[4] = (crc >> 8) & 0xFF;
						res = 5;
					}
						break;
			case 29: //Save to EEPROM
					if (T == 1) {
						SaveToEEPROM();
						*MB4Regs[AddrReg] = T;
						memcpy(Answer, Data, 8);
						Answer[0] = SlaveID;
						res = 8;
					} else { //set error to not valid value
						Answer[0] = SlaveID;
						Answer[1] = Data[1] | 128;
						Answer[2] = MB_Err_NotVal;
						crc = CRC16(Answer, 3);
						Answer[3] = crc & 0xFF;
						Answer[4] = (crc >> 8) & 0xFF;
						res = 5;
						}
					break;
			default: //set default to register error
						Answer[0] = SlaveID;
						Answer[1] = Data[1] | 128;
						Answer[2] = MB_Err_NoAddr;
						crc = CRC16(Answer, 3);
						Answer[3] = crc & 0xFF;
						Answer[4] = (crc >> 8) & 0xFF;
						res = 5;
										} //switch AddrReg
		} else { //Error - out of registers
						Answer[0] = SlaveID;
						Answer[1] = Data[1] | 128;
						Answer[2] = MB_Err_NoAddr;
						crc = CRC16(Answer, 3);
						Answer[3] = crc & 0xFF;
						Answer[4] = (crc >> 8) & 0xFF;
						res = 5;
				}
		break; //command 0x06

		case 0x10:
		AddrReg = (Data[2] << 8) | Data[3];
		NReg = (Data[4] << 8) | Data[5];
		AddrReg -= 40000;
		if ((AddrReg + NReg) <= MB_MAX4regs) {
			for (i = 0; i < (Data[6] / 2); i++) { //Data[6] - length in bytes
				T = (Data[7 + 2*i] << 8) | (Data[8 + 2*i]);
				switch (AddrReg) {
					case 1: SlaveID = T; break;
					case 2: FB.w[0] = T; break;
					case 3: FB.w[1] = T;
						if (!(Sost & SOST_Running)) {
							if (FB.f <= 100.0) TargetO2 = FB.f; else ParseError = MB_Err_NotVal;
						} else ParseError = MB_Err_Refuse;
					break;
					case 4: FB.w[0] = T; break;
					case 5: FB.w[1] = T;
						if ((!(Sost & SOST_Running)) && (!ParseError)) {
						if (FB.f <= 100.0) TargetCO2 = FB.f; else ParseError = MB_Err_NotVal;
						} else ParseError = MB_Err_Refuse;
					break;
					case 6: if ((!(Sost & SOST_Running)) && (!ParseError)) {
						if (T <= 100)
						{Valve1 = T;
						 htim1.Instance->CCR1 = 30 * Valve1;
						} else ParseError = MB_Err_NotVal;
						} else ParseError = MB_Err_Refuse;
					break;
					case 7: if ((!(Sost & SOST_Running)) && (!ParseError)) {
									if (T <= 100) {Valve2 = T;
										htim2.Instance->CCR1 = 30 * Valve2;
									} else ParseError = MB_Err_NotVal;
									} else ParseError = MB_Err_Refuse;
									break;
					case 8: if ((!(Sost & SOST_Running)) && (!ParseError)) {
									if (T <= 100) KoefN2 = T; else ParseError = MB_Err_NotVal;
									} else ParseError = MB_Err_Refuse;
									break;
					case 9: if ((!(Sost & SOST_Running)) && (!ParseError)) {
									if (T <= 100) KoefCO2 = T; else ParseError = MB_Err_NotVal;
									} else ParseError = MB_Err_Refuse;
									break;
					case 10: FB.w[0] = T; break;
					case 11: FB.w[1] = T;
									if ((!(Sost & SOST_Running)) && (!ParseError)) {
									Volume = FB.f;
									} else ParseError = MB_Err_Refuse;
									break;
					case 12: LastCommand = T;
								if (!ParseError) {
								if (T == 1) {
									if (Sost & SOST_PlanReady) {Sost |= SOST_Running;
										CurrentPlan = 0;
										Ms10Counter = 0;
													} else {
													ParseError = MB_Err_Refuse;
													}
											}
									if (T == 2) {
											Sost &= ~(SOST_OpenCO2 | SOST_OpenN2 | SOST_Running | SOST_PlanStarted);
											CloseCO2();
											CloseN2();
											}
									if ((T != 1) || (T != 2)) ParseError = MB_Err_NotVal;
										}
											break;
					case 13: FB.w[0] = T; break;
					case 14: FB.w[1] = T; k1 = FB.f; break;
					case 15: FB.w[0] = T; break;
					case 16: FB.w[1] = T; b1 = FB.f; break;
					case 17: FB.w[0] = T; break;
					case 18: FB.w[1] = T; k2 = FB.f; break;
					case 19: FB.w[0] = T; break;
					case 20: FB.w[1] = T; b2 = FB.f; break;
					case 21: FB.w[0] = T; break;
					case 22: FB.w[1] = T; k3 = FB.f; break;
					case 23: FB.w[0] = T; break;
					case 24: FB.w[1] = T; b3 = FB.f; break;
					case 25: FB.w[0] = T; break;
					case 26: FB.w[1] = T; k4 = FB.f; break;
					case 27: FB.w[0] = T; break;
					case 28: FB.w[1] = T; b4 = FB.f; break;
					case 29: if (T == 1) SaveToEEPROM(); else ParseError = MB_Err_NotVal;
									break;
					default: break;
					}
				AddrReg++;
			} //for
			if (!ParseError) {
					memcpy(Answer, Data, 6);
				  Answer[0] = SlaveID;
					crc = CRC16(Answer, 6);
					Answer[6] = crc & 0xFF;
					Answer[7] = (crc >> 8) & 0xFF;
					res = 8;}
		} else { //Error = ParseError
							Answer[0] = SlaveID;
							Answer[1] = Data[1] | 128;
							Answer[2] = ParseError;
							crc = CRC16(Answer, 3);
							Answer[3] = crc & 0xFF;
							Answer[4] = (crc >> 8) & 0xFF;
							res = 5;
					}
		break; //command 0x10
		default:  //Error - function not supported
			Answer[0] = SlaveID;
		  Answer[1] = Data[1] | 128;
		  Answer[2] = MB_Err_NoFunc;
		  crc = CRC16(Answer, 3);
		  Answer[3] = crc & 0xFF;
		  Answer[4] = (crc >> 8) & 0xFF;
		  res = 5;
		break;
	} //switch
	return res;
}


void ShiftBuffer(unsigned char *Buf, unsigned char *Len, unsigned char N)
{unsigned char i;
	for (i = N; i < *Len; i++) Buf[i-N] = Buf[N];
	if (N < *Len) *Len -= N; else *Len = 0;
//	Buf[*Len] = 0;
}
*/


/*	void SendAnswer(int Channel, int Code, int Result)
	{
		char *Buf;
		char S[12];
		int L;
		switch (Channel) {
			case 2: Buf = UART2_TX_Buf; break;
			case 3: Buf = UART3_TX_Buf; break;
			default: Buf = NULL;
		}
		if (Buf != NULL) {
			Buf[0] = 'G';
			Buf[1] = 0;
			snprintf(S, 12, "%d", Code);
			Buf = strcat(Buf, S);
			Buf = strcat(Buf, ";");
			snprintf(S, 12, "%d", Result);
			Buf = strcat(Buf, S);
			L = strlen(Buf);
			Buf[L] = 0x0D;
			L++;
		switch (Channel) {
			case 2: UART2_TX_Pos = L;
							WriteUART2();
							break;
			case 3: UART3_TX_Pos = L;
							WriteUART3();
						  break;
			default: break;
		}
		} //Buf != Null
	}
*/

/*void SendError(int Channel, int Code)
	{
		char *Buf;
		char S[12];
		int L;
		switch (Channel) {
			case 2: Buf = UART2_TX_Buf; break;
			case 3: Buf = UART3_TX_Buf; break;
			default: Buf = NULL;
		}
		if (Buf != NULL) {
			Buf[0] = 'E';
			Buf[1] = 0;
			snprintf(S, 12, "%d", Code);
			Buf = strcat(Buf, S);
			L = strlen(Buf);
			Buf[L] = 0x0D;
			L++;
		switch (Channel) {
			case 2: UART2_TX_Pos = L;
							WriteUART2();
							break;
			case 3: UART3_TX_Pos = L;
							WriteUART3();
						  break;
			default: break;
		}
		} //Buf != Null
	}
*/

/*	void SendPlan(int Channel, int Element)
	{
		char *Buf;
		char S[12];
		int L;
		if ((Element <= PlanSize) && (Element >= 0)) {
		switch (Channel) {
			case 2: Buf = UART2_TX_Buf; break;
			case 3: Buf = UART3_TX_Buf; break;
			default: Buf = NULL;
		}
	if (Buf != NULL) {
			Buf[0] = 'P';
			Buf[1] = 0;
			snprintf(S, 12, "%d", Element);
			Buf = strcat(Buf, S);
			Buf = strcat(Buf, ";");
			snprintf(S, 12, "%d", Plan[Element-1].GasType);
			Buf = strcat(Buf, S);
			Buf = strcat(Buf, ";");
			snprintf(S, 12, "%4.3f", Plan[Element-1].Value);
			Buf = strcat(Buf, S);
			Buf = strcat(Buf, ";");
			snprintf(S, 12, "%d", Plan[Element-1].TimeMs /1000);
			Buf = strcat(Buf, S);
			L = strlen(Buf);
			Buf[L] = 0x0D;
			L++;
		switch (Channel) {
			case 2: UART2_TX_Pos = L;
							WriteUART2();
							break;
			case 3: UART3_TX_Pos = L;
							WriteUART3();
						  break;
			default: break;
		}
		} //Buf != Null
	} else SendError(Channel, ERROR_OUT_OF_PLAN);
	}

*/

/*	void ProcessMessage(unsigned char *Buf, unsigned char *Len)
	{ char* SepPos;
    char*		EndPos;
		int Code;
		int Param;
		int L;

		while ((Buf[0] != 'S') && (*Len > 0)) ShiftBuffer(Buf, Len, 1);

		SepPos = strchr(Buf, ';');
		EndPos = strchr(Buf, 0x0D);
		if ((EndPos != NULL) && (SepPos != NULL)) {
		 // ������������ ���������
			Code = atoi((char*)&Buf[1]);
		//	L = SepPos - Buf;
		//	Param = atoi((char*)&Buf[L]);
			Param = atoi(++SepPos);
			if (Buf == UART2_RX_Buf) ExecCommand(2, Code, Param);
			if (Buf == UART3_RX_Buf) ExecCommand(3, Code, Param);
			L = EndPos - Buf;
			ShiftBuffer(Buf, Len, L);
		}
		if (*Len >= UART_BUF_Size) ShiftBuffer(Buf, Len, 1);
	} */

/*	void SendParam(int Channel, int Param)
	{
		char *Buf;
		char S[12];
		char V[12];

		switch (Param) {
			case 10: snprintf(V, 12, "%s", Version); break;
			case 11: snprintf(V, 12, "%d", (int)(CurrentO2 * 10)); break; //10 - ��������� � ���������, 1 ���� ����� �������
			case 12: snprintf(V, 12, "%d", (int)(CurrentCO2 * 10)); break;
			case 13: snprintf(V, 12, "%d", (int)(TargetO2 * 10)); break;
			case 14: snprintf(V, 12, "%d", (int)(TargetCO2 * 10)); break;
			case 15: snprintf(V, 12, "%d", Valve1); break;
			case 16: snprintf(V, 12, "%d", Valve2); break;
			case 17: snprintf(V, 12, "%d", (int)Volume); break;
			case 18: snprintf(V, 12, "%d", ADC_Data[0]); break;
			case 19: snprintf(V, 12, "%d", ADC_Data[1]); break;
			case 20: snprintf(V, 12, "%d", ADC_Data[2]); break;
			case 21: snprintf(V, 12, "%d", ADC_Data[3]); break;
			case 22: snprintf(V, 12, "%d", Sost); break;
			case 23: snprintf(V, 12, "%d", KoefN2); break;
			case 24: snprintf(V, 12, "%d", KoefCO2); break;
			case 25: snprintf(V, 12, "%d", PlanSize); break;
			case 26: snprintf(V, 12, "%2.3f", CurrentPress1); break;
			case 27: snprintf(V, 12, "%2.3f", CurrentPress2); break;
			default: V[0] = 0;
		}

		if (V[0]) {
		int L;
		switch (Channel) {
			case 2: Buf = UART2_TX_Buf; break;
			case 3: Buf = UART3_TX_Buf; break;
			default: Buf = NULL;
		}
		if (Buf != NULL) {
			Buf[0] = 'G';
			Buf[1] = 0;
			snprintf(S, 12, "%d", Param);
			Buf = strcat(Buf, S);
			L = strlen(Buf);
			Buf[L] = ';';
			Buf[L+1] = 0;
			Buf = strcat(Buf, V);
			L = strlen(Buf);
			Buf[L] = 0x0D;
			L++;
		switch (Channel) {
			case 2: UART2_TX_Pos = L;
							WriteUART2();
							break;
			case 3: UART3_TX_Pos = L;
							WriteUART3();
						  break;
			default: break;
		}
		} //Buf != Null
	} else SendError(Channel, ERROR_UNKNOWN_PARAMETR);
	}*/

	/*void ExecCommand(int Channel, int Code, int Param)
	{
		switch (Code) {
			case 1: SendParam(Channel, Param);
							break;
			case 2: SendPlan(Channel, Param);  //Param-1
							break;
			case 3: 	//Start
								if (Param == 1) {
									if (Sost & SOST_PlanReady) {Sost |= SOST_Running;
										CurrentPlan = 0;
										Ms10Counter = 0;
								//		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
										}
									else SendError(Channel, ERROR_PLAN_NOT_READY);
									}
								break;
			case 4: 	//Stop
								if (Param == 1) {
							//	HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
								Sost &= ~(SOST_OpenCO2 | SOST_OpenN2 | SOST_Running | SOST_PlanStarted);
								CloseCO2();
								CloseN2();
								}
								break;
			case 51:  if (!(Sost & SOST_Running)) {
								TargetO2 = Param / 10.0;
								SendAnswer(Channel, 13, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			case 52: if (!(Sost & SOST_Running)) {
								TargetCO2 = Param / 10.0;
								SendAnswer(Channel, 14, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			case 53: if (!(Sost & SOST_Running)) {
								Valve1 = Param;
								htim1.Instance->CCR1 = 30 * Valve1;
								SendAnswer(Channel, 15, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			case 54: if (!(Sost & SOST_Running)) {
								Valve2 = Param;
								htim2.Instance->CCR1 = 30 * Valve2;
								SendAnswer(Channel, 16, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			case 55: if (!(Sost & SOST_Running)) {
								Volume = Param;
								SendAnswer(Channel, 17, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			case 56: 	if (!(Sost & SOST_Running)) {
								KoefN2 = Param;
								SendAnswer(Channel, 23, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			case 57: if (!(Sost & SOST_Running)) {
								KoefCO2 = Param;
								SendAnswer(Channel, 24, Param);
								Sost &= ~SOST_PlanReady;
								} else SendError(Channel, ERROR_CANT_CHANGE_ON_RUNNING);
								break;
			default: break;
		} //switch (Code)
	}
*/


