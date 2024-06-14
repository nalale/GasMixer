#ifndef __DEFAULTS_H
#define __DEFAULTS_H

//Газовая постоянная
#define Gas_R 8.31446261815324
//Окружающая температура (предполагаем постоянной)
#define Tout 293.5
//показатели адиобаты
#define Adiobata_CO2 1.3
#define Adiobata_N2 1.4

#define Ro_CO2 1.839
#define Ro_N2 1.25

#define Mu_CO2 44.0
#define Mu_N2 14.0

//Объём
#define Tank_V 5.0

//Сечения
#define S_CO2 0.0000002f
#define S_N2  0.0000004f

//Критерий
#define Krit_O2 0.1f
#define Krit_CO2 0.1f

//коэффициенты датчиков "по умолчанию"
#define DEF_k1 4.16
#define DEF_b1 -2.5
#define DEF_k2 4.16
#define DEF_b2 -2.5
#define DEF_k3 6.36
#define DEF_b3 -0.51
#define DEF_k4 62.5
#define DEF_b4 -25

//напряжение АЦП
#define ADC_Volts 3.3
#define ADC_Scale 4096

double SpeedOut(double P, double adiobata);
double VOut(double S, double P, double adiobata);
double MOut(double S, double P, double adiobata, double Ro);

#define MAX_Plan 			16
#define MIX_Time 			60
#define PLAN_Update_Time 	10

#define SOST_Running 		1
#define SOST_PlanReady 		2
#define SOST_PlanStarted 	4
#define SOST_OpenN2 		256
#define SOST_OpenCO2 		512

#define CMD_NoAction 	0
#define CMD_Start 		1
#define CMD_Stop 		2

#define Version "0.1"

#define ERROR_CANT_CHANGE_ON_RUNNING 1
#define ERROR_PLAN_NOT_READY 2
#define ERROR_UNKNOWN_PARAMETR 3
#define ERROR_OUT_OF_PLAN 4

//maximum number of modbus registers in 3* and 4*
#define MB_MAX3regs (25 + 5 * MAX_Plan) //USER_REG_INPUTS_NUM
#define MB_MAX4regs	30

#define MB_Err_NoFunc 1
#define MB_Err_NoAddr 2
#define MB_Err_NotVal 3
#define MB_Err_Refuse 6
#define MB_Err_NoServ 7

#define MB_BroadCast 0
#define MB_SlaveID 0x0030

#define EEPROM_Address 0xA0
//0xA0 = (0x50 << 1)
#define EEPROM_TimeOut 10

typedef enum {
	EEPROM_OK = 0,
	EEPROM_SAVE_RQST = 1,
	EEPROM_FREE = 0xFFFF,
} EEPROM_Statuses_t;

typedef union  {
	unsigned char b[4];
	char c[4];
	unsigned short w[2];
	short s[2];
	unsigned int u;
	int i;
	float f;
} FourByte_t;

#endif
