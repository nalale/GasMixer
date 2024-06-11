/*
 * user_app.c
 *
 *  Created on: Jun 11, 2024
 *      Author: SoftwareEngineer_01
 */

#include <math.h>
#include <stdint.h>
#include "main.h"

static TIM_HandleTypeDef *htim1;		// канал PWM для N2
static TIM_HandleTypeDef *htim2;		// канал PWM для O2


static float GetPressure1(unsigned short v)
{ float res;
	res = k1*((double)v / ADC_Scale * ADC_Volts)+b1; //в атмосферах (100 кПа)
	if (res < 0.0) return 0.0;
	return res;
}

static float GetPressure2(unsigned short v)
{ float res;
	res = k2 * ((double)v / ADC_Scale * ADC_Volts) +b2;
	if (res < 0.0) res = 0.0;
	return res;
}

static float GetO2(unsigned short v)
{ float res;
	res = (k3*((double)v / ADC_Scale * ADC_Volts)+b3); //в процентах
	if (res < 0.0) res = 0.0;
	return res;
}

static float GetCO2(unsigned short v)
{
	float res;
	res = (k4*((double)v / ADC_Scale * ADC_Volts)+b4);
	if (res < 0.0) res = 0.0;
	return res;
}

static int CalcN2(int PlanIndex)
{
	uint32_t duty_ms = 0;
	//double DeltaO2 = TargetO2 - CurrentO2;
	//double TargetVO2 = Volume * TargetO2 /100; //оставляемый объем О2 //100%
	//double CurrentVO2 = Volume * CurrentO2 /100; //имеющийся О2
	//double TargetVN2 = CurrentVO2 / TargetO2 * Volume - (Volume - CurrentVO2); //Объем в литрах
	//double TargetVN2 = Volume - CurrentVO2;

	//double HaveO2 = Volume * CurrentO2 / 100; //100%

	//Объем в литрах
	float ReqO2 = Volume * TargetO2;
	float ReqAir = 0.0;

	if (CurrentO2 != 0)
		ReqAir	= ReqO2 / CurrentO2;

	// Требуемый объем N2 в литрах
	float TargetVN2 = Volume - ReqAir;

	//if (CurrentO2 != 0) TargetVN2 = Volume * (1 - TargetO2 / CurrentO2);

	if (TargetVN2 < 0.0f)
		TargetVN2 = 0.0f;

	double OneSecondOut = VOut(S_N2 * Valve1 / 100.0f, CurrentPress1, Adiobata_N2) * 1000.0f; //Выход в кубометрах/с * 1000 = литры/с
	duty_ms = (uint32_t)(TargetVN2 / (float)OneSecondOut * (float)KoefN2 / 100.0f * 1000.0f); // литры / литры/с * Кклапана/100% = с * 1000 = мс

	if (duty_ms < 0)
		duty_ms = 0;

	Plan[PlanIndex].Value = TargetVN2;
	Plan[PlanIndex].TimeMs = duty_ms;
	Plan[PlanIndex].GasType = GasType_N2;
	return duty_ms;
}

static int CalcCO2(int PlanIndex)
{
	uint32_t duty_ms = 0;
	float DeltaCO2 = (TargetCO2 - CurrentCO2) / 100.0f; //100%
	float TargetVCO2 = Volume * DeltaCO2;

	if (TargetVCO2 < 0.0f)
		TargetVCO2 = 0.0f;

	double OneSecondOut = VOut(S_CO2 * Valve2 / 100.0f, CurrentPress2, Adiobata_CO2) * 1000.0f;	//Выход в кубометрах/с * 1000 =  литр/с
	duty_ms = (uint32_t)(TargetVCO2 / OneSecondOut * KoefCO2 / 100.0f * 1000.0f); 		//литры / литры/секунду * Кклапана/100% * 1000 = мс

	if (duty_ms < 0)
		duty_ms = 0;

	Plan[PlanIndex].Value = TargetVCO2;
	Plan[PlanIndex].TimeMs = duty_ms;
	Plan[PlanIndex].GasType = GasType_O2;

	return duty_ms;
}

static void GasPause(int PlanIndex)
{
	Plan[PlanIndex].GasType = GasType_None;
	Plan[PlanIndex].Value = 0.0f;
	Plan[PlanIndex].TimeMs = MIX_Time * 1000;
}



void InitRelays(TIM_HandleTypeDef *hpwmO2, TIM_HandleTypeDef *hpwmN2) {
	htim1 = hpwmN2;
	htim2 = hpwmO2;
}

void OpenN2(void)
{
	HAL_TIM_PWM_Start(htim1, TIM_CHANNEL_1);
	OperationState |= SOST_OpenN2;
}

void OpenCO2(void)
{
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
	OperationState |= SOST_OpenCO2;
}

void CloseN2(void)
{
	HAL_TIM_PWM_Stop(htim1, TIM_CHANNEL_1);
	OperationState &= ~SOST_OpenN2;
}

void CloseCO2(void)
{
	HAL_TIM_PWM_Stop(htim2, TIM_CHANNEL_1);
	OperationState &= ~SOST_OpenCO2;
}

void UpdateMesure(void)
{
	CurrentPress1 = GetPressure1(ADC_Data[0]);
	CurrentPress2 = GetPressure2(ADC_Data[1]);
	CurrentO2 = GetO2(ADC_Data[2]);
	CurrentCO2 = GetCO2(ADC_Data[3]);
}

uint8_t CreateSchedule(void)
{
	// Задание 0
	CurrentPlan = 0;
	if ((CurrentO2 - TargetO2) > Krit_O2) {
		CalcN2(CurrentPlan);
	} else
		GasPause(CurrentPlan);

	// Задание 1
	CurrentPlan++;
	GasPause(CurrentPlan);

	// Задание 2
	CurrentPlan++;
	if ((TargetCO2 - CurrentCO2) > Krit_CO2) {
		CalcCO2(CurrentPlan);
	} else
		GasPause(CurrentPlan);

	// Задание 3
	CurrentPlan++;
	GasPause(CurrentPlan);

	// Количество шагов, +1 - т.к счет с 0
	PlanSize = CurrentPlan + 1;
	CurrentPlan = 0;

	return SOST_PlanReady;
}

void ProcSchedule() {
	// Если выполнение не начато - начать
	if (!(OperationState & SOST_PlanStarted))	{
		OperationState |= SOST_PlanStarted;

		//30-10-23
		if (Plan[CurrentPlan].GasType == GasType_N2)
			OpenN2();
		else if (Plan[CurrentPlan].GasType == GasType_O2)
			OpenCO2();
	}

	//Проверяем концентрацию в ходе исполнения
	if ((Plan[CurrentPlan].GasType == GasType_N2) && (fabs(TargetO2 - CurrentO2) < (Krit_O2 / 2.0f)) && (OperationState & SOST_PlanStarted)) { //30-10-23
		CloseN2();
		Ms10Counter = 0;
		OperationState &= ~SOST_PlanStarted;
		CurrentPlan++;

		if (CurrentPlan >= PlanSize)
			OperationState |= CreateSchedule();
	}


	if ((Plan[CurrentPlan].GasType == GasType_O2) && (fabs(TargetCO2 - CurrentCO2) < (Krit_CO2 / 2.0f)) && (OperationState & SOST_PlanStarted)) {
		//30-10-23
		CloseCO2();
		Ms10Counter = 0;
		OperationState &= ~SOST_PlanStarted;
		CurrentPlan++;

		if (CurrentPlan >= PlanSize)
			OperationState |= CreateSchedule();
	}

	//отрабатываем шаги плана по времени
	if (((Ms10Counter * 10) > Plan[CurrentPlan].TimeMs) && (OperationState & SOST_PlanStarted)) { //30-10-23
		//кончаем текущий шаг
		if (Plan[CurrentPlan].GasType == GasType_N2) CloseN2();
		if (Plan[CurrentPlan].GasType == GasType_O2) CloseCO2();
		CurrentPlan++;

		OperationState &= ~SOST_PlanStarted;
		Ms10Counter = 0;

		if (CurrentPlan < PlanSize) {
			//начинаем следующий
			if (Plan[CurrentPlan].GasType == GasType_N2) OpenN2();
			if (Plan[CurrentPlan].GasType == GasType_O2) OpenCO2();
			OperationState |= SOST_PlanStarted;  //30-10-23
		} else {
			//Пересоздаем план
			OperationState |= CreateSchedule();
			//Sost &= ~(SOST_PlanReady | SOST_PlanStarted);
			OperationState &= ~SOST_PlanStarted;	//30-10-23
		}
	}
}

