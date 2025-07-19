//statemachine_AB.c
#include "statemachine_Valve.h"

eSystemState_Valve 	InitHandler_Valve(void){
	PRINTF("State Machine Init: Valve...\n");
	return STATE_VALVE_OFF;
}

eSystemState_Valve 	Valve_Handler_OnOff(void){
	return STATE_VALVE_OFF;
}
eSystemState_Valve 	Valve_Handler_OffOn(void){
	return STATE_VALVE_ON;
}

eSystemState_Valve 	Valve_Handler_OffOff(void){
	return STATE_VALVE_OFF;
}

eSystemState_Valve 	Valve_Handler_OnOn(void){
	return STATE_VALVE_ON;
}


sStateMachine_Valve fsmMachineValve [] = 
{
	{STATE_INIT_VALVE, evInit_Valve, InitHandler_Valve},
	{STATE_VALVE_ON, evValve_turnON, Valve_Handler_OffOn},
	{STATE_VALVE_OFF, evValve_turnOFF, Valve_Handler_OnOff},
	{STATE_VALVE_ON, evValve_turnON, Valve_Handler_OnOn},
	{STATE_VALVE_OFF, evValve_turnOFF, Valve_Handler_OffOff}
};