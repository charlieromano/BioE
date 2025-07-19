//statemachine_AB.c
#include "statemachine_AB.h"

eSystemState_AB 	InitHandler_AB(void){ 
	PRINTF("State Machine Init...\n");
	return STATE_A; 
}

eSystemState_AB 	AtoBHandler(void){
	PRINTF("STATE_A;\n");
	return STATE_B; 
}
eSystemState_AB 	BtoAHandler(void){ 
	PRINTF("STATE_B;\n");
	return STATE_A; 
}

sStateMachine_AB fsmMachineAB [] = 
{
	{STATE_INIT_AB, evInit_AB, InitHandler_AB},
	{STATE_A, evTimeout_A, AtoBHandler},
	{STATE_B, evTimeout_B, BtoAHandler}
};