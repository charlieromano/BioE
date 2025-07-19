#ifndef STATEMACHINE_VALVE_H
#define STATEMACHINE_VALVE_H

#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"
#include "fsl_debug_console.h"

#define QUEUE_MAX_LENGTH 10

extern SemaphoreHandle_t xMutexUART;

typedef enum {
	STATE_INIT_VALVE,
	STATE_VALVE_OFF,
	STATE_VALVE_ON
} eSystemState_Valve;

typedef enum{
	evInit_Valve,
	evValve_turnON,
	evValve_turnOFF
} eSystemEvent_Valve;

typedef eSystemState_Valve (*pfEventHandler_Valve)(void);

typedef struct{
	eSystemState_Valve 		fsmState;
	eSystemEvent_Valve 		fsmEvent;
	pfEventHandler_Valve	fsmHandler;
} sStateMachine_Valve;

eSystemState_Valve 	InitHandler_Valve(void);
eSystemState_Valve 	Valve_Handler_OnOff(void);
eSystemState_Valve 	Valve_Handler_OffOn(void);
eSystemState_Valve 	Valve_Handler_OffOff(void);
eSystemState_Valve 	Valve_Handler_OnOn(void);


#endif /* STATEMACHINE_VALVE_H_ */