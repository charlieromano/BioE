#ifndef STATEMACHINE_UART_H
#define STATEMACHINE_UART_H

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
	STATE_UART_IDLE,
	STATE_UART_RECEIVE,
    STATE_UART_PROCESS,
    STATE_UART_TRANSMIT,
    STATE_UART_ERROR
} eSystemState_UART;

typedef enum{
	evUART_Idle,
    evUART_Receive,
    evUART_Process,
    evUART_Transmit,
    evUART_Error,
    evUART_Timeout,
} eSystemEvent_UART;

typedef eSystemState_UART (*pfEventHandler_UART)(void);

typedef struct{
	eSystemState_UART 		fsmState;
	eSystemEvent_UART 		fsmEvent;
	pfEventHandler_UART		fsmHandler;
} sStateMachine_UART;

eSystemState_UART 	UART_InitHandler(void);
eSystemState_UART 	UART_ReceiveHandler(void);
eSystemState_UART 	UART_ProcessHandler(void);
eSystemState_UART 	UART_TransmitHandler(void);


#endif /* STATEMACHINE_UART_H_ */