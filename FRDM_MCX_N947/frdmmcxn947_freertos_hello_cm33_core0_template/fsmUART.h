#ifndef FSM_UART_H
#define FSM_UART_H

#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "pin_mux.h"

#define QUEUE_MAX_LENGTH 10
#define UART_MAX_COMMAND_LENGTH 64

extern SemaphoreHandle_t xMutexUART;

typedef enum {
    STATE_UART_INIT,
	STATE_UART_IDLE,
	STATE_UART_RECEIVE,
    STATE_UART_PROCESS,
    STATE_UART_TRANSMIT,
    STATE_UART_ERROR
} eSystemState_fsmUART;

typedef enum{
    evUART_Init,
	evUART_Idle,
    evUART_Receive,
    evUART_Process,
    evUART_Transmit,
    evUART_Error,
    evUART_Timeout,
} eSystemEvent_fsmUART;

typedef eSystemState_fsmUART (*pfEventHandler_fsmUART)(void);

typedef struct sStateMachine_fsmUART{
	eSystemState_fsmUART 		fsmState;
	eSystemEvent_fsmUART 		fsmEvent;
	pfEventHandler_fsmUART		fsmHandler;
}sStateMachine_fsmUART;

/* fsmUART[] structure array definition is given in fsmUART.c*/
extern sStateMachine_fsmUART fsmUART[]; 

eSystemState_fsmUART 	fsmUART_InitHandler(void);
eSystemState_fsmUART 	fsmUART_ReceiveHandler(void);
eSystemState_fsmUART 	fsmUART_ProcessHandler(void);
eSystemState_fsmUART 	fsmUART_TransmitHandler(void);

typedef enum{
    ERROR_UART_NONE,
    ERROR_UART_TIMEOUT,
    ERROR_UART_OVERFLOW,
    ERROR_UART_PARITY,
    ERROR_UART_FRAMING,
    ERROR_UART_UNKNOWN_COMMAND,
    ERROR_UART_INVALID_PARAMETER
} eUART_ErrorCode;

typedef struct {
    eUART_ErrorCode code;
    char message[UART_MAX_COMMAND_LENGTH];
} sUART_Error;

#endif /* STATEMACHINE_UART_H_ */