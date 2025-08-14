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
    evUART_Init,
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
    char message[64];
} sUART_Error;

#endif /* STATEMACHINE_UART_H_ */