#ifndef MAIN_TASKS_H_
#define MAIN_TASKS_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "fsl_pwm.h"
#include "pin_mux.h"

#include "statemachine_AB.h"
#include "statemachine_Valve.h"
#include "statemachine_UART.h"

void timerCallbackAB(TimerHandle_t xTimerHandle);
void vTaskAB(void *xTimerHandle);
void vTaskUART(void *pvParameters);
void vTaskValve(void *pvParameters);
void vTaskPump(void *pvParameters);


extern TimerHandle_t timerHandle_AB;
extern QueueHandle_t queueHandle_AB;

extern sStateMachine_AB fsmMachineAB[];

#endif /* MAIN_TASKS_H_ */
