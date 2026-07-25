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
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "fsl_pwm.h"
#include "fsl_gpio.h"

#include "uart_driver.h"
#include "fsmUART.h"
#include "statemachine_AB.h"
#include "statemachine_Valve.h"

extern TaskHandle_t xTaskStateMachineHandler_UART;
extern TaskHandle_t xTaskStateMachineHandler_fsmUART;
extern TaskHandle_t xTaskStateMachineHandler_AB;

extern TimerHandle_t timerHandle_AB;
extern TimerHandle_t timerHandle_fsmUART;

extern QueueHandle_t queueHandle_AB;
extern QueueHandle_t queueHandle_fsmUART;

void vTaskCreate_UART(void);
void vTaskCreate_AB(void);

void vTaskUART(void *pvParameters);
void vTask_fsmAB(void *xTimerHandle);
void vTask_fsmUART(void *xTimerHandle);

void timerCallback_AB(TimerHandle_t xTimerHandle);
void timerCallback_fsmUART(TimerHandle_t xTimerHandle);

void vTaskValve(void *pvParameters);
void vTaskPump(void *pvParameters);

#endif /* MAIN_TASKS_H_ */
