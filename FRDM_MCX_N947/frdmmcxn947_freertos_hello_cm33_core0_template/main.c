/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app.h"

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
static void hello_task(void *pvParameters);
TaskHandle_t helloTaskHandle = NULL;
TaskHandle_t xTaskStateMachineHandler_AB = NULL;


int main(void)
{
    BOARD_InitHardware();
/*
    if (xTaskCreate(hello_task, "Hello_task", 
      configMINIMAL_STACK_SIZE + 100,
      NULL, tskIDLE_PRIORITY + 3,
      &xTaskStateMachineHandler_UART) != pdPASS){
        perror("Error creating UART task");
        while (1);
    }
*/
   //***************************************************************************// 
   // State Machine AB tasks 
   //***************************************************************************
    /*
   // Create the timer 
   if( (timerHandle_AB = xTimerCreate( "TimerAB", 1000, true, NULL, 
      timerCallbackAB)) == NULL ) {
         perror("Error creating timer");
         return 1;
   }

   // Start the timer
   if(xTimerStart(timerHandle_AB, 0) != pdPASS){
      perror("Error starting timer");
      return 1;
      }

   // Create the queue
   queueHandle_AB = xQueueCreate(QUEUE_MAX_LENGTH, sizeof(eSystemEvent_AB));
   if (queueHandle_AB == NULL){
      perror("Error creating queue");
      return 1;
   }

   // Create the task 
   if( xTaskCreate( vTaskAB, "State Machine using active object", 
      configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY+2, 
      &xTaskStateMachineHandler_AB) 
      == pdFAIL ) {
      perror("Error creating task");
      return 1;
   }
    */
   //***************************************************************************//
   // Create the queue
   createTaskUART();
   //***************************************************************************//

   vTaskStartScheduler();
   while (1);
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters)
{
    while (1)
    {
        PRINTF("Hello world 46.\r\n");
        vTaskSuspend(NULL);
    }
}
