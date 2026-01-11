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

int main(void)
{
    BOARD_InitHardware();

    if (xTaskCreate(hello_task, "Hello_task", 
      configMINIMAL_STACK_SIZE + 100,
      NULL, tskIDLE_PRIORITY + 3,
      &xTaskStateMachineHandler_UART) != pdPASS){
        perror("Error creating UART task");
        while (1);
    }

   //vTaskCreate_AB(); 
   vTaskCreate_UART();
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
        PRINTF("Hello world 47.\r\n");
        vTaskSuspend(NULL);
    }
}
