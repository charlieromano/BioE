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
TaskHandle_t xTaskStateMachineHandler_Valve = NULL;
TaskHandle_t xTaskStateMachineHandler_Pump = NULL;
TaskHandle_t xTaskStateMachineHandler_UART = NULL;
TaskHandle_t xTaskStateMachineHandler_fsmUART = NULL;

/*!
 * @brief Application entry point.
 */

#define LPUART_RING_BUFFER_SIZE 16
uint8_t g_tipString[] = "Lpuart API welcome message.\r\nNow please input:\r\n";
uint8_t demoRingBuffer[LPUART_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

void LPUART_IRQHandler(void)
{
    uint8_t data;
    uint16_t tmprxIndex = rxIndex;
    uint16_t tmptxIndex = txIndex;

    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(LPUART4))
    {
        data = LPUART_ReadByte(LPUART4);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((tmprxIndex + 1) % LPUART_RING_BUFFER_SIZE) != tmptxIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= LPUART_RING_BUFFER_SIZE;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

int main(void)
{
    /* Init board hardware. */
    lpuart_config_t config;
    uint16_t tmprxIndex = rxIndex;
    uint16_t tmptxIndex = txIndex;

    BOARD_InitHardware();

    if (xTaskCreate(hello_task, "Hello_task", 
        configMINIMAL_STACK_SIZE + 100, NULL, 
        hello_task_PRIORITY, NULL) != pdPASS){
        PRINTF("Task creation failed!.\r\n");
        while (1);
    }
   
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;
    LPUART_Init(LPUART4, &config, LPUART_CLK_FREQ);
    LPUART_WriteBlocking(LPUART4, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
    LPUART_EnableInterrupts(LPUART4, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LPUART_IRQn);

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
   fsmUART_queueHandle = xQueueCreate(QUEUE_MAX_LENGTH, sizeof(eSystemEvent_fsmUART));
   if (fsmUART_queueHandle == NULL){
      perror("Error creating queue");
      return 1;
   }
   // Create the task 
   if( xTaskCreate( vTaskUART, "State Machine UART using active object", 
      configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY+3, 
      &xTaskStateMachineHandler_UART) 
      == pdFAIL ) {
      perror("Error creating task");
      return 1;
   }
   //***************************************************************************//
    //uart2_init();

    /*
    xTaskCreate(uart2_echo_task, "UART2_ECHO", 
        configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 2, 
        NULL);

    */
   //vTaskStartScheduler();
   while (1){
      while (kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(LPUART4))
      {
         tmprxIndex = rxIndex;
         tmptxIndex = txIndex;
         if (tmprxIndex != tmptxIndex)
         {
               LPUART_WriteByte(LPUART4, demoRingBuffer[txIndex]);
               txIndex++;
               txIndex %= LPUART_RING_BUFFER_SIZE;
         }
      }
   }
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters)
{
    for (;;)
    {
        PRINTF("Hello world 45.\r\n");
        vTaskSuspend(NULL);
    }
}
