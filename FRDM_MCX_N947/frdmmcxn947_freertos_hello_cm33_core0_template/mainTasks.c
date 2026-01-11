// mainTasks.c
#include "mainTasks.h"
#include "uart_driver.h"
#include "fsmUART.h"

TimerHandle_t timerHandle_AB;
TimerHandle_t timerHandle_fsmUART;
QueueHandle_t queueHandle_AB;
QueueHandle_t queueHandle_fsmUART = NULL;
TaskHandle_t xTaskStateMachineHandler_UART = NULL;
TaskHandle_t xTaskStateMachineHandler_fsmUART = NULL;
TaskHandle_t xTaskStateMachineHandler_AB = NULL;

void vTaskCreate_UART(void)
{
    //Create UART Task and related resources (queue, timer)
    PRINTF("createTaskUART().\r\n");

    queueHandle_fsmUART = xQueueCreate(QUEUE_MAX_LENGTH, sizeof(eSystemEvent_fsmUART));
    if (queueHandle_fsmUART == NULL){
        perror("Error creating FSM queue");
        while(1);
    }

    // Create the timer 
    if( (timerHandle_fsmUART = xTimerCreate( "Timer fsmUART 300", 300, true, NULL, 
        timerCallback_fsmUART)) == NULL ) {
            perror("Error creating timer");
            while(1);
    }

    // Start the timer
    if(xTimerStart(timerHandle_fsmUART, 0) != pdPASS){
        perror("Error starting timer");
        while(1);
    }

    /* Create UART/fsmUART task */
    if (xTaskCreate(vTask_fsmUART, "UART_Task",
                    configMINIMAL_STACK_SIZE + 100,
                    NULL, tskIDLE_PRIORITY + 3,
                    &xTaskStateMachineHandler_UART) != pdPASS){
        perror("Error creating UART task");
        while(1);
    }
}

void vTaskUART(void *pvParameters)
{
    uint16_t tmprxIndex;
    uint16_t tmptxIndex;
    PRINTF("vTaskUART().\r\n");

    while(1)
    {
        if (kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(LPUART4))
        {
            tmprxIndex = rxIndex;
            tmptxIndex = txIndex;
            
            if (tmprxIndex != tmptxIndex)
            // buffer is not empty, there is data to send.
            {
                LPUART_WriteByte(LPUART4, demoRingBuffer[txIndex]);
                txIndex++;
                txIndex %= LPUART_RING_BUFFER_SIZE;
            }
        }

        //vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void vTask_fsmUART(void *xTimerHandle)
{
    // This task is responsible for UART communication
    (void)xTimerHandle;
    PRINTF("vTask_fsmUART!\r\n");

    while(1){
        
        UART_DriverInit();

        // fsmUART init
        eSystemEvent_fsmUART newEvent = evUART_Init;
        eSystemState_fsmUART nextState = STATE_UART_INIT;
        fsmUART[nextState].fsmEvent = newEvent; 
        nextState = (*fsmUART[nextState].fsmHandler)();
        
        // Active object
        while(1){
            if( pdPASS == xQueueReceive(queueHandle_fsmUART, &newEvent, portMAX_DELAY)){
                fsmUART[nextState].fsmEvent = newEvent;
                nextState = (*fsmUART[nextState].fsmHandler)();
            }
        }
        //vPrintString("This task is running and about to delete itself.\r\n");
        //vTaskDelete(xTaskStateMachineHandler);
    }
}

void timerCallback_fsmUART(TimerHandle_t xTimerHandle)
{
    (void)xTimerHandle;
    PRINTF("fsmUART: Timer!\r\n");

    //if(fsmUART_timerFlag){

        eSystemEvent_fsmUART fsmUART_evemt = evUART_Timeout;

        if(xQueueSend(queueHandle_fsmUART, &fsmUART_evemt, 0U)!=pdPASS){
            perror("Error sending data to the queueHandle_AB from timer\r\n");
        }
    //}
}

void vTaskCreate_AB(void)
{
    //Create AB Task and related resources (queue, timer)
    PRINTF("createTaskAB().\r\n");

    queueHandle_AB = xQueueCreate(QUEUE_MAX_LENGTH, sizeof(eSystemEvent_AB));
    if (queueHandle_AB == NULL){
        perror("Error creating AB queue");
        while(1);
    }

    // Create the timer 
    if( (timerHandle_AB = xTimerCreate( "Timer AB 1000", 1000, true, NULL, 
        timerCallbackAB)) == NULL ) {
            perror("Error creating timer");
            while(1);
    }

    // Start the timer
    if(xTimerStart(timerHandle_AB, 0) != pdPASS){
        perror("Error starting timer");
        while(1);
    }

    /* Create AB task */
    if (xTaskCreate(vTask_fsmAB, "fsmAB_Task",
                    configMINIMAL_STACK_SIZE + 100,
                    NULL, tskIDLE_PRIORITY + 3,
                    &xTaskStateMachineHandler_AB) != pdPASS){
        perror("Error creating AB task");
        while(1);
    }
}

void vTask_fsmAB(void *xTimerHandle)
{
    (void)xTimerHandle;
    PRINTF("vTask fsmAB!\r\n");
  
    while(true){

        // fsmMachineAB init
        eSystemEvent_AB newEvent = evInit_AB;
        eSystemState_AB nextState = STATE_INIT_AB;
        fsmMachineAB[nextState].fsmEvent = newEvent; 
        nextState = (*fsmMachineAB[nextState].fsmHandler)();
        
        // Active object
        while(true){
            if( pdPASS == xQueueReceive(queueHandle_AB, &newEvent, portMAX_DELAY)){
                fsmMachineAB[nextState].fsmEvent = newEvent;
                nextState = (*fsmMachineAB[nextState].fsmHandler)();
            }
        }
       //vPrintString("This task is running and about to delete itself.\r\n");
       //vTaskDelete(xTaskStateMachineHandler);
    }
}

void timerCallbackAB(TimerHandle_t xTimerHandle)
{
    PRINTF("Timer!\r\n");
    static uint8_t cnt = 0;
    cnt++;
    eSystemEvent_AB data_AB = cnt%2;
    if(xQueueSend(queueHandle_AB, &data_AB, 0U)!=pdPASS){
          perror("Error sending data to the queueHandle_AB from timer\r\n");
    }
}
   

#define VALVE_GPIO_PORT     GPIO1 // For P1_12, it's GPIO1
#define VALVE_GPIO_PIN      12U   // Pin 12

void vTaskValve(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(2000); // Toggle every 2 seconds

    xLastWakeTime = xTaskGetTickCount();

    PRINTF("Valve control task started on P1_12.\r\n");
    /*
    if (pdTRUE == xSemaphoreTake( xMutexUART, portMAX_DELAY)){
       vPrintString("Task AB is running.\r\n");
       xSemaphoreGive(xMutexUART);
    }
   */ 
    while (true) {
        GPIO_PortSet(VALVE_GPIO_PORT, (1U << VALVE_GPIO_PIN)); // Set P1_12 high
        PRINTF("Valve ON\r\n");

        if (GPIO_PinRead(VALVE_GPIO_PORT, VALVE_GPIO_PIN)) {
            PRINTF("Valve is currently ON\r\n");
        } else {
            PRINTF("Valve is currently OFF\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// #define PWM_BASEADDR        PWM1 
// #define PWM_OUTPUT_CHANNEL  kPWM_PwmChannel1
// #define PWM_CLOCK_FREQ      CLOCK_GetPwmClkFreq(1U) 
// #define PWM_FREQUENCY_HZ    1000U // 1 kHz
// #define PWM_DUTY_CYCLE_MIN  0U    // 0% duty cycle
// #define PWM_DUTY_CYCLE_MAX  100U  // 100% duty cycle

void vTaskPump(void *pvParameters) {
//     pwm_config_t pwmConfig;           // Corrected: flexpwm_config_t -> pwm_config_t
//     pwm_channel_config_t channelConfig; // Corrected: flexpwm_signal_param_t -> pwm_channel_config_t
//     uint8_t dutyCycle = PWM_DUTY_CYCLE_MIN; // Corrected: uint32_t -> uint8_t
//     bool dutyCycleIncreasing = true;

//     PRINTF("PWM Control Task Started on P2_7 (PWM1_B0).\r\n");

//     // Get default PWM configuration
//     PWM_GetDefaultConfig(&pwmConfig); // Corrected: FLEXPWM_GetDefaultConfig -> PWM_GetDefaultConfig

//     // Set PWM frequency
//     pwmConfig.reloadOption = kPWM_ReloadPwmCounter; // Reload counter at period end
//     pwmConfig.enablePWMInStop = false; // Disable PWM when in stop mode
//     pwmConfig.clockSource = kPWM_PwmClockSourcePwm0Clk; // Often a default, ensure correct clock source
//     pwmConfig.prescale = kPWM_DivBy1; // No prescaler, or adjust as needed for desired resolution/range
//     pwmConfig.periodType = kPWM_EdgeAligned; // Or kPWM_CenterAligned, depending on application
//     pwmConfig.fullCycle = PWM_CLOCK_FREQ / PWM_FREQUENCY_HZ; // Calculate period based on clock and frequency

//     // Initialize the PWM module
//     PWM_Init(PWM_BASEADDR, &pwmConfig); // Corrected: FLEXPWM_Init -> PWM_Init

//     // Configure the PWM channel (P2_7 -> PWM1_B0 -> kPWM_PwmChannel1)
//     channelConfig.level = kPWM_HighTrue; // Output high when active (or kPWM_LowTrue)
//     channelConfig.dutyCycle = PWM_DUTY_CYCLE_MIN; // Initial duty cycle (0-100)
//     channelConfig.enableComplementary = false; // Not a complementary output
//     channelConfig.enableDeadtime = false; // No deadtime
//     channelConfig.deadtimeValue = 0U;

//     // Setup the PWM channel
//     // The `PWM_SetupPwm` takes an array of channel configs
//     PWM_SetupPwm(PWM_BASEADDR, &channelConfig, 1U, PWM_OUTPUT_CHANNEL, PWM_CLOCK_FREQ);

//     // Start the PWM timer
//     PWM_StartTimer(PWM_BASEADDR); // Corrected: FLEXPWM_StartTimer -> PWM_StartTimer

//     // Main loop to change duty cycle
//     while (1) {
//         // Update duty cycle
//         // Corrected: FLEXPWM_SetDutyCycle -> PWM_UpdatePwmDutycycle
//         // CRITICAL: Duty cycle is 0-100 for this driver
//         PWM_UpdatePwmDutycycle(PWM_BASEADDR, PWM_OUTPUT_CHANNEL, dutyCycle);

//         // For this driver, the update is often implicit or handled by `PWM_UpdatePwmDutycycle`
//         // or a periodic reload from the `pwm_config_t.reloadOption`
//         // No direct equivalent of FLEXPWM_UpdatePwm is typically needed for simple updates.

//         PRINTF("PWM Duty Cycle: %d%%\r\n", dutyCycle);

//         // Change duty cycle for next iteration
//         if (dutyCycleIncreasing) {
//             dutyCycle += 5; // Increase by 5%
//             if (dutyCycle >= PWM_DUTY_CYCLE_MAX) {
//                 dutyCycle = PWM_DUTY_CYCLE_MAX;
//                 dutyCycleIncreasing = false;
//             }
//         } else {
//             dutyCycle -= 5; // Decrease by 5%
//             if (dutyCycle <= PWM_DUTY_CYCLE_MIN) {
//                 dutyCycle = PWM_DUTY_CYCLE_MIN;
//                 dutyCycleIncreasing = true;
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(500)); // Wait 500ms before changing duty cycle again
//     }
}

/*
#include "fsl_lpuart.h"

#define DEMO_UART2          LPUART2
#define DEMO_UART2_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)  // Adjust depending on your SDK clock functions
#define DEMO_UART2_IRQn     LPUART2_IRQn

void uart2_init(void)
{
    lpuart_config_t config;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(DEMO_UART2, &config, DEMO_UART2_CLK_FREQ);
}

void uart2_echo_task(void *pvParameters)
{
    uint8_t ch;

    for (;;)
    {
        // Blocking read of 1 byte
        LPUART_ReadBlocking(DEMO_UART2, &ch, 1);

        // Echo back
        LPUART_WriteBlocking(DEMO_UART2, &ch, 1);
    }
}
*/