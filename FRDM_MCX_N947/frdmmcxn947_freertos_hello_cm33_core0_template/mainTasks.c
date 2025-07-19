#include "mainTasks.h"

TimerHandle_t timerHandleAB;
QueueHandle_t queueHandle_AB;


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

      
void vTaskAB(void *xTimerHandle)
{
    (void)xTimerHandle;
    PRINTF("vTaskAB!\r\n");
 
    /*
    if (pdTRUE == xSemaphoreTake( xMutexUART, portMAX_DELAY)){
       vPrintString("Task AB is running.\r\n");
       xSemaphoreGive(xMutexUART);
    }
   */ 
 
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
