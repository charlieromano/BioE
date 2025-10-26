//statemachine_UART.c
#include "statemachine_UART.h"

LPUART_Type *base = LPUART1;
uint8_t ch;

eSystemState_fsmUART 	fsmUART_InitHandler(void)
{
    PRINTF("UART State Machine Init...\n");
    // Initialize UART hardware and state

    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;
    LPUART_Init(base, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    LPUART_EnableInterrupts(base, kLPUART_RxDataRegFullInterruptEnable | kLPUART_TxDataRegEmptyInterruptEnable);
    LPUART_EnableTx(base, true);
    LPUART_EnableRx(base, true);

    PRINTF("UART State Machine started.\r\n");

    return STATE_UART_IDLE;
}

eSystemState_fsmUART 	fsmUART_IdleHandler(void)
{
    PRINTF("UART State Machine IDLE\n");
    // Wait for an event to trigger state transition
    // For demonstration, we'll just echo received characters

    if (kStatus_Success == LPUART_ReadBlocking(base, &ch, 1)){
        // Blocking write: Echo the character back
        //LPUART_WriteBlocking(base, &ch, 1);
        PRINTF("vTaskUART!\r\n");
    }

    return STATE_UART_IDLE;
}

eSystemState_fsmUART 	fsmUART_ReceiveHandler(void)
{
    PRINTF("Receiving data...\n");
    // Implement UART data reception logic
    // If data received successfully, return STATE_UART_PROCESS
    // If an error occurs, return STATE_UART_ERROR
    return STATE_UART_IDLE;
}

eSystemState_fsmUART 	fsmUART_ProcessHandler(void)
{
    PRINTF("Processing received data...\n");
    // Implement data processing logic
    // If processing is successful, return STATE_UART_TRANSMIT
    // If an error occurs, return STATE_UART_ERROR
    return STATE_UART_IDLE;
}

eSystemState_fsmUART 	fsmUART_TransmitHandler(void)
{
    PRINTF("Transmitting data...\n");
    // Implement UART data transmission logic
    // If transmission is successful, return STATE_UART_IDLE
    // If an error occurs, return STATE_UART_ERROR
    return STATE_UART_IDLE;
}

sStateMachine_fsmUART fsmUART [] = 
{
	{STATE_UART_INIT, evUART_Init, fsmUART_InitHandler},
	{STATE_UART_IDLE, evUART_Idle, fsmUART_IdleHandler},
//    {STATE_UART_RECEIVE, evUART_Receive, fsmUART_ReceiveHandler},
//	{STATE_UART_PROCESS, evUART_Process, fsmUART_ProcessHandler},
//    {STATE_UART_TRANSMIT, evUART_Transmit, fsmUART_TransmitHandler},
//    {STATE_UART_ERROR, evUART_Error, fsmUART_IdleHandler} // Error state handler can be NULL or a specific error handler
};