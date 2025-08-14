//statemachine_UART.c
#include "statemachine_UART.h"

eSystemState_UART 	UART_InitHandler(void)
{
    PRINTF("UART State Machine Init...\n");
    // Initialize UART hardware and state
    return STATE_UART_IDLE;
}

eSystemState_UART 	UART_ReceiveHandler(void)
{
    PRINTF("Receiving data...\n");
    // Implement UART data reception logic
    // If data received successfully, return STATE_UART_PROCESS
    // If an error occurs, return STATE_UART_ERROR
    return STATE_UART_RECEIVE;
}

eSystemState_UART 	UART_ProcessHandler(void)
{
    PRINTF("Processing received data...\n");
    // Implement data processing logic
    // If processing is successful, return STATE_UART_TRANSMIT
    // If an error occurs, return STATE_UART_ERROR
    return STATE_UART_PROCESS;
}

eSystemState_UART 	UART_TransmitHandler(void)
{
    PRINTF("Transmitting data...\n");
    // Implement UART data transmission logic
    // If transmission is successful, return STATE_UART_IDLE
    // If an error occurs, return STATE_UART_ERROR
    return STATE_UART_TRANSMIT;
}

sStateMachine_UART fsmMachineUART [] = 
{
	{STATE_UART_IDLE, evUART_Idle, UART_InitHandler},
	{STATE_UART_RECEIVE, evUART_Receive, UART_ReceiveHandler},
	{STATE_UART_PROCESS, evUART_Process, UART_ProcessHandler},
    {STATE_UART_TRANSMIT, evUART_Transmit, UART_TransmitHandler},
    {STATE_UART_ERROR, evUART_Error, UART_InitHandler} // Error state handler can be NULL or a specific error handler
};