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
