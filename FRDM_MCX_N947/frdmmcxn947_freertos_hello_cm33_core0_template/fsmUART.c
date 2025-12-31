//fsmUART.c
#include "fsmUART.h"
#include "uart_driver.h"

LPUART_Type *base = LPUART4;
uint8_t ch;

bool fsmUART_timerFlag = false;

eSystemState_fsmUART 	fsmUART_InitHandler(void)
{
    PRINTF("fsmUART: Init.\r\n");
    fsmUART_timerFlag = true;
    return STATE_UART_IDLE;
}

eSystemState_fsmUART 	fsmUART_IdleHandler(void)
{
    PRINTF("fsmUART: IDLE\n");
    //UART_DriverInit();
    fsmUART_timerFlag = false;
    return STATE_UART_IDLE;
}

eSystemState_fsmUART 	fsmUART_ReceiveHandler(void)
{  
    PRINTF("fsmUART: Receiving data...\n");
    // If data received successfully, return STATE_UART_PROCESS
    // If an error occurs, return STATE_UART_ERROR
    
    fsmUART_timerFlag = true;
    return STATE_UART_TRANSMIT;
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
    PRINTF("fsmUART: Transmit\n");
    // If transmission is successful, return STATE_UART_IDLE
    // If an error occurs, return STATE_UART_ERROR
 
    while (txIndex != rxIndex)
    {
        LPUART_WriteByte(LPUART4, demoRingBuffer[txIndex]);
        txIndex = (txIndex + 1) % LPUART_RING_BUFFER_SIZE;
    }

    fsmUART_timerFlag = false;

    return STATE_UART_IDLE;
}

sStateMachine_fsmUART fsmUART[] = 
{
	{STATE_UART_INIT, evUART_Timeout, fsmUART_InitHandler},
	{STATE_UART_IDLE, evUART_Timeout, fsmUART_IdleHandler},
    {STATE_UART_IDLE, evUART_Timeout, fsmUART_ReceiveHandler},
    {STATE_UART_RECEIVE, evUART_Timeout, fsmUART_TransmitHandler},
    {STATE_UART_TRANSMIT, evUART_Timeout, fsmUART_IdleHandler}
};