// uart_driver.c
#include "uart_driver.h"
#include "fsmUART.h" 

/* UART ring buffer */
uint8_t demoRingBuffer[LPUART_RING_BUFFER_SIZE];
volatile uint16_t txIndex = 0;
volatile uint16_t rxIndex = 0;

/* Initialization */
void UART_DriverInit(void)
{
    PRINTF("UART_driverInit(2026).\r\n");
    lpuart_config_t config;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = LPUART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(LPUART4, &config, LPUART_CLK_FREQ);
    //const uint8_t welcomeMsg[] = "Lpuart API welcome message.\r\nNow please input:\r\n";
    //LPUART_WriteBlocking(LPUART4, welcomeMsg, sizeof(welcomeMsg));
    LPUART_EnableInterrupts(LPUART4, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LPUART_IRQn);
}

/* ISR Handler */
void LPUART_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    uint8_t data;
    //uint16_t tmprxIndex = rxIndex;
    //uint16_t tmptxIndex = txIndex;
    //eSystemEvent_fsmUART fsmUART_event = evUART_Receive;
    
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(LPUART4))
    {
        UART_ReadByteFromISR(&data, rxIndex);
        UART_WriteByteFromISR(data, txIndex); // Echo back     
    }
    // Clear interrupt flag. 
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    SDK_ISR_EXIT_BARRIER;
}

void UART_WriteByteFromISR(uint8_t data, uint16_t txIndex)
{
    LPUART_WriteByte(LPUART4, data);
}

BaseType_t UART_ReadByteFromISR(uint8_t *data, uint16_t rxIndex)
{
    *data = LPUART_ReadByte(LPUART4);
    BaseType_t status = pdFALSE;
    uint16_t tmptxIndex = txIndex;
    if (rxIndex != tmptxIndex)
    {
        *data = demoRingBuffer[tmptxIndex];
        txIndex = (txIndex + 1) % LPUART_RING_BUFFER_SIZE;
        status = pdTRUE;
    }
    return status;
}

// End of uart_driver.c