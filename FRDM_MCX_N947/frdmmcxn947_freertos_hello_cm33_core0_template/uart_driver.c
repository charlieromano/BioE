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
    uint16_t tmprxIndex = rxIndex;
    uint16_t tmptxIndex = txIndex;
    //eSystemEvent_fsmUART fsmUART_event = evUART_Receive;
    
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(LPUART4))
    {
    
        data = LPUART_ReadByte(LPUART4);

        uint16_t next = (tmprxIndex + 1) % LPUART_RING_BUFFER_SIZE;
        if (next == tmptxIndex) {
            // buffer full -> drop oldest by advancing txIndex
            txIndex = (txIndex + 1) % LPUART_RING_BUFFER_SIZE;
        }
        demoRingBuffer[tmprxIndex] = data;
        rxIndex = next;
        UART_WriteByteFromISR(data); // Echo back
        // If ring buffer is not full, add data to ring buffer. 
/*         if (((tmprxIndex + 1) % LPUART_RING_BUFFER_SIZE) != tmptxIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= LPUART_RING_BUFFER_SIZE;
            UART_WriteByteFromISR(data); // Echo back
            // Send event to FSM queue
           if (queueHandle_fsmUART != NULL){
                xQueueSendFromISR(queueHandle_fsmUART, &fsmUART_event, &xHigherPriorityTaskWoken);
            }
        }
*/    }
    /*
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        uint8_t data = LPUART_ReadByte(LPUART4);

        // store into ring buffer if space available 
        uint16_t next = (rxIndex + 1) % LPUART_RING_BUFFER_SIZE;
        if (next != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex = next;
        }
       eSystemEvent_fsmUART ev = evUART_Receive;
        if (queueHandle_fsmUART != NULL){
            xQueueSendFromISR(queueHandle_fsmUART, &ev, &xHigherPriorityTaskWoken);
        }
    }
    */
    // Clear interrupt flag. 
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    SDK_ISR_EXIT_BARRIER;
}
/*
void LPUART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t ch;
    // Check if RX data is available
    if (LPUART_GetStatusFlags(LPUART1) & kLPUART_RxDataRegFullFlag)    {
        ch = LPUART_ReadByte(LPUART1);
        // Send event or data to a queue
        xQueueSendFromISR(queueHandle_fsmUART, &ch, &xHigherPriorityTaskWoken);
        // Optionally, echo back
        LPUART_WriteByte(LPUART1, ch);
    }
    // Clear interrupt flag if needed (usually handled by reading the byte)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
*/
/* Optional: Byte access from ISR-safe interface */
void UART_WriteByteFromISR(uint8_t data)
{
    LPUART_WriteByte(LPUART4, data);
}