#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "fsl_lpuart.h"
#include "FreeRTOS.h"
#include "queue.h"

/* UART configuration macros */
#define LPUART_RING_BUFFER_SIZE 64
#define LPUART_CLK_FREQ   CLOCK_GetLPFlexCommClkFreq(4u)
#define LPUART_IRQn       LP_FLEXCOMM4_IRQn
#define LPUART_IRQHandler LP_FLEXCOMM4_IRQHandler
#define LPUART_BAUDRATE     115200U //BOARD_DEBUG_UART_BAUDRATE

extern QueueHandle_t fsmUART_queueHandle;
extern uint8_t g_tipString[];
extern uint8_t demoRingBuffer[LPUART_RING_BUFFER_SIZE];
extern volatile uint16_t txIndex;
extern volatile uint16_t rxIndex;
extern QueueHandle_t queueHandle_fsmUART;
extern bool fsmUART_timerFlag;

void UART_DriverInit(void);
BaseType_t UART_ReadByteFromISR(uint8_t *data, uint16_t rxIndex);
void UART_WriteByteFromISR(uint8_t data, uint16_t txIndex);

#endif /* UART_DRIVER_H_ */
