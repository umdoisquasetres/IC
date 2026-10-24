#ifndef UART_H
#define UART_H

#include <stdint.h>

void UART_Init(uint32_t baud_rate);
void UART_Write(uint8_t data);
void UART_WriteString(const char *str);
void UART_WaitTXComplete(void);
uint8_t UART_DataReady(void);
uint8_t UART_Read(void);
void UART_ReceiveISR(void);
void UART_TransmitBegin(void);
void UART_TransmitEnd(void);

#endif
