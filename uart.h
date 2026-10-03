#ifndef UART_H
#define UART_H

#include <stdint.h>

// Inicializa a UART com o baud rate especificado (ex: 9600)
void UART_Init(uint32_t baud_rate);

// Envia um único caractere
void UART_Write(char data);

// Envia uma string inteira
void UART_WriteString(const char* str);

// Lê um caractere recebido
char UART_Read(void);

#endif // UART_H