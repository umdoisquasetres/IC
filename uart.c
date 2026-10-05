#include "uart.h"
#include "config.h"
#include <xc.h>

void UART_Init(uint32_t baud_rate) {
    TRISCbits.TRISC6 = 1; // TX
    TRISCbits.TRISC7 = 1; // RX

    uint32_t spbrg_val = (_XTAL_FREQ / (16 * baud_rate)) - 1;
    TXSTAbits.BRGH = 1;
    BAUDCONbits.BRG16 = 0;
    SPBRG = (uint8_t)spbrg_val;

    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;

    RCSTAbits.CREN = 1;
    RCSTAbits.SPEN = 1;
}

void UART_Write(char data) {
    while (!TXSTAbits.TRMT); 
    TXREG = data;            
}

void UART_WriteString(const char* str) {
    while (*str != '\0') {
        UART_Write(*str);
        str++;
    }
}

// Retorna 1 se houver dados prontos, 0 caso contrário
uint8_t UART_DataReady(void) {
    // Trata erro de Overrun (trava comum da UART do PIC)
    if (RCSTAbits.OERR) {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    return PIR1bits.RCIF;
}

// Lê o byte atual sem bloquear o programa se o buffer estiver vazio
char UART_Read(void) {
    return RCREG;            
}