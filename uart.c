#include "uart.h"
#include "config.h"
#include <xc.h>

void UART_Init(uint32_t baud_rate) {
    // Configura os pinos RX e TX como entrada (o módulo reconfigura o TX)
    TRISCbits.TRISC6 = 1; // TX
    TRISCbits.TRISC7 = 1; // RX

    // Configura o Baud Rate
    uint32_t spbrg_val = (_XTAL_FREQ / (16 * baud_rate)) - 1;
    TXSTAbits.BRGH = 1;
    BAUDCONbits.BRG16 = 0;
    SPBRG = (uint8_t)spbrg_val;

    // Habilita Transmissão (TX)
    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;

    // Habilita Recepção (RX) e Porta Serial
    RCSTAbits.CREN = 1;
    RCSTAbits.SPEN = 1;
}

void UART_Write(char data) {
    while (!TXSTAbits.TRMT); // Aguarda o buffer de transmissão ficar vazio
    TXREG = data;            // Envia o dado
}

void UART_WriteString(const char* str) {
    while (*str != '\0') {
        UART_Write(*str);
        str++;
    }
}

char UART_Read(void) {
    while (!PIR1bits.RCIF);  // Aguarda receber um dado
    return RCREG;            // Retorna o dado lido
}