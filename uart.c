#include "uart.h"
#include "config.h"
#include <xc.h>

#define UART_RX_BUFFER_SIZE 64U
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1U)

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile uint8_t tx_in_progress = 0;

void UART_Init(uint32_t baud_rate)
{
    // A transmissão assíncrona exige RC6 como saída; RC7 permanece como entrada de recepção.
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 1;

    // Reinicializa a EUSART para não herdar modo ou baud antigo.
    TXSTAbits.TXEN = 0;
    RCSTAbits.SPEN = 0;
    TXSTA = 0;
    RCSTA = 0;
    BAUDCON = 0;

    // Assíncrono, alta velocidade e BRG de 16 bits: baud = Fosc/(4*(SPBRG+1)).
    TXSTAbits.BRGH = 1;
    BAUDCONbits.BRG16 = 1;
    uint32_t divisor = ((_XTAL_FREQ + (2UL * baud_rate)) / (4UL * baud_rate)) - 1UL;
    SPBRGH = (uint8_t)(divisor >> 8);
    SPBRG = (uint8_t)divisor;

    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;
    RCSTAbits.CREN = 1;
    RCSTAbits.SPEN = 1;
    PIE1bits.RCIE = 1;

    // RD0 controla DE do transceptor RS-485 (DE ativo em nível alto).
    LATDbits.LATD0 = 0;
    TRISDbits.TRISD0 = 0;
}

void UART_Write(uint8_t data)
{
    while (!PIR1bits.TXIF) { }
    TXREG = data;
}

void UART_WriteString(const char *str)
{
    while (*str != '\0') {
        UART_Write((uint8_t)*str++);
    }
}

void UART_WaitTXComplete(void)
{
    while (!TXSTAbits.TRMT) { }
}

uint8_t UART_DataReady(void)
{
    return (uint8_t)(rx_head != rx_tail);
}

uint8_t UART_Read(void)
{
    uint8_t value = rx_buffer[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) & UART_RX_BUFFER_MASK);
    return value;
}

void UART_ReceiveISR(void)
{
    while (PIR1bits.RCIF) {
        uint8_t framing_error = RCSTAbits.FERR;
        uint8_t value = RCREG;

        if (RCSTAbits.OERR) {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }

        // O /RE pode ficar sempre habilitado, fazendo o transceptor ecoar a própria
        // resposta enquanto DE está alto. Descarta esses bytes sem colocá-los
        // na fila como uma nova solicitação Modbus.
        if (!framing_error && !tx_in_progress) {
            uint8_t next = (uint8_t)((rx_head + 1U) & UART_RX_BUFFER_MASK);
            if (next != rx_tail) {
                rx_buffer[rx_head] = value;
                rx_head = next;
            }
        }
    }
}

void UART_TransmitBegin(void)
{
    tx_in_progress = 1;
    LATDbits.LATD0 = 1;
}

void UART_TransmitEnd(void)
{
    UART_WaitTXComplete();
    LATDbits.LATD0 = 0;
    // Permite que a interrupção de recepção descarte o último byte ecoado
    // enquanto a proteção de transmissão continua ativa.
    while (PIR1bits.RCIF) { }
    tx_in_progress = 0;
}
