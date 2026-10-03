#include "config.h"
#include "rtc_eeprom.h"
#include "uart.h"
#include <xc.h>
#include <stdio.h> // Para o uso do sprintf

#define BOTAO PORTBbits.RB0     
#define LED LATAbits.LATA0  

// ISR do Timer0 
void __interrupt() ISR(void) {
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        Timer0_AtualizaRelogio(); // Chama a função externa para limpar o main
    }
}

void main(void) {
    // Inicialização do Hardware
    ADCON1 = 0x0F;          // Pinos como digitais
    TRISBbits.TRISB0 = 1;   // RB0 como entrada (botão)
    TRISAbits.TRISA0 = 0;   // RA0 saída (LED)
    LATAbits.LATA0 = 0;     // LED começa apagado
    
    // Inicializa UART a 9600 bps
    UART_Init(9600);
    UART_WriteString("\r\n--- Sistema de Coleta de Dados Iniciado ---\r\n");

    // Configuração Timer0
    T0CON = 0b10001000;     // Modo 16-bits, Fosc/4, Prescaler 1:1
    TMR0H = 0xEC;
    TMR0L = 0x78;

    // Interrupções
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIE = 1;

    uint8_t estado_anterior = 0;
    RealTimeClock momento_inicio;
    uint32_t ms_inicio_evento = 0;
    
    // Buffer para armazenar as mensagens da UART
    char mensagem_serial[64]; 

    while(1) {
        uint8_t estado_atual = BOTAO;

        // Borda de subida (Entrada foi para nível ALTO)
        if (estado_atual == 1 && estado_anterior == 0) {
            __delay_ms(20); // Debounce
            if (BOTAO == 1) {
                momento_inicio = rtc; 
                ms_inicio_evento = ((uint32_t)rtc.hora * 3600000) + ((uint32_t)rtc.minuto * 60000) + (rtc.segundo * 1000) + ms_contador;
                LED = 1; 
                
                UART_WriteString(">> Botao PRESSIONADO. Registrando inicio do evento...\r\n");
            }
        }
    
        // Borda de descida (Entrada voltou para nível BAIXO)
        else if (estado_atual == 0 && estado_anterior == 1) {
            __delay_ms(20); // Debounce
            if (BOTAO == 0) {
                uint32_t ms_fim_evento = ((uint32_t)rtc.hora * 3600000) + ((uint32_t)rtc.minuto * 60000) + (rtc.segundo * 1000) + ms_contador;
                uint32_t duracao_total = ms_fim_evento - ms_inicio_evento;
                LED = 0;  
                
                // Grava EEPROM
                salvar_evento_eeprom(momento_inicio, duracao_total);
                
                // Transmite os dados salvos pela porta serial
                sprintf(mensagem_serial, "<< Botao SOLTO. Duracao: %lu ms. Evento salvo na EEPROM.\r\n", duracao_total);
                UART_WriteString(mensagem_serial);
            }
        }
        
        estado_anterior = estado_atual;
    }
}