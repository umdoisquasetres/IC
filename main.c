#include "config.h"
#include "rtc_eeprom.h"
#include "uart.h"
#include "modbus.h"
#include <xc.h>
#include <stdio.h> 

#define BOTAO PORTBbits.RB0      
#define LED LATAbits.LATA0  

// Variáveis para o buffer Modbus
uint8_t modbus_rx_buffer[64];
volatile uint8_t modbus_rx_index = 0;
volatile uint8_t modbus_idle_timer = 0;
volatile uint8_t modbus_frame_ready = 0;

// ISR (Timer0 e Interrupção Serial UART)
void __interrupt() ISR(void) {
    // Tratamento do Timer0
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        Timer0_AtualizaRelogio(); 
        
        // --- Temporizador Modbus (Silêncio de Quadro) ---
        if (modbus_idle_timer < 5) {
            modbus_idle_timer++;
            if (modbus_idle_timer == 4 && modbus_rx_index > 0) {
                modbus_frame_ready = 1; 
            }
        }
    }
    
    // Tratamento de Receção UART por Interrupção (Garante que nenhum byte se perde)
    if (PIR1bits.RCIF) {
        uint8_t dado = RCREG;
        
        // Limpa erro de Overrun da UART caso ocorra
        if (RCSTAbits.OERR) {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }
        
        modbus_idle_timer = 0; 
        modbus_frame_ready = 0;
        
        if (modbus_rx_index < sizeof(modbus_rx_buffer)) {
            modbus_rx_buffer[modbus_rx_index++] = dado;
        }
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

    // Configuração Timer0
    T0CON = 0b10001000;     // Modo 16-bits, Fosc/4, Prescaler 1:1
    TMR0H = 0xEC;
    TMR0L = 0x78;

    // Configuração de Interrupções (Timer0 + Periféricos/UART)
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    
    PIE1bits.RCIE = 1;      // Habilita interrupção de receção da UART
    INTCONbits.PEIE = 1;    // Habilita interrupções de periféricos
    INTCONbits.GIE = 1;     // Habilita interrupções globais

    uint8_t estado_anterior = 0;
    RealTimeClock momento_inicio;
    uint32_t ms_inicio_evento = 0;
    holding_registers[16] = 0;

    while(1) {
        if (holding_registers[16] == 1) {
            // Desativa Timer0 temporariamente para evitar corrupção durante a cópia
            INTCONbits.TMR0IE = 0; 
            
            // Copia os dados dos registradores Modbus para a estrutura do relógio
            rtc.ano     = (uint16_t)holding_registers[10];
            rtc.mes     = (uint8_t)holding_registers[11];
            rtc.dia     = (uint8_t)holding_registers[12];
            rtc.hora    = (uint8_t)holding_registers[13];
            rtc.minuto  = (uint8_t)holding_registers[14];
            rtc.segundo = (uint8_t)holding_registers[15];
            
            // Zera o gatilho para aguardar a próxima sincronização
            holding_registers[16] = 0; 
            
            // Reativa Timer0
            INTCONbits.TMR0IE = 1; 
        }
        
        uint8_t estado_atual = BOTAO;
        holding_registers[0] = estado_atual; // Reg 0: Status em tempo real

        // Borda de subida (Entrada foi para nível ALTO)
        if (estado_atual == 1 && estado_anterior == 0) {
            __delay_ms(20); // Debounce
            if (BOTAO == 1) {
                momento_inicio = rtc; 
                ms_inicio_evento = ((uint32_t)rtc.hora * 3600000) + ((uint32_t)rtc.minuto * 60000) + ((uint32_t)rtc.segundo * 1000) + ms_contador;
                LED = 1; 
            }
        }
    
        // Borda de descida (Entrada voltou para nível BAIXO)
        else if (estado_atual == 0 && estado_anterior == 1) {
            __delay_ms(20); // Debounce
            if (BOTAO == 0) {
                uint32_t ms_fim_evento = ((uint32_t)rtc.hora * 3600000) + ((uint32_t)rtc.minuto * 60000) + ((uint32_t)rtc.segundo * 1000) + ms_contador;
                uint32_t duracao_total = ms_fim_evento - ms_inicio_evento;
                
                LED = 0;  
                
                // Grava EEPROM
                salvar_evento_eeprom(momento_inicio, duracao_total);
                
                // Atualiza os registradores Modbus
                holding_registers[1] = (uint16_t)(duracao_total / 1000); // Segundos
                holding_registers[2] = momento_inicio.ano;
                holding_registers[3] = ((uint16_t)momento_inicio.mes << 8) | momento_inicio.dia;
                holding_registers[4] = ((uint16_t)momento_inicio.hora << 8) | momento_inicio.minuto;
                holding_registers[5] = momento_inicio.segundo;
            }
        }
        
        estado_anterior = estado_atual;
        
        // --- Processamento do Pacote Modbus ---
        if (modbus_frame_ready) {
            Modbus_ProcessFrame(modbus_rx_buffer, modbus_rx_index);
            modbus_rx_index = 0; 
            modbus_frame_ready = 0;
        }
    }
}