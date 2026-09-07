#include <xc.h>
#include <stdint.h>

// Configurações de Fuse Bits para Cristal Externo de 20 MHz
#pragma config OSC = HS         // High Speed Crystal
#pragma config WDT = OFF        // Watchdog Timer desativado
#pragma config LVP = OFF        // Low Voltage Programming desativado
#pragma config BOREN = OFF      // Brown-out Reset desativado

#define _XTAL_FREQ 20000000     // Frequência do oscilador (20 MHz)
#define BOTAO PORTBbits.RB0     // Entrada digital a ser monitorada

// Estrutura para Data e Hora
typedef struct {
    uint8_t  segundo;
    uint8_t  minuto;
    uint8_t  hora;
    uint8_t  dia;
    uint8_t  mes;
    uint16_t ano;
} RealTimeClock;

// Inicializa com uma data/hora base (Ex: 07/09/2026 12:00:00)
volatile RealTimeClock rtc = {0, 0, 12, 7, 9, 2026};
volatile uint16_t ms_contador = 0;

uint16_t endereco_eeprom = 0x000; 

// --- Função Nativa para Escrita na EEPROM (Substitui as funções legadas) ---
void EEPROM_WriteByte(uint16_t endereco, uint8_t dado) {
    EEADRH = (uint8_t)((endereco >> 8) & 0x03); // PIC18F4685 possui 1024 bytes (0x000-0x3FF)
    EEADR  = (uint8_t)(endereco & 0xFF);
    EEDATA = dado;
    
    EECON1bits.EEPGD = 0; // Aponta para memória EEPROM
    EECON1bits.CFGS  = 0; // Acessa Flash/EEPROM
    EECON1bits.WREN  = 1; // Habilita escrita
    
    uint8_t gie_state = INTCONbits.GIE;
    INTCONbits.GIE = 0;   // Desabilita interrupções temporariamente (obrigatório)
    
    EECON2 = 0x55;        // Sequência de destravamento da Microchip
    EECON2 = 0xAA;
    EECON1bits.WR = 1;    // Inicia gravação
    
    INTCONbits.GIE = gie_state; // Restaura interrupções
    
    while(EECON1bits.WR); // Aguarda fim da escrita (~4ms)
    EECON1bits.WREN = 0;  // Trava escrita por segurança
}

// ISR do Timer0 (Gera base de tempo de 1 ms e atualiza o relógio)
void __interrupt() ISR(void) {
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        
        TMR0H = 0xEC; // Recarrega para exatamente 1ms @ 20MHz
        TMR0L = 0x78;
        
        ms_contador++;
        if (ms_contador >= 1000) {
            ms_contador = 0;
            rtc.segundo++;
            if (rtc.segundo >= 60) {
                rtc.segundo = 0;
                rtc.minuto++;
                if (rtc.minuto >= 60) {
                    rtc.minuto = 0;
                    rtc.hora++;
                    if (rtc.hora >= 24) {
                        rtc.hora = 0;
                        rtc.dia++;
                        if (rtc.dia > 30) { // Lógica simplificada de dias
                            rtc.dia = 1;
                            rtc.mes++;
                            if (rtc.mes > 12) {
                                rtc.mes = 1;
                                rtc.ano++;
                            }
                        }
                    }
                }
            }
        }
    }
}

// Função para gravar evento com Data, Hora e Duração na EEPROM (8 bytes por registro)
void salvar_evento_eeprom(RealTimeClock inicio, uint32_t duracao_ms) {
    if (endereco_eeprom > 1016) return; // Limite da EEPROM (1024 bytes)

    EEPROM_WriteByte(endereco_eeprom++, inicio.dia);
    EEPROM_WriteByte(endereco_eeprom++, inicio.mes);
    EEPROM_WriteByte(endereco_eeprom++, (uint8_t)(inicio.ano - 2000)); // Salva ano resumido (ex: 26)
    EEPROM_WriteByte(endereco_eeprom++, inicio.hora);
    EEPROM_WriteByte(endereco_eeprom++, inicio.minuto);
    EEPROM_WriteByte(endereco_eeprom++, inicio.segundo);
    
    // Salva a duração em milissegundos (divida em 2 bytes para economizar espaço)
    uint16_t duracao_sec = (uint16_t)(duracao_ms / 1000);
    EEPROM_WriteByte(endereco_eeprom++, (duracao_sec >> 8) & 0xFF);
    EEPROM_WriteByte(endereco_eeprom++, duracao_sec & 0xFF);
}

void main(void) {
    ADCON1 = 0x0F;          // Pinos como digitais
    TRISBbits.TRISB0 = 1;   // RB0 como entrada
    
    // Timer0: Modo 16-bits, Fosc/4, Prescaler 1:1
    T0CON = 0b10001000;     
    TMR0H = 0xEC;
    TMR0L = 0x78;

    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIE = 1;

    uint8_t estado_anterior = 0;
    RealTimeClock momento_inicio;
    uint32_t ms_inicio_evento = 0;

    while(1) {
        uint8_t estado_atual = BOTAO;

        // Borda de subida (Entrada foi para nível ALTO)
        if (estado_atual == 1 && estado_anterior == 0) {
            __delay_ms(20); // Debounce
            if (BOTAO == 1) {
                momento_inicio = rtc; // Copia a data/hora atual
                // Salva timestamp em ms absoluto para calcular duração
                ms_inicio_evento = ((uint32_t)rtc.hora * 3600000) + ((uint32_t)rtc.minuto * 60000) + (rtc.segundo * 1000) + ms_contador;
            }
        }
        
        // Borda de descida (Entrada voltou para nível BAIXO)
        else if (estado_atual == 0 && estado_anterior == 1) {
            __delay_ms(20); // Debounce
            if (BOTAO == 0) {
                uint32_t ms_fim_evento = ((uint32_t)rtc.hora * 3600000) + ((uint32_t)rtc.minuto * 60000) + (rtc.segundo * 1000) + ms_contador;
                uint32_t duracao_total = ms_fim_evento - ms_inicio_evento;
                
                // Grava o registro completo na EEPROM
                salvar_evento_eeprom(momento_inicio, duracao_total);
            }
        }
        
        estado_anterior = estado_atual;
    }
}