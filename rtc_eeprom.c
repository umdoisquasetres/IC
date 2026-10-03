#include "rtc_eeprom.h"
#include <xc.h>

// Instanciação das variáveis globais
volatile RealTimeClock rtc = {0, 0, 12, 7, 9, 2026};
volatile uint32_t tempo_ms_total = 0;
volatile uint16_t ms_contador = 0;
uint16_t endereco_eeprom = 0x000; 

void EEPROM_WriteByte(uint16_t endereco, uint8_t dado) {
    EEADRH = (uint8_t)((endereco >> 8) & 0x03);
    EEADR  = (uint8_t)(endereco & 0xFF);
    EEDATA = dado;
    
    EECON1bits.EEPGD = 0; 
    EECON1bits.CFGS  = 0; 
    EECON1bits.WREN  = 1; 
    
    uint8_t gie_state = INTCONbits.GIE;
    INTCONbits.GIE = 0;   
    
    EECON2 = 0x55;        
    EECON2 = 0xAA;
    EECON1bits.WR = 1;    
    
    INTCONbits.GIE = gie_state; 
      
    while(EECON1bits.WR); 
    EECON1bits.WREN = 0;  
}

void salvar_evento_eeprom(RealTimeClock inicio, uint32_t duracao_ms) {
    if (endereco_eeprom > 1016) return; // Limite da EEPROM

    EEPROM_WriteByte(endereco_eeprom++, inicio.dia);
    EEPROM_WriteByte(endereco_eeprom++, inicio.mes);
    EEPROM_WriteByte(endereco_eeprom++, (uint8_t)(inicio.ano - 2000));
    EEPROM_WriteByte(endereco_eeprom++, inicio.hora);
    EEPROM_WriteByte(endereco_eeprom++, inicio.minuto);
    EEPROM_WriteByte(endereco_eeprom++, inicio.segundo);
    
    uint16_t duracao_sec = (uint16_t)(duracao_ms / 1000);
    EEPROM_WriteByte(endereco_eeprom++, (duracao_sec >> 8) & 0xFF);
    EEPROM_WriteByte(endereco_eeprom++, duracao_sec & 0xFF);
}

//Essa função faz a leitura de um byte
uint8_t EEPROM_ReadByte(uint16_t endereco) {
    EEADRH = (uint8_t)((endereco >> 8) & 0x03);
    EEADR  = (uint8_t)(endereco & 0xFF);
    
    EECON1bits.EEPGD = 0; // Aponta para memória EEPROM
    EECON1bits.CFGS  = 0; // Acessa Flash/EEPROM
    EECON1bits.RD    = 1; // Inicia a leitura
    
    return EEDATA;        // Retorna o dado lido
}

// Essa função encapsula a lógica que ficava no TMR0IF
void Timer0_AtualizaRelogio(void) {
    TMR0H = 0xEC; // Recarrega para exatamente 1ms @ 20MHz
    TMR0L = 0x78;
    
    ms_contador++;
    tempo_ms_total++;
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
                    if (rtc.dia > 30) { 
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