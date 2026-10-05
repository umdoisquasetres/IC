#include <xc.h>
#include <stdio.h>
#include "rtc_eeprom.h"
#include "uart.h"

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
// Função que varre a EEPROM e envia todos os eventos gravados via UART
void ler_e_enviar_eeprom(void) {
    char buffer_serial[80];
    uint16_t end_leitura = 0; // Começa a ler do endereço 0x000

    // Verifica se há algo gravado
    if (endereco_eeprom == 0) {
        UART_WriteString(">> A EEPROM esta vazia. Nenhum evento registrado.\r\n");
        return;
    }

    UART_WriteString("\r\n--- INICIO DA LEITURA DA EEPROM ---\r\n");

    // Lê os dados de 8 em 8 bytes até atingir o limite do que foi gravado
    while (end_leitura < endereco_eeprom) {
        // 1. Reconstrução da Data e Hora
        uint8_t dia = EEPROM_ReadByte(end_leitura++);
        uint8_t mes = EEPROM_ReadByte(end_leitura++);
        uint16_t ano = EEPROM_ReadByte(end_leitura++) + 2000; // Restaura o ano completo
        
        uint8_t hora = EEPROM_ReadByte(end_leitura++);
        uint8_t minuto = EEPROM_ReadByte(end_leitura++);
        uint8_t segundo = EEPROM_ReadByte(end_leitura++);
        
        // 2. Reconstrução da duração de 16 bits (segundos)
        uint8_t duracao_high = EEPROM_ReadByte(end_leitura++);
        uint8_t duracao_low = EEPROM_ReadByte(end_leitura++);
        uint16_t duracao_sec = ((uint16_t)duracao_high << 8) | duracao_low; // Junta os dois bytes

        // 3. Formatação e Envio para o PC
        sprintf(buffer_serial, "Evento no end %03X: %02d/%02d/%04d %02d:%02d:%02d | Duracao: %u seg\r\n", 
                (end_leitura - 8), dia, mes, ano, hora, minuto, segundo, duracao_sec);
        
        UART_WriteString(buffer_serial);
    }
    
    UART_WriteString("--- FIM DA LEITURA ---\r\n");
}