#ifndef RTC_EEPROM_H
#define RTC_EEPROM_H

#include <stdint.h>

// Estrutura para Data e Hora
typedef struct {
    uint8_t  segundo;
    uint8_t  minuto;
    uint8_t  hora;
    uint8_t  dia;
    uint8_t  mes;
    uint16_t ano;
} RealTimeClock;

// Variáveis globais acessíveis a outros arquivos
extern volatile RealTimeClock rtc;
extern volatile uint32_t tempo_ms_total;
extern volatile uint16_t ms_contador;
extern uint16_t endereco_eeprom;

// Funções de manipulação
void EEPROM_WriteByte(uint16_t endereco, uint8_t dado);
void salvar_evento_eeprom(RealTimeClock inicio, uint32_t duracao_ms);
void Timer0_AtualizaRelogio(void);
uint8_t EEPROM_ReadByte(uint16_t endereco); // Função para ler um único byte
void ler_e_enviar_eeprom(void);

#endif // RTC_EEPROM_H