#ifndef MODBUS_H
#define MODBUS_H

#include <xc.h>
#include <stdint.h>

#define SLAVE_ID 1

// Mapa de Holding Registers (O que o ScadaBR vai ler)
// 0: Estado atual do botão (1 = pressionado, 0 = solto)
// 1: Duração do último evento (segundos)
// 2: Ano do último evento
// 3: Mês (High Byte) / Dia (Low Byte)
// 4: Hora (High Byte) / Minuto (Low Byte)
// 5: Segundo do último evento
extern uint16_t holding_registers[10];

void Modbus_ProcessFrame(uint8_t *frame, uint8_t len);

#endif