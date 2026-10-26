#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>

#define MODBUS_SLAVE_ID 1U
#define MODBUS_REGISTER_COUNT 6U

// 0..2=eixos (valores brutos com sinal), 3=estado do sensor, 4=sequência, 5=ticks de aquisição perdidos.
extern volatile uint16_t holding_registers[MODBUS_REGISTER_COUNT];

void Modbus_ProcessByte(uint8_t value);

#endif
