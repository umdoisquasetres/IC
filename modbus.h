#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>

#define MODBUS_SLAVE_ID 1U
#define MODBUS_REGISTER_COUNT 12U

// 0..2=eixos brutos, 3=estado, 4=sequência, 5=ticks perdidos, 6..8=RMS em mg, 9..11=pico a pico em mg.
extern volatile uint16_t holding_registers[MODBUS_REGISTER_COUNT];

void Modbus_ProcessByte(uint8_t value);

#endif
