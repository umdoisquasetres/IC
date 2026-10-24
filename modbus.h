#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>

#define MODBUS_SLAVE_ID 1U
#define MODBUS_REGISTER_COUNT 4U

// Holding registers: 0=X, 1=Y, 2=Z (signed raw MPU6050), 3=status (0=OK, 1=I2C error).
extern volatile uint16_t holding_registers[MODBUS_REGISTER_COUNT];

void Modbus_ProcessByte(uint8_t value);

#endif
