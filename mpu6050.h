#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetInitError(void);
uint8_t MPU6050_GetI2CLineState(void);
uint8_t MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);

#endif
