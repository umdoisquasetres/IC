#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/*
 * Driver MPU6050 para PIC18F4685 com XC8.
 * Barramento MSSP I2C nos pinos RC3/SCL e RC4/SDA.
 *
 * Configuração inicial:
 * - endereço I2C: 0x68 (AD0 em nível baixo)
 * - taxa de saída: 1 kHz
 * - DLPF: configuração 0 (acelerômetro, banda nominal de 260 Hz)
 * - acelerômetro: +/-4 g (8192 LSB/g)
 *
 * A taxa de saída de 1 kHz não significa banda útil de 500 Hz:
 * respeite o filtro analógico/digital e o limite de Nyquist.
 */

#define MPU6050_I2C_ADDRESS       0x68u
#define MPU6050_ACCEL_LSB_PER_G   8192.0f

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_RawAccel;

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR_I2C = 1,
    MPU6050_ERROR_ID = 2
} MPU6050_Status;

/* Inicializa o MSSP em I2C master a aproximadamente 400 kHz. */
void I2C1_Init(void);

/* Inicialização do sensor. Retorna MPU6050_OK em caso de sucesso. */
MPU6050_Status MPU6050_Init(void);

/* Retorna 1 se DATA_RDY estiver ativo; 0 se não estiver ou houver erro. */
uint8_t MPU6050_DataReady(void);

/* Lê X/Y/Z em uma transação burst de seis bytes. */
MPU6050_Status MPU6050_ReadAccel(MPU6050_RawAccel *sample);

/* Converte um valor bruto para g, escala +/-4 g. */
float MPU6050_RawToG(int16_t raw);

#endif
