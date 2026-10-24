#include "mpu6050.h"
#include "config.h"
#include <xc.h>

#define MPU6050_ADDRESS_LOW   0x68
#define MPU6050_ADDRESS_HIGH  0x69
#define MPU6050_REG_SMPLRT    0x19
#define MPU6050_REG_CONFIG    0x1A
#define MPU6050_REG_ACCEL_CFG 0x1C
#define MPU6050_REG_ACCEL_XH  0x3B
#define MPU6050_REG_PWR_MGMT  0x6B
#define MPU6050_REG_WHO_AM_I  0x75
#define I2C_TIMEOUT           60000U

static uint8_t init_error = 0;
static uint8_t device_address = MPU6050_ADDRESS_LOW;

static uint8_t wait_idle(void)
{
    uint16_t timeout = I2C_TIMEOUT;
    while ((SSPCON2 & 0x1F) || SSPSTATbits.R_W) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static uint8_t i2c_start(void)
{
    if (!wait_idle()) return 0;
    SSPCON2bits.SEN = 1;
    return wait_idle();
}

static uint8_t i2c_restart(void)
{
    if (!wait_idle()) return 0;
    SSPCON2bits.RSEN = 1;
    return wait_idle();
}

static uint8_t i2c_stop(void)
{
    if (!wait_idle()) return 0;
    SSPCON2bits.PEN = 1;
    return wait_idle();
}

// Releases open-drain I2C pins and clocks a stuck slave out of a partial byte.
static void i2c_bus_recover(void)
{
    SSPCON1bits.SSPEN = 0;
    LATCbits.LATC3 = 0;
    LATCbits.LATC4 = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    __delay_us(5);

    if (!PORTCbits.RC4) {
        for (uint8_t pulse = 0; pulse < 9; pulse++) {
            TRISCbits.TRISC3 = 0; // Drive SCL low; never drive it high.
            __delay_us(5);
            TRISCbits.TRISC3 = 1; // Release SCL; the pull-up raises the line.
            __delay_us(5);
        }
    }

    // Generate STOP: SDA rises while SCL is released high.
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    __delay_us(5);
    TRISCbits.TRISC3 = 1;
    __delay_us(5);
    TRISCbits.TRISC4 = 1;
    __delay_us(5);
}

static void i2c_configure(void)
{
    SSPCON1 = 0x00;
    SSPCON2 = 0x00;
    SSPSTAT = 0x80; // SMP=1: slew-rate control disabled for standard mode.
    SSPADD = 49; // Fosc/(4 * (SSPADD + 1)) = 100 kHz at 20 MHz.
    SSPCON1 = 0x28; // Enable MSSP in I2C master mode.
}

static uint8_t i2c_write(uint8_t value)
{
    if (!wait_idle()) return 0;
    SSPBUF = value;
    if (!wait_idle()) return 0;
    return (SSPCON2bits.ACKSTAT == 0);
}

static uint8_t i2c_read(uint8_t *value, uint8_t acknowledge)
{
    if (!wait_idle()) return 0;
    SSPCON2bits.RCEN = 1;
    if (!wait_idle()) return 0;
    *value = SSPBUF;
    SSPCON2bits.ACKDT = acknowledge ? 0 : 1;
    SSPCON2bits.ACKEN = 1;
    return wait_idle();
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t ok = i2c_start();
    if (ok) ok = i2c_write((uint8_t)(device_address << 1));
    if (ok) ok = i2c_write(reg);
    if (ok) ok = i2c_write(value);
    if (!i2c_stop()) ok = 0;
    return ok;
}

static uint8_t read_registers(uint8_t reg, uint8_t *data, uint8_t count)
{
    uint8_t ok = i2c_start();
    if (ok) ok = i2c_write((uint8_t)(device_address << 1));
    if (ok) ok = i2c_write(reg);
    if (ok) ok = i2c_restart();
    if (ok) ok = i2c_write((uint8_t)((device_address << 1) | 1));

    for (uint8_t i = 0; ok && i < count; i++) {
        ok = i2c_read(&data[i], (uint8_t)(i + 1 < count));
    }
    if (!i2c_stop()) ok = 0;
    return ok;
}

uint8_t MPU6050_Init(void)
{
    uint8_t device_id = 0;
    init_error = 0;

    // MSSP I2C master on RC3=SCL and RC4=SDA, standard mode at 100 kHz.
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    i2c_bus_recover();
    i2c_configure();

    __delay_ms(100);
    if (!read_registers(MPU6050_REG_WHO_AM_I, &device_id, 1)) {
        // AD0 selects 0x68 or 0x69; probe both so either wiring works.
        device_address = MPU6050_ADDRESS_HIGH;
        if (!read_registers(MPU6050_REG_WHO_AM_I, &device_id, 1)) {
            // A failed transaction can leave a slave mid-byte; recover and retry.
            i2c_bus_recover();
            i2c_configure();
            device_address = MPU6050_ADDRESS_LOW;
            if (!read_registers(MPU6050_REG_WHO_AM_I, &device_id, 1)) {
                device_address = MPU6050_ADDRESS_HIGH;
                if (!read_registers(MPU6050_REG_WHO_AM_I, &device_id, 1)) {
                    init_error = 1; // No I2C response at either address.
                    return 0;
                }
            }
        }
    }
    if (device_id != 0x68) {
        init_error = 2; // Device responded, but returned an unexpected identity.
        return 0;
    }
    if (!write_register(MPU6050_REG_PWR_MGMT, 0x01)) {
        init_error = 3;
        return 0;
    } // Wake, X gyro clock.
    __delay_ms(10);
    if (!write_register(MPU6050_REG_CONFIG, 0x02)) {
        init_error = 4;
        return 0;
    } // Accelerometer DLPF bandwidth about 94 Hz.
    if (!write_register(MPU6050_REG_SMPLRT, 0x03)) {
        init_error = 5;
        return 0;
    } // 1 kHz base rate / (1 + 3) = 250 Hz.
    if (!write_register(MPU6050_REG_ACCEL_CFG, 0x00)) {
        init_error = 6;
        return 0;
    } // +/-2 g.
    return 1;
}

uint8_t MPU6050_GetInitError(void)
{
    return init_error;
}

uint8_t MPU6050_GetI2CLineState(void)
{
    // Release MSSP ownership before sampling the physical bus levels.
    SSPCON1bits.SSPEN = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    __delay_us(10);
    return (uint8_t)((PORTCbits.RC3 ? 1U : 0U) |
                     (PORTCbits.RC4 ? 2U : 0U));
}

uint8_t MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t data[6];
    if (!read_registers(MPU6050_REG_ACCEL_XH, data, sizeof(data))) return 0;

    *ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    return 1;
}



