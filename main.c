#include "config.h"
#include "uart.h"
#include "mpu6050.h"
#include "modbus.h"
#include <xc.h>
#include <stdint.h>

static volatile uint8_t sample_due = 0;

static void terminal_write_string(const char *text)
{
    UART_WriteString(text);
    UART_WaitTXComplete();
}

static void terminal_write_i16(int16_t value)
{
    char digits[6];
    uint8_t count = 0;
    int32_t remaining = value;

    if (remaining < 0) {
        UART_Write('-');
        remaining = -remaining;
    }
    do {
        digits[count++] = (char)('0' + (remaining % 10));
        remaining /= 10;
    } while (remaining != 0);
    while (count != 0) UART_Write((uint8_t)digits[--count]);
}

// Timer0 overflow every 4 ms at Fosc=20 MHz, prescaler 1:1.
void __interrupt() ISR(void)
{
    if (PIR1bits.RCIF && PIE1bits.RCIE) {
        UART_ReceiveISR();
    }
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0H = 0xB1;
        TMR0L = 0xE0;
        sample_due = 1;
    }
}

void main(void)
{
    int16_t ax = 0, ay = 0, az = 0;
    uint8_t sensor_ready;
    ADCON1 = 0x0F;
    UART_Init(9600);
    // DE permanece bajo: o texto aparece em RC6, mas não entra no barramento RS-485.
    terminal_write_string("BOOT\r\n");

    sensor_ready = MPU6050_Init();
    holding_registers[3] = sensor_ready ? 0U : 1U;
    if (sensor_ready) {
        terminal_write_string("MPU OK ID=0x68\r\n");
    } else {
        terminal_write_string("MPU ERRO #");
        terminal_write_i16((int16_t)MPU6050_GetInitError());
        uint8_t i2c_lines = MPU6050_GetI2CLineState();
        terminal_write_string(" SCL=");
        UART_Write((uint8_t)('0' + ((i2c_lines & 0x01U) != 0U)));
        terminal_write_string(" SDA=");
        UART_Write((uint8_t)('0' + ((i2c_lines & 0x02U) != 0U)));
        terminal_write_string("\r\n");
    }

    // Timer0 16-bit, clock interno Fosc/4, prescaler 1:1.
    T0CON = 0b10001000;
    TMR0H = 0xB1;
    TMR0L = 0xE0;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    uint16_t terminal_divider = 0;
    int16_t terminal_ax = 0, terminal_ay = 0, terminal_az = 0;

    while (1) {
        // O pedido 0x03 tem sempre oito bytes; só há resposta após CRC válido.
        while (UART_DataReady()) {
            Modbus_ProcessByte(UART_Read());
        }

        if (sample_due) {
            INTCONbits.GIE = 0;
            sample_due = 0;
            INTCONbits.GIE = 1;

            if (sensor_ready && MPU6050_ReadAccel(&ax, &ay, &az)) {
                holding_registers[0] = (uint16_t)ax;
                holding_registers[1] = (uint16_t)ay;
                holding_registers[2] = (uint16_t)az;
                holding_registers[3] = 0U;
            } else {
                holding_registers[3] = 1U;
            }

            terminal_ax = ax;
            terminal_ay = ay;
            terminal_az = az;
            if (++terminal_divider >= 250U) {
                terminal_divider = 0;
                terminal_write_string("X="); terminal_write_i16(terminal_ax);
                terminal_write_string(" Y="); terminal_write_i16(terminal_ay);
                terminal_write_string(" Z="); terminal_write_i16(terminal_az);
                terminal_write_string("\r\n");
            }
        }
    }
}
