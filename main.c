#include "config.h"
#include "uart.h"
#include "mpu6050.h"
#include "modbus.h"
#include <xc.h>
#include <stdint.h>

static volatile uint8_t sample_due = 0;
static volatile uint16_t sample_overruns = 0;
static uint16_t last_overrun_snapshot = 0;

#define VIBRATION_WINDOW_SAMPLES 64U
#define ACCEL_SCALE_DIVISOR 16L

static int32_t vibration_sum[3] = {0, 0, 0};
static uint32_t vibration_sum_squares[3] = {0, 0, 0};
static int16_t vibration_min[3];
static int16_t vibration_max[3];
static uint8_t vibration_window_count = 0;

static uint16_t integer_sqrt(uint32_t value)
{
    uint32_t bit = 1UL << 30;
    uint32_t result = 0;

    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint16_t)result;
}

static void reset_vibration_window(void)
{
    vibration_window_count = 0;
    for (uint8_t axis = 0; axis < 3; axis++) {
        vibration_sum[axis] = 0;
        vibration_sum_squares[axis] = 0;
    }
}

static void update_vibration_metrics(int16_t ax, int16_t ay, int16_t az)
{
    int16_t raw[3] = {ax, ay, az};

    for (uint8_t axis = 0; axis < 3; axis++) {
        int16_t scaled = (int16_t)(raw[axis] / ACCEL_SCALE_DIVISOR);
        int32_t sample = scaled;

        if (vibration_window_count == 0U) {
            vibration_min[axis] = scaled;
            vibration_max[axis] = scaled;
        } else {
            if (scaled < vibration_min[axis]) vibration_min[axis] = scaled;
            if (scaled > vibration_max[axis]) vibration_max[axis] = scaled;
        }

        vibration_sum[axis] += sample;
        vibration_sum_squares[axis] += (uint32_t)(sample * sample);
    }

    vibration_window_count++;
    if (vibration_window_count >= VIBRATION_WINDOW_SAMPLES) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            int32_t mean = vibration_sum[axis] / VIBRATION_WINDOW_SAMPLES;
            uint32_t mean_square = (uint32_t)(mean * mean);
            uint32_t mean_of_squares = vibration_sum_squares[axis] / VIBRATION_WINDOW_SAMPLES;
            uint32_t variance = mean_of_squares > mean_square ? mean_of_squares - mean_square : 0U;
            uint32_t rms_scaled = integer_sqrt(variance);
            uint32_t peak_to_peak_scaled = (uint16_t)(vibration_max[axis] - vibration_min[axis]);

            // As amostras escaladas têm 1024 unidades por g; publica RMS e pico a pico em mg.
            holding_registers[6U + axis] = (uint16_t)((rms_scaled * 1000UL) / 1024UL);
            holding_registers[9U + axis] = (uint16_t)((peak_to_peak_scaled * 1000UL) / 1024UL);
        }
        reset_vibration_window();
    }
}

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

// Período do Timer2 = 250 * 3,2 us * 5 = 4 ms com Fosc=20 MHz.
void __interrupt() ISR(void)
{
    if (PIR1bits.RCIF && PIE1bits.RCIE) {
        UART_ReceiveISR();
    }
    if (PIR1bits.TMR2IF && PIE1bits.TMR2IE) {
        PIR1bits.TMR2IF = 0;
        if (sample_due && sample_overruns != 0xFFFFU) {
            sample_overruns++;
        }
        sample_due = 1;
    }
}

void main(void)
{
    int16_t ax = 0, ay = 0, az = 0;
    uint8_t sensor_ready;
    ADCON1 = 0x0F;
    UART_Init(115200);
    // DE permanece baixo: o texto aparece em RC6, mas não entra no barramento RS-485.
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

    // Timer2: Fosc/4, prescaler 1:16, PR2=249, postscaler 1:5.
    T2CON = 0x00;
    PR2 = 249;
    TMR2 = 0;
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;
    INTCONbits.PEIE = 1;
    T2CON = 0x26;
    INTCONbits.GIE = 1;

    while (1) {
        // O pedido 0x03 tem sempre oito bytes; só há resposta após CRC válido.
        while (UART_DataReady()) {
            Modbus_ProcessByte(UART_Read());
        }

        if (sample_due) {
            INTCONbits.GIE = 0;
            sample_due = 0;
            uint16_t overrun_snapshot = sample_overruns;
            INTCONbits.GIE = 1;

            if (sensor_ready && MPU6050_ReadAccel(&ax, &ay, &az)) {
                holding_registers[0] = (uint16_t)ax;
                holding_registers[1] = (uint16_t)ay;
                holding_registers[2] = (uint16_t)az;
                holding_registers[3] = 0U;
                if (overrun_snapshot == last_overrun_snapshot) {
                    update_vibration_metrics(ax, ay, az);
                } else {
                    // Descarta uma janela se houve perda de ticks, pois o intervalo ficou irregular.
                    reset_vibration_window();
                }
            } else {
                holding_registers[3] = 1U;
                reset_vibration_window();
            }
            // A sequência acompanha os ticks do temporizador de aquisição, inclusive os que
            // se acumularam enquanto o código principal estava ocupado.
            holding_registers[4] = (uint16_t)(holding_registers[4] + 1U +
                                              (uint16_t)(overrun_snapshot - last_overrun_snapshot));
            last_overrun_snapshot = overrun_snapshot;
            holding_registers[5] = overrun_snapshot;
        }
    }
}
