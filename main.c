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
#define SENSOR_RETRY_PERIOD_TICKS 250U

static int32_t vibration_sum[3] = {0, 0, 0};
static uint32_t vibration_sum_squares[3] = {0, 0, 0};
static int16_t vibration_min[3];
static int16_t vibration_max[3];
static uint8_t vibration_window_count = 0;

static uint32_t integer_sqrt(uint64_t value)
{
    uint64_t bit = 1ULL << 62;
    uint64_t result = 0;

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
    return (uint32_t)result;
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
            int64_t sum = vibration_sum[axis];
            uint64_t count = VIBRATION_WINDOW_SAMPLES;
            uint64_t variance_numerator =
                (uint64_t)vibration_sum_squares[axis] * count - (uint64_t)(sum * sum);
            uint32_t rms_scaled = integer_sqrt(variance_numerator) / VIBRATION_WINDOW_SAMPLES;
            uint32_t peak_to_peak_scaled = (uint16_t)(vibration_max[axis] - vibration_min[axis]);

            // As amostras escaladas têm 1024 unidades por g; publica RMS e pico a pico em mg.
            holding_registers[6U + axis] = (uint16_t)((rms_scaled * 1000UL) / 1024UL);
            holding_registers[9U + axis] = (uint16_t)((peak_to_peak_scaled * 1000UL) / 1024UL);
        }
        reset_vibration_window();
    }
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
    uint8_t consecutive_read_failures = 0;
    uint16_t sensor_retry_ticks = 0;
    ADCON1 = 0x0F;
    UART_Init(115200);

    sensor_ready = MPU6050_Init();
    holding_registers[3] = sensor_ready ? 0U : 1U;

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
                consecutive_read_failures = 0;
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
                if (sensor_ready) {
                    if (++consecutive_read_failures >= 3U) {
                        sensor_ready = 0U;
                        consecutive_read_failures = 0U;
                        sensor_retry_ticks = 0U;
                    }
                } else if (++sensor_retry_ticks >= SENSOR_RETRY_PERIOD_TICKS) {
                    sensor_retry_ticks = 0U;
                    sensor_ready = MPU6050_Init();
                }
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
