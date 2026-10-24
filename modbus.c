#include "modbus.h"
#include "uart.h"

volatile uint16_t holding_registers[MODBUS_REGISTER_COUNT] = {0};

#define MODBUS_REQUEST_LENGTH 8U
#define MODBUS_MAX_READ_REGISTERS 20U

static uint8_t request[MODBUS_REQUEST_LENGTH];
static uint8_t request_index = 0;

static uint16_t modbus_crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void send_response(uint8_t *response, uint8_t length)
{
    uint16_t crc = modbus_crc16(response, length);
    response[length++] = (uint8_t)crc;       // RTU CRC low byte first.
    response[length++] = (uint8_t)(crc >> 8);

    UART_TransmitBegin();
    for (uint8_t i = 0; i < length; i++) {
        UART_Write(response[i]);
    }
    UART_TransmitEnd();
}

static void send_exception(uint8_t function, uint8_t exception)
{
    uint8_t response[3];
    response[0] = MODBUS_SLAVE_ID;
    response[1] = function | 0x80U;
    response[2] = exception;
    send_response(response, sizeof(response));
}

static void process_request(const uint8_t *frame)
{
    uint16_t received_crc = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    if (modbus_crc16(frame, 6) != received_crc) return;

    uint8_t function = frame[1];
    if (function != 0x03U) {
        send_exception(function, 0x01U); // Illegal function.
        return;
    }

    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    if (count == 0U || count > MODBUS_MAX_READ_REGISTERS) {
        send_exception(function, 0x03U); // Illegal data value.
        return;
    }
    if (start >= MODBUS_REGISTER_COUNT || count > MODBUS_REGISTER_COUNT - start) {
        send_exception(function, 0x02U); // Illegal data address.
        return;
    }

    uint8_t response[3U + MODBUS_MAX_READ_REGISTERS * 2U];
    response[0] = MODBUS_SLAVE_ID;
    response[1] = function;
    response[2] = (uint8_t)(count * 2U);
    uint8_t index = 3U;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t value = holding_registers[start + i];
        response[index++] = (uint8_t)(value >> 8);
        response[index++] = (uint8_t)value;
    }
    send_response(response, index);
}

void Modbus_ProcessByte(uint8_t value)
{
    if (request_index == 0U) {
        if (value != MODBUS_SLAVE_ID) return;
        request[request_index++] = value;
        return;
    }

    request[request_index++] = value;
    if (request_index == MODBUS_REQUEST_LENGTH) {
        request_index = 0;
        process_request(request);
    }
}
