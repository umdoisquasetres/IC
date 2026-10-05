#include "modbus.h"
#include "uart.h"

uint16_t holding_registers[10] = {0};

// Função padrão Modbus para calcular o Checksum (CRC16)
uint16_t Modbus_CRC16(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Processa o frame recebido
void Modbus_ProcessFrame(uint8_t *frame, uint8_t len) {
    if (len < 4) return; // Tamanho mínimo (Endereço + Função + 2 bytes de CRC)
    if (frame[0] != SLAVE_ID) return; // Verifica se é o endereço deste PIC

    // O CRC no Modbus RTU vem com o Low Byte primeiro e o High Byte depois
    uint16_t crc_rx = ((uint16_t)frame[len-1] << 8) | frame[len-2];
    uint16_t crc_calc = Modbus_CRC16(frame, len-2);
    if (crc_rx != crc_calc) return; // Ignora se o CRC não bater certo

    // O ScadaBR enviará a Função 0x03 (Read Holding Registers)
    if (frame[1] == 0x03) {  
        uint16_t start_addr = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t num_regs = ((uint16_t)frame[4] << 8) | frame[5];

        // Limite de segurança do nosso mapa (10 registos)
        if (start_addr + num_regs > 10) return;  

        uint8_t tx_buffer[32];
        tx_buffer[0] = SLAVE_ID;
        tx_buffer[1] = 0x03;
        tx_buffer[2] = (uint8_t)(num_regs * 2); // Contagem de bytes de dados

        uint8_t tx_idx = 3;
        for (uint16_t i = 0; i < num_regs; i++) {
            uint16_t val = holding_registers[start_addr + i];
            tx_buffer[tx_idx++] = (val >> 8) & 0xFF; // Byte alto
            tx_buffer[tx_idx++] = val & 0xFF;         // Byte baixo
        }

        // Calcula o CRC da resposta
        uint16_t tx_crc = Modbus_CRC16(tx_buffer, tx_idx);
        
        // ORDEM CORRETA MODBUS RTU: Low Byte primeiro, High Byte depois
        tx_buffer[tx_idx++] = (uint8_t)(tx_crc & 0xFF);        // Low Byte
        tx_buffer[tx_idx++] = (uint8_t)((tx_crc >> 8) & 0xFF); // High Byte

        // Envia a resposta binária via UART para o ScadaBR
        for(uint16_t i = 0; i < tx_idx; i++) {
            UART_Write(tx_buffer[i]);
        }
    }
}