#include "modbus.h"
#include "uart.h"

uint16_t holding_registers[20] = {0};

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

    // --- FUNÇÃO 0x03: Leitura de Registos (Read Holding Registers) ---
    if (frame[1] == 0x03) {  
        uint16_t start_addr = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t num_regs = ((uint16_t)frame[4] << 8) | frame[5];

        // CORREÇÃO 1: Limite de segurança aumentado para cobrir os 20 registos do mapa
        if (start_addr + num_regs > 20) return;  

        // CORREÇÃO 2: Buffer aumentado para 64 bytes para evitar overflow
        uint8_t tx_buffer[64]; 
        tx_buffer[0] = SLAVE_ID;
        tx_buffer[1] = 0x03;
        tx_buffer[2] = (uint8_t)(num_regs * 2); // Contagem de bytes de dados

        uint8_t tx_idx = 3;
        for (uint16_t i = 0; i < num_regs; i++) {
            uint16_t val = holding_registers[start_addr + i];
            tx_buffer[tx_idx++] = (val >> 8) & 0xFF; // Byte alto
            tx_buffer[tx_idx++] = val & 0xFF;        // Byte baixo
        }

        // Calcula o CRC da resposta
        uint16_t tx_crc = Modbus_CRC16(tx_buffer, tx_idx);
        
        tx_buffer[tx_idx++] = (uint8_t)(tx_crc & 0xFF);        // Low Byte
        tx_buffer[tx_idx++] = (uint8_t)((tx_crc >> 8) & 0xFF); // High Byte

        // Envia a resposta binária
        for(uint16_t i = 0; i < tx_idx; i++) {
            UART_Write(tx_buffer[i]);
        }
    }
    
    // --- CORREÇÃO 3: Adicionado suporte à FUNÇÃO 0x06 (Escrita de Registo Único) ---
    else if (frame[1] == 0x06) {
        uint16_t reg_addr = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t reg_value = ((uint16_t)frame[4] << 8) | frame[5];

        if (reg_addr < 20) {
            holding_registers[reg_addr] = reg_value;
        }

        // Na função 0x06, a resposta é um eco exato do pedido
        for (uint8_t i = 0; i < len; i++) {
            UART_Write(frame[i]);
        }
    }
    
    // --- CORREÇÃO 3 (Opcional): Suporte à FUNÇÃO 0x10 (Escrita de Múltiplos Registos) ---
    // Alguns sistemas SCADA preferem enviar "Sets" agrupados nesta função
    else if (frame[1] == 0x10) {
        uint16_t start_addr = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t num_regs = ((uint16_t)frame[4] << 8) | frame[5];
        
        if (start_addr + num_regs <= 20) {
            uint8_t data_idx = 7; // Os dados começam no byte 7
            for (uint16_t i = 0; i < num_regs; i++) {
                holding_registers[start_addr + i] = ((uint16_t)frame[data_idx] << 8) | frame[data_idx + 1];
                data_idx += 2;
            }
        }

        // Resposta de confirmação para a função 0x10
        uint8_t tx_buffer[8];
        tx_buffer[0] = SLAVE_ID;
        tx_buffer[1] = 0x10;
        tx_buffer[2] = frame[2];
        tx_buffer[3] = frame[3];
        tx_buffer[4] = frame[4];
        tx_buffer[5] = frame[5];
        
        uint16_t tx_crc = Modbus_CRC16(tx_buffer, 6);
        tx_buffer[6] = (uint8_t)(tx_crc & 0xFF);
        tx_buffer[7] = (uint8_t)((tx_crc >> 8) & 0xFF);
        
        for(uint8_t i = 0; i < 8; i++) {
            UART_Write(tx_buffer[i]);
        }
    }
}