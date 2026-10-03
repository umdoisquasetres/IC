#ifndef CONFIG_H
#define CONFIG_H

#include <xc.h>

// Configurações de Fuse Bits para Cristal Externo de 20 MHz
#pragma config OSC = HS         // High Speed Crystal
#pragma config WDT = OFF        // Watchdog Timer desativado
#pragma config LVP = OFF        // Low Voltage Programming desativado
#pragma config BOREN = OFF      // Brown-out Reset desativado
#pragma config IESO = OFF       // Desliga a troca automática
#pragma config MCLRE = ON       // Resetar o microchip
#pragma config PBADEN = OFF     // RB0-RB4 iniciam como digitais

#define _XTAL_FREQ 20000000     // Frequência do oscilador (20 MHz)

#endif // CONFIG_H