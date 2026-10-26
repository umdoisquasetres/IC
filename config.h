#ifndef CONFIG_H
#define CONFIG_H

#include <xc.h>

// Configurações dos bits de configuração para cristal externo de 20 MHz
#pragma config OSC = HS         // Cristal externo de alta velocidade
#pragma config WDT = OFF        // Temporizador watchdog desativado
#pragma config LVP = OFF        // Programação em baixa tensão desativada
#pragma config BOREN = OFF      // Reset por baixa tensão desativado
#pragma config IESO = OFF       // Desativa a troca automática de oscilador
#pragma config MCLRE = ON       // Pino MCLR habilitado para reset
#pragma config PBADEN = OFF     // RB0-RB4 iniciam como pinos digitais

#define _XTAL_FREQ 20000000     // Frequência do oscilador (20 MHz)

#endif // CONFIG_H