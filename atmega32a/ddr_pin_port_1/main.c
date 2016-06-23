/*
 * @file    main.c
 * @brief   Set PORTC I/O pins using data direction register,
 *          read input from pins and set high PORTC.0 2 and 3
 *          if input pin PORTC.6 is high.
 * @date    06 Jun 2016 10:28 PM
 */


#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 1000000UL    /* 1 MHz clock speed */
#endif

#include <avr/io.h>
#include <util/delay.h>

int
main(void) {
    DDRC = 0x0F;    /* 00001111, set lower nibble of PORTC to OUTPUT, upper nibble to INPUT */
    PORTC = 0x0C;   /* set HIGH PORT PC3 and PC2 00001100 */

    while(1) {
        if (PINC == 0b01000000) { /* lets assume a 4V supply comes to PORTC.6 and Vcc = 5V */
            PORTC = 0x0B;       /* 00001011 but don't touch PORT PC2 - it stays LOW */
            _delay_ms(1000);    /* delay 1s */
        } else {
            PORTC = 0x00;       /* turn off all PORTs */
        }
    }
}
