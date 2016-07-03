/*
 * @file    main.c
 * @brief   Set PORTC I/O pins using data direction register,
 *          read input from pins and set high PORTC.0 1 and 3
 *          if input pin PORTC.6 is high. PORTC2 stays HIGH always.
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
    PORTC = 0xFC;   /* set HIGH PORTC.3 and PORTC.2 00001100, Low PORTC.1 PORTC.0 and set pull-ups on all INPUT pins */

    while(1) {
        if (PINC & (1 << PC6)) { /* lets assume a 4V supply comes to PORTC.6 and Vcc = 5V */
            PORTC |= (1 << PC3);       /* 00001011 but don't touch PORT PC2 - it stays HIGH */
            PORTC |= (1 << PC1);       /* 00001011 but don't touch PORT PC2 - it stays HIGH */
            PORTC |= (1 << PC0);       /* 00001011 but don't touch PORT PC2 - it stays HIGH */
            _delay_ms(1000);    /* delay 1s */
        } else {
            PORTC &= ~(1 << PC3);       /* turn off all output pins */
            PORTC &= ~(1 << PC1);       /* turn off all output pins */
            PORTC &= ~(1 << PC0);       /* turn off all output pins */
            _delay_ms(1000);    /* delay 1s */
        }
    }
}
