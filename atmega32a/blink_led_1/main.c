/*
 * @file    main.c
 * @brief   Blink leds on PORTA 0.
 * @date    27 Apr 2016 01:47 PM
 */


#ifndef F_CPU
#define F_CPU 16000000UL    /* 16 MHz clock speed */
#endif


#include <avr/io.h>
#include <util/delay.h>


int
main(void) {
    DDRA = 0xFF;            /* Nakes PORTA as output,
                             * mark all 8 pins as output  */

    while (1)
    {
        PORTA = 0x01;       /* turn 1st led on */
        _delay_ms(16);     /* 16ms delay */
        PORTA = 0x02;       /* turn 2nd led on */
        _delay_ms(14);     /* delay again */
        PORTA = 0x04;       /* turn 3d led on */
        _delay_ms(12);     /* delay again */
        PORTA = 0x08;       /* turn 4th led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0x10;       /* turn 5th led on */
        _delay_ms(8);     /* delay again */
        PORTA = 0x20;       /* turn 6th led on */
        _delay_ms(6);     /* delay again */
        PORTA = 0x40;       /* turn 7th led on */
        _delay_ms(4);     /* delay again */
        PORTA = 0x80;       /* turn 8th led on */
        _delay_ms(2);     /* delay again */
    }
}
