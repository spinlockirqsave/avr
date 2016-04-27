/*
 * @file    main.c
 * @brief   Blink leds.
 * @date    27 Apr 2016 01:47 PM
 */


#include <avr/io.h>
#include <util/delay.h>


#ifndef F_CPU
#define F_CPU 16000000UL    /* 16 MHz clock speed */
#endif

int
main(void)
{
    DDRA |= 0xFF;            /* Nakes PORTA as output,
                             * mark all 8 pins as output  */

    while (1)
    {
        PORTA = 0b00000001;       /* turn 1st led on */
        _delay_ms(10);     /* 0.5s delay */
        PORTA = 0b00000010;       /* turn 2nd led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0b00000100;       /* turn 3d led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0b00001000;       /* turn 4th led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0b00010000;       /* turn 5th led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0b00100000;       /* turn 6th led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0b01000000;       /* turn 7th led on */
        _delay_ms(10);     /* delay again */
        PORTA = 0b10000000;       /* turn 8th led on */
        _delay_ms(10);     /* delay again */
    }
}
