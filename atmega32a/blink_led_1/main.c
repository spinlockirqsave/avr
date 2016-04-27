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
    DDRA = 0xFF;            /* Nakes PORTA as output,
                             * mark all 8 pins as output  */

    while (1)
    {
        PORTA = 0x01;       /* turn 1st led on */
        _delay_ms(500);     /* 0.5s delay */
        PORTA = 0x02;       /* turn 2nd led on */
        _delay_ms(500);     /* delay again */
        PORTA = 0x04;       /* turn 3d led on */
        _delay_ms(500);     /* delay again */
        PORTA = 0x08;       /* turn 4th led on */
        _delay_ms(500);     /* delay again */
        PORTA = 0x10;       /* turn 5th led on */
        _delay_ms(500);     /* delay again */
        PORTA = 0x20;       /* turn 6th led on */
        _delay_ms(500);     /* delay again */
        PORTA = 0x40;       /* turn 7th led on */
        _delay_ms(500);     /* delay again */
        PORTA = 0x80;       /* turn 8th led on */
        _delay_ms(500);     /* delay again */
    }
}
