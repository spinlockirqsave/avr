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
    DDRC = 0xFF;            /* Nakes PORTC as output */

    while (1)
    {
        PORTC = 0xFF;       /* turn all leds on */
        _delay_ms(500);     /* 0.5s delay */
        PORTB = 0x00;       /* all leds off */
        _delay_ms(500);     /* delay again */
    }
}
