/*
 * @file    main.c
 * @brief   Blink led using PORTA 0.
 * @date    27 Apr 2016 01:47 PM
 */


#include <avr/io.h>
#include <util/delay.h>


#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 16000000UL    /* 16 MHz clock speed */
#endif

int
main(void)
{
    DDRA |= (1<<PA0);            /* Nakes PORTA 0 as output */

    while (1)
    {
        PORTA |= (1<<PA0);          /* turn 1st led on */
        _delay_ms(10);              /* 0.01s delay */
        PORTA &= ~(1<<PA0);         /* turn led off */
        _delay_ms(10);              /* delay 0.05s */
    }
}
