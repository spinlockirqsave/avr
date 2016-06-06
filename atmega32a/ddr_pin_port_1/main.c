/*
 * @file    main.c
 * @brief   Set PORTC I/O pins using data direction register,
 *          read input from pins and set high PORTC.0 2 and 3
 *          if input pin PORTC.6 is high.
 * @date    06 Jun 2016 10:28 PM
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
    DDRC = 0x0F;
    PORTC = 0x0C;
     
    /* lets assume a 4V supply comes to PORTC.6 and Vcc = 5V */
    if (PINC == 0b01000000) {
        PORTC = 0x0B;
        _delay_ms(1000);    /* delay 1s */
    } else {
        PORTC = 0x00;
    }
}