/*
 * @file    main.c
 * @brief   Photocell 1. Read the light sensor signal through ADC,
 *          flash LEDs accordingly to digitized analog input signal.
 * @date    27 Aug 2016 05:50 PM
 */


#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 1000000UL    /* 1 MHz clock speed */
#endif

#include <avr/io.h>
#include <util/delay.h>

/* Voltage divider:
 *
 ********************************
 * Vin --
 *     R1
 *     -----Vout
 *     R2
 *     gnd
 *******************************
 *
 * Vout = Vin * R2 / (R1 + R2)
 */


uint8_t V_REF = 5;  /* 5V reference */

/* Initialize ADC */
void adc_init(void) {
    ADMUX = 1 << REFS0; /* set up voltage reference to AVCC */
    ADCSRA = (1 << ADEN); /* enable ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   /* set prescaler mode to 128 (min resolution) */
}

/* @brief   Return ADC sample. */
uint16_t adc_read(uint8_t channel) {
    channel &= 0x07;    /* assert channel is between 0-7 inclusive */
    ADMUX |= channel;   /* set multiplexer */
    ADCSRA |= (1 << ADSC);  /* start single conversion */
    while (!(ADCSRA & (1 << ADIF)));    /* allow conversion to complete, wait for ADIF flag set */
    ADCSRA |= (1 << ADIF);  /* clear bit, by writing 1 to it */
    return ADC;
}

/* @brief   Initialize port pins to be output direction. */
void ports_init(void) {
    DDRC = 0xFF;    /* set all PORTC to OUTPUT */
    PORTC = 0x00;   /* set LOW all output port pins */
}

/* @brief   ADC reading from voltage.
 * @detaild Voltage in mV for proper arithmetic without truncation. */
uint16_t adc_from_v(uint16_t v) {
    uint32_t tmp = v << 10;  /* V * 1024 */
    return ((tmp / (V_REF * 1000)) & 0xFFFF);
}

/* @brief   Voltage reading from ADC sample.
 * @details Result in mV. */
uint16_t v_from_adc(uint16_t adc) {
    uint32_t tmp = adc * V_REF *1000;
    return ((tmp >> 10) & 0xFFFF);
}


int
main(void) {
    uint16_t adc_sample;

    adc_init();
    while(1) {
        adc_sample = adc_read(1);
        if (adc_sample < adc_from_v(4600)) {
            PORTC |= (1 << PC0);       /* set HIGH */
        } else {
            PORTC |= (0 << PC0);       /* set LOW */
        }
    }
}
