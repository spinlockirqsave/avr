/*
 * @file    main.c
 * @brief   Photocell 1. Read the light sensor signal through ADC,
 *          flash LEDs accordingly to digitized analog input signal.
 * @details Compile with 'make flash', i.e. JTAG enabled.
 *          One needs to calibrate accordingly to sensor's spectral response.
 *          This is wriiten for SEN 09088 mini photocell (GL5528):
 *          - light resistance at 10Lux(at 25*C): 8-20 KOhm
 *          - dark resistance at 0 Lux: 1.0 MOhm (min)
 *          - gamma value at 100-10 Lux: 0.7
 *          - power dissipation (at 25*C): 100 mW
 *          - max voltage (at 25*C): 150 V
 *          - spectral response peak (at 25*C): 540 nm
 *          - ambient temperature range: -30 ~ +70 *C 
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
 *     R1 (6.67 kOhm used)
 *     -----Vout (to ADC port 0)
 *     R2 (photocell)
 *     gnd
 *******************************
 *
 * Vout = Vin * R2 / (R1 + R2)
 */


uint8_t V_REF = 5;  /* 5V reference */

/* @brief   Initialize ADC */
void adc_init(void) {
    ADMUX = 1 << REFS0; /* set up voltage reference to AVCC */
    ADCSRA = (1 << ADPS2);   /* set prescaler mode to 16: 1MHz/16 = 62.5kHz so it fits into 50kHz - 200kHz suggested in datasheet */
    ADCSRA |= (1 << ADEN); /* enable ADC */
}

/* @brief   Return ADC sample. */
uint16_t adc_read(uint8_t channel) {
    channel &= 0x07;    /* assert channel is between 0-7 inclusive */
    ADMUX = (ADMUX & 0xF8) | channel;   /* set multiplexer, clear bottom 3 bits before ORing */
    ADCSRA |= (1 << ADSC);  /* start single conversion */
    while (!(ADCSRA & (1 << ADIF)));    /* allow conversion to complete */
    return (ADC);
}

/* @brief   Initialize port pins to be output direction. */
void ports_init(void) {
    DDRC = 0xFF;    /* set all PORTC to OUTPUT */
    PORTC = 0x00;   /* set LOW all output port pins */
}

/* @brief   ADC reading from voltage.
 * @details Voltage in mV for proper arithmetic without truncation. */
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

    ports_init();
    adc_init();

    while(1) {
        adc_sample = adc_read(0);
        if (adc_sample > 900) {
            PORTC = 0b11111111;         /* set all HIGH */
        } else if (adc_sample > 900) {
            PORTC = 0b01111111;
        } else if (adc_sample > 840) {
            PORTC = 0b00111111;
        } else if (adc_sample > 780) {
            PORTC = 0b00011111;
        } else if (adc_sample > 740) {
            PORTC = 0b00001111;
        } else if (adc_sample > 680) {
            PORTC = 0b00000111;
        } else if (adc_sample > 620) {
            PORTC = 0b00000011;
        } else if (adc_sample > 560) {
            PORTC = 0b00000001;
        } else {
            PORTC = 0x00;               /* set all LOW */
        }
    }
}
