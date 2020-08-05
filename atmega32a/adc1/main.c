/*
 * @date	28/07/2020 14:29
 */


#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>


double V_REF = 2560;
double ADC_CONVERSION_FACTOR = 512;
double ADC_GAIN = 1.0;

static void ports_init(void) {

	// Set all pins of port C as output, and set it to low
    DDRC = 0xFF;
    PORTC = 0x00;   /* set LOW all output port pins */

	// Set all pins of port D as output, and set it to low
    DDRD = 0xFF;
    PORTD = 0x00;
}
/**
static void adc_init(void) {

	// Set ADMUX to 1101 0000
	// Internal 2.56V Voltage Reference with external capacitor at AREF pin, output left adjusted, ADC0 - ADC1 with gain=1 (ADC0 - positive input, ADC1 - negative input)
    ADMUX = 0xD0;

	// Set ADCSRA to 1000 0100, set ADEN on and prescaler to 16 (1 Mhz / 16 = 62.5 kHz)
    ADCSRA = 0x84;
}**/

static void adc_single_mode_init(void) {

	// Internal voltage reference 2560 mV with external capacitor acorcc AREF, left justified result, ADC0 single ended input
	ADMUX = 0xE0;

	// Set ADCSRA to 1000 0100, set ADEN on and prescaler to 16 (1 Mhz / 16 = 62.5 kHz)
    ADCSRA = 0x84;

	//ADCSRA = (1 << ADPS2);   /* set prescaler mode to 16: 1MHz/16 = 62.5kHz so it fits into 50kHz - 200kHz suggested in datasheet */
    //ADCSRA |= (1 << ADEN); /* enable ADC */

	//TCCR1B |= ((1 << CS11) | (1 << CS10));   /* use 16 bit timer, set timer prescaler to 64 mode, 1 tick = 1 / (F_CPU/64) s => 1 tick = (1/15625)s = 0.000064s, 15625 ticks/s */
    //TCNT1 = 0; /* reset/start */
}

static uint16_t adc_single_mode_read(uint8_t channel) {

	uint16_t adc_val = 0;

    channel &= 0x07;
    ADMUX = (ADMUX & 0xF8) | channel;
    ADCSRA |= (1 << ADSC);
    while (!(ADCSRA & (1 << ADIF)));

	adc_val = (ADC);


		if (adc_val > 100) {
            PORTD |= (1 << PD0);
		} else {
            PORTD &= ~(1 << PD0);
		}
		if (adc_val > 300) {
            PORTD |= (1 << PD1);
		} else {
            PORTD &= ~(1 << PD1);
		}
		if (adc_val > 600) {
            PORTD |= (1 << PD2);
		} else {
            PORTD &= ~(1 << PD2);
		}
		if (adc_val > 800) {
            PORTD |= (1 << PD3);
		} else {
            PORTD &= ~(1 << PD3);
		}
		if (adc_val > 1000) {
            PORTD |= (1 << PD4);
		} else {
            PORTD &= ~(1 << PD4);
			PORTC |= (1 << PC0);
		}
		if (adc_val >= 1024) {
            PORTD |= (1 << PD5);
		} else {
            PORTD &= ~(1 << PD5);
		}

    return adc_val;
}
/**
static int16_t adc_read(void) {

	uint8_t positive = 0;
	uint16_t adc_val = 0;
	int16_t res = 0;

	// Start ADC conversion and wait until it completes
    ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADIF)));

	// flash LED if result is positive
	positive = !(ADCH & (1 << 1));
	if (positive) {
		PORTC |= (1 << PC0);
	} else {
		PORTC &= ~(1 << PC0);
	}

	adc_val = ADCL;
	//adc_val |= (ADCH << 8);

	if (!positive) {

		// Convert from 2's complement
		//adc_val = ~adc_val;
		//adc_val += 1;
		//adc_val = adc_val & 0x03FF;
		//res = 0 - adc_val;
		res = 0;

	} else {

		if (adc_val > 100) {
            PORTD |= (1 << PD0);
		} else {
            PORTD &= ~(1 << PD0);
		}
		if (adc_val > 200) {
            PORTD |= (1 << PD1);
		} else {
            PORTD &= ~(1 << PD1);
		}
		if (adc_val > 300) {
            PORTD |= (1 << PD2);
		} else {
            PORTD &= ~(1 << PD2);
		}
		if (adc_val > 400) {
            PORTD |= (1 << PD3);
		} else {
            PORTD &= ~(1 << PD3);
		}
		if (adc_val > 500) {
            PORTD |= (1 << PD4);
		} else {
            PORTD &= ~(1 << PD4);
		}
		if (adc_val >= 511) {
            PORTD |= (1 << PD5);
		} else {
            PORTD &= ~(1 << PD5);
		}
		res = 0x1FF;
	}

	return res;
}**/

static double v_from_adc(int16_t adc_code) {

	(void) adc_code;
    //return adc_code * V_REF / (ADC_CONVERSION_FACTOR * ADC_GAIN);
	return 2500;
}


int
main(void) {

	double v = 0;

    ports_init();
    adc_single_mode_init();

    while(1) {

		// TEST
		//PORTC |= (1 << PC0);

		v = v_from_adc(adc_single_mode_read(0));
		v = 2500;

        if (v < 1500.0) {
            PORTD |= (1 << PD6);
        } else {
            PORTD &= ~(1 << PD6);
        }
        //_delay_loop_2(0);               /* 16 bit counter, 0 means do max loops, i.e. 65536, each of them takes 4 CPU cycles, so if F_CPU is 1MHz this does busy waiting for 261.1 milliseconds */
		//_delay_ms(20);
    }
}
