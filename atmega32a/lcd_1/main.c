//#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "lcd.h"


uint8_t seconds = 0;

ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
	TCNT1 = 63974;   // for 1 sec at 16 MHz

	lcd_exec_instruction_clear_display();		// clear display RAM
	_delay_ms(4);                               // 1.64 mS delay (min)

	seconds++;
	lcd_printf("%u", seconds);
}

int main(void)
{
	lcd_configuration_t  lcd_config = {
		.lcd_port_rs = { &DDRC, &PORTC, &PINC, 5 },
		.lcd_port_rw = { &DDRC, &PORTC, &PINC, 6 },
		.lcd_port_en = { &DDRC, &PORTC, &PINC, 7 },
		.lcd_port_d0 = { &DDRD, &PORTD, &PIND, 0 },
		.lcd_port_d1 = { &DDRD, &PORTD, &PIND, 1 },
		.lcd_port_d2 = { &DDRD, &PORTD, &PIND, 2 },
		.lcd_port_d3 = { &DDRD, &PORTD, &PIND, 3 },
		.lcd_port_d4 = { &DDRD, &PORTD, &PIND, 4 },
		.lcd_port_d5 = { &DDRD, &PORTD, &PIND, 5 },
		.lcd_port_d6 = { &DDRD, &PORTD, &PIND, 6 },
		.lcd_port_d7 = { &DDRD, &PORTD, &PIND, 7 },
		.want_8_bit_mode = 1,
		.want_display_2_lines = 1,
		.want_dotes_5x10 = 0,
	};

	lcd_init(&lcd_config);

	TCNT1 = 63974;							// for 1 sec at 16 MHz
	TCCR1A = 0x00;
	TCCR1B = (1 << CS10) | (1 << CS12);		// Timer mode with 1024 prescaler
	TIMSK = (1 << TOIE1) ;					// Enable timer1 overflow interrupt(TOIE1)
	sei();									// Enable global interrupts by setting global interrupt enable bit in SREG


	while (1) {
	}
}
