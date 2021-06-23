#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#include "lcd.h"

int main(void)
{
	lcd_configuration_t  lcd_config = {
		.lcd_port_rs = { &DDRC, &PORTC, &PINC, 0 },
		.lcd_port_rw = { &DDRC, &PORTC, &PINC, 1 },
		.lcd_port_en = { &DDRC, &PORTC, &PINC, 2 },
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
	lcd_display("OK\n");
}
