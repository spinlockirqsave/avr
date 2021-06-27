//#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "lcd.h"


uint8_t	seconds = 0;
uint8_t	minutes = 0;
uint8_t	hours = 0;
char	time_buf[16] = { 0 };

static void time_2_str(char *buf, uint8_t buflen)
{
	snprintf(buf, buflen, "%u:%u:%u", hours, minutes, seconds);
}

static void timer_tick(void)
{
	seconds++;
	if (seconds == 60) {
		minutes++;
		if (minutes == 60) {
			hours++;
			minutes = 0;
		}
		seconds = 0;
	}
}

ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
	TCNT1 = 63974;   // for 1 sec at 16 MHz

	lcd_exec_instruction_clear_display();		// clear display RAM
	_delay_ms(4);                               // 1.64 mS delay (min)

	timer_tick();
	time_2_str(time_buf, sizeof(time_buf));
	lcd_printf("%s", time_buf);
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
		.want_dotes_5x10 = 1,
	};

	lcd_init(&lcd_config);

	lcd_display_1st_line("Timer: MiSW 2021");
	lcd_display_2nd_line("Piotr Gregor");
	int i = 0;
	while (i < 500) {
		_delay_ms(10);
		++i;
	}

	TCNT1 = 63974;							// for 1 sec at 16 MHz
	TCCR1A = 0x00;
	TCCR1B = (1 << CS10) | (1 << CS12);		// Timer mode with 1024 prescaler
	TIMSK = (1 << TOIE1) ;					// Enable timer1 overflow interrupt(TOIE1)
	sei();									// Enable global interrupts by setting global interrupt enable bit in SREG

	// toggle the diode every 1s
	DDRC |= (1 << DDRC4);
	while (1) {
		int i = 0;
		while (i < 100) {
			_delay_ms(10);
			++i;
		}
		PORTC = (PORTC ^ (1 << PORTC4));
	}
}
