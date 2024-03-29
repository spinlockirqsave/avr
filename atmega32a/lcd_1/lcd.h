#ifndef LCD_BCB1602_H
#define LCD_BCB1602_H

/**
 * This is a driver for LCD devices based on Hitachi HD44780 chips,
 * like e.g. Blaze BCB1602-03x series.
 *
 * The driver can work with any pin configuration, all you need to do to make things work is:
 * - connect pins
 * - tell the driver about your selected pinout by passing proper configuration to lcd_init()
 *
 * To use this driver:
 *  - select your pins for LCD on microcontroller. 3 control pins are needed,
 *  and 4 or 8 data pins (depending if using LCD in 4 or 8 bit data bus mode)
 *  - connect them to LCD
 *  - include "lcd.h" in your project
 *  - define an instance of lcd_configuration_t, here you can pass your options (chosen data bus mode, 
 *  1 or 2 lines displayed on LCD, extended font) and tell the driver about selected pinout
 *  - call lcd_init() with your config as argument
 *  - call lcd_display_1st_line(), lcd_display_2nd_line(), lcd_display or lcd_printf(fmt, args) to display text on LCD
 *
 *  Example 4 bit mode connection and corresponding config:
 *
 *                -----------                   ----------
 *               | ATmega32  |                 |   LCD    |
 *               |           |                 |          |
 *               |        PD7|---------------->|D7        |
 *               |        PD6|---------------->|D6        |
 *               |        PD5|---------------->|D5        |
 *               |        PD4|---------------->|D4        |
 *               |        PD3|                 |D3        |
 *               |        PD2|                 |D2        |
 *               |        PD1|                 |D1        |
 *               |        PD0|                 |D0        |
 *               |           |                 |          |
 *               |        PC7|---------------->|E         |
 *               |        PC6|---------------->|RW        |
 *               |        PC5|---------------->|RS        |
 *                -----------                   ----------
 *
 *	lcd_configuration_t  lcd_config = {
 *		.lcd_port_rs = { &DDRC, &PORTC, &PINC, 5 },
 *		.lcd_port_rw = { &DDRC, &PORTC, &PINC, 6 },
 *		.lcd_port_en = { &DDRC, &PORTC, &PINC, 7 },
 *		.lcd_port_d0 = { 0 },
 *		.lcd_port_d1 = { 0 },
 *		.lcd_port_d2 = { 0 },
 *		.lcd_port_d3 = { 0 },
 *		.lcd_port_d4 = { &DDRD, &PORTD, &PIND, 4 },
 *		.lcd_port_d5 = { &DDRD, &PORTD, &PIND, 5 },
 *		.lcd_port_d6 = { &DDRD, &PORTD, &PIND, 6 },
 *		.lcd_port_d7 = { &DDRD, &PORTD, &PIND, 7 },
 *		.want_8_bit_mode = 0,
 *		.want_display_2_lines = 1,
 *		.want_dotes_5x10 = 0,
 *	};
 * 
 *  Example 8 bit mode connection and corresponding config:
 *
 *                -----------                   ----------
 *               | ATmega32  |                 |   LCD    |
 *               |           |                 |          |
 *               |        PD7|---------------->|D7        |
 *               |        PD6|---------------->|D6        |
 *               |        PD5|---------------->|D5        |
 *               |        PD4|---------------->|D4        |
 *               |        PD3|---------------->|D3        |
 *               |        PD2|---------------->|D2        |
 *               |        PD1|---------------->|D1        |
 *               |        PD0|---------------->|D0        |
 *               |           |                 |          |
 *               |        PC7|---------------->|E         |
 *               |        PC6|---------------->|RW        |
 *               |        PC5|---------------->|RS        |
 *                -----------                   ----------
 *
 *	lcd_configuration_t  lcd_config = {
 *		.lcd_port_rs = { &DDRC, &PORTC, &PINC, 5 },
 *		.lcd_port_rw = { &DDRC, &PORTC, &PINC, 6 },
 *		.lcd_port_en = { &DDRC, &PORTC, &PINC, 7 },
 *		.lcd_port_d0 = { &DDRD, &PORTD, &PIND, 0 },
 *		.lcd_port_d1 = { &DDRD, &PORTD, &PIND, 1 },
 *		.lcd_port_d2 = { &DDRD, &PORTD, &PIND, 2 },
 *		.lcd_port_d3 = { &DDRD, &PORTD, &PIND, 3 },
 *		.lcd_port_d4 = { &DDRD, &PORTD, &PIND, 4 },
 *		.lcd_port_d5 = { &DDRD, &PORTD, &PIND, 5 },
 *		.lcd_port_d6 = { &DDRD, &PORTD, &PIND, 6 },
 *		.lcd_port_d7 = { &DDRD, &PORTD, &PIND, 7 },
 *		.want_8_bit_mode = 1,
 *		.want_display_2_lines = 1,
 *		.want_dotes_5x10 = 0,
 *	};
 *
 * Program example:
 *
 * lcd_init(&lcd_config);
 *
 * lcd_display_1st_line("Hello");
 * lcd_display_2nd_line("LCD");
 *
 * Author: Piotr Gregor piotr@dataandsignal.com, 2021
 */

#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>

#define LCD_COL_COUNT 16
#define LCD_ROW_COUNT 2
#define LCD_DDRAM_LINE_ONE_ADDRESS 0x00
#define LCD_DDRAM_LINE_TWO_ADDRESS 0x40


typedef struct lcd_port_s {
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	volatile uint8_t *pin;
	uint8_t bit;
} lcd_port_t;

typedef struct lcd_configuration_s {
	lcd_port_t lcd_port_rs;
	lcd_port_t lcd_port_rw;
	lcd_port_t lcd_port_en;
	lcd_port_t lcd_port_d0;
	lcd_port_t lcd_port_d1;
	lcd_port_t lcd_port_d2;
	lcd_port_t lcd_port_d3;
	lcd_port_t lcd_port_d4;
	lcd_port_t lcd_port_d5;
	lcd_port_t lcd_port_d6;
	lcd_port_t lcd_port_d7;
	uint8_t want_8_bit_mode;
	uint8_t want_display_2_lines;
	uint8_t want_dotes_5x10;
} lcd_configuration_t;

/*
 * Initialiase LCD.
 */
void lcd_init(lcd_configuration_t *config);

/*
 * Display string on LCD.
 * Start on 1st line, if string doesn't fit (it's more than 16 chars) wrap it up and start from 2nd line (if 2 lines are used).
 */
void lcd_display(char *str);

/*
 * Display string on LCD.
 * Start on 1st line, if string doesn't fit (it's more than 16 chars) ignore remaining chars and return.
 */
void lcd_display_1st_line(char *str);

/*
 * Display string on LCD.
 * Start on 2nd line, if string doesn't fit (it's more than 16 chars) ignore remaining chars and return.
 */
void lcd_display_2nd_line(char *str);

/*
 * Display string on LCD, string is built from paramters and format.
 */
void lcd_printf(char *format, ...);

void lcd_configure_control_ports_as_output(lcd_configuration_t *c);
void lcd_configure_data_7_4_ports_as_output(lcd_configuration_t *c);
void lcd_configure_data_7_4_ports_as_input(lcd_configuration_t *c);
void lcd_configure_data_3_0_ports_as_output(lcd_configuration_t *c);
void lcd_configure_data_3_0_ports_as_input(lcd_configuration_t *c);
void lcd_configure_data_ports_as_output(lcd_configuration_t *c);
void lcd_configure_data_ports_as_input(lcd_configuration_t *c);
void lcd_transmit_4(uint8_t nibble);
void lcd_transmit_8(uint8_t word);
uint8_t lcd_read_8(void);

/*
 * Transmit lower nibble assuming 4 bit bus mode (single transfer on D7-D4).
 */
void lcd_transmit_4(uint8_t nibble);

/*
 * Transmit whole byte (if in 8 bit bus mode - single transfer using D7-D0, if in 4 bit bus mode - 2 transfers using D7-D4).
 */
void lcd_transmit_8(uint8_t word);

/*
 * Write '20H' to DDRAM and set DDRAM address to '00H' from Address Counter.
 */
void lcd_exec_instruction_clear_display(void);

/*
 * Set DDRAM address to '00H' from Address Counter and return cursor to it's original position if shifted.
 * The content of DDRAM is not changed.
 */
void lcd_exec_instruction_return_home(void);

/*
 * Assign cursor moving direction and enable the shift of entire display.
 */
void lcd_exec_instruction_entry_mode_set(uint8_t increment, uint8_t left_2_right);

/*
 * Turn on/off display, cursor and blinking of the cursor.
 */
void lcd_exec_instruction_display_control(uint8_t display_on, uint8_t cursor_on, uint8_t cursor_blinking_on);

/*
 * Move cursor left or right.
 */
void lcd_exec_instruction_cursor_shift(uint8_t left);

/*
 * Move display left or right.
 */
void lcd_exec_instruction_display_shift(uint8_t left);

/*
 * Set interface data length, number of lines to display and font.
 * @interface_data_length_8bit - 0 for 4 bit, 1 for 8 bit
 * @display_2_lines - 0 for 1 line, 1 for 2 lines
 * @dotes_5x10 - font type, 0 for 5x8, 1 for 5x10
 */
void lcd_exec_instruction_function_set(uint8_t interface_data_length_8bit, uint8_t display_2_lines, uint8_t dotes_5x10);

/*
 * Reset LCD to initial, known state of 4 bit mode.
 */
void lcd_exec_instruction_function_reset(uint8_t interface_data_length_8bit);

/*
 * Set CGRAM address in address counter.
 */
void lcd_exec_instruction_set_cgram_address(uint8_t address);

/*
 * Set DDRAM address in address counter.
 */
void lcd_exec_instruction_set_ddram_address(uint8_t address);

/*
 * Read busy flag. Return 1 if busy flag is set, 0 otherwise.
 */
uint8_t lcd_exec_instruction_read_busy_flag(void);

/*
 * Read address counter.
 */
uint8_t lcd_exec_instruction_read_address_counter(void);

/*
 * Read busy flag and address counter. Return 1 if busy_flag is set, 0 otherwise.
 */
uint8_t lcd_exec_instruction_read_busy_flag_and_address_counter(uint8_t *busy_flag, uint8_t *address);

/*
 * Write data into internal RAM (DDRAM/CGRAM).
 */
void lcd_exec_instruction_write_data(uint8_t data);

/*
 * Read data from internal RAM (DDRAM/CGRAM).
 */
uint8_t lcd_exec_instruction_read_data(void);

#endif // define LCD_BCB1602_H
