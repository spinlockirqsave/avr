#include "lcd.h"

/**
 * Author: Piotr Gregor piotr@dataandsignal.com, 2021
 */

struct lcd_configuration_s  lcd_default_config = {
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

lcd_configuration_t *lcd_config = &lcd_default_config;


/*
 * To send a command to LCD:
 * 1. Put command word on the data bus
 * 2. Configure control port to command tx
 * 3. Execute command by latching it to LCD on falling edge of LCD_EN
 *
 * To send a data to LCD:
 * 1. Put data word on the data bus
 * 2. Configure control port to data tx
 * 3. Execute command by latching it to LCD on falling edge of LCD_EN
 */

void lcd_put_word_on_databus(uint8_t word);
void lcd_signal_command_tx();
void lcd_signal_command_rx();
void lcd_signal_data_tx();
void lcd_signal_data_rx();
void lcd_latch();

uint8_t lcd_read_port(lcd_port_t *p);
void lcd_set_port_low(lcd_port_t *p);
void lcd_set_port_high(lcd_port_t *p);
void lcd_set_port(lcd_port_t *p, uint8_t value);
void lcd_configure_port_as_input(lcd_port_t *p);
void lcd_configure_port_as_output(lcd_port_t *p);


void lcd_init(lcd_configuration_t *config)
{
	uint8_t bit_mode_8 = 1;

	if (config) {
		lcd_config = config;
	}

	_delay_ms(100);

	// Configure control port as output
	lcd_configure_control_ports_as_output(lcd_config);

	// Configure data port as output
	lcd_configure_port_as_output(&lcd_config->lcd_port_d7);
	lcd_configure_port_as_output(&lcd_config->lcd_port_d6);
	lcd_configure_port_as_output(&lcd_config->lcd_port_d5);
	lcd_configure_port_as_output(&lcd_config->lcd_port_d4);

	if (config->want_8_bit_mode) {
		lcd_configure_port_as_output(&lcd_config->lcd_port_d3);
		lcd_configure_port_as_output(&lcd_config->lcd_port_d2);
		lcd_configure_port_as_output(&lcd_config->lcd_port_d1);
		lcd_configure_port_as_output(&lcd_config->lcd_port_d0);
	}

	/*
	 * LCD may be in unknown state. Before it can be configured to the full set of desired options
	 * it must be configured to the right bit mode (4/8). Writing same command to set desired bit mode
	 * 3 times guarantees LCD will be put into 8 bit mode. Execute commands as single transfers to D7-D4.
	 */
	bit_mode_8 = 1;
	lcd_exec_instruction_function_reset(bit_mode_8);
	_delay_ms(10); 
	lcd_exec_instruction_function_reset(bit_mode_8);
	_delay_us(200); 
	lcd_exec_instruction_function_reset(bit_mode_8);
	_delay_us(200);

	/*
	 * The bus interface is now in 8 bit mode, 1 line, 5×8 characters.
	 * If a different configuration 8 bit mode is desired an 8 bit bus Function Set command should be sent to set the full parameters.
	 * If 4 bit mode is desired 0b0010 should be sent on D7-D4 with a single enable pulse. Now the controller will be in 4 bit mode
	 * and a full 4 bit bus Function Set command sequence (two enables with command bits 7-4 and 3-0 on subsequent cycles)
	 * will complete the configuration of the Function Set register.
	 */
	if (lcd_config->want_8_bit_mode) {

		// Set  complete options for desired 8 bit mode
		bit_mode_8 = 1;
		lcd_exec_instruction_function_set(bit_mode_8, lcd_config->want_display_2_lines, lcd_config->want_dotes_5x10);

		// Configuration of Function Set register is completed.

	} else {

		// Set  options for desired 4 bit mode

		// First, execute single cycle (4 bit bus) Function Set command to enter 4 bit mode 
		bit_mode_8 = 0;
		lcd_exec_instruction_function_reset(bit_mode_8);
		_delay_us(200); 

		// Now LCD is in 4 bit mode. Execute complete Function Set command with standard transmit of 8 bits in 2 write cycles, setting all D7-D0 LCD bits.
		lcd_exec_instruction_function_set(bit_mode_8, lcd_config->want_display_2_lines, lcd_config->want_dotes_5x10);

		// Configuration of Function Set register is completed.
	}

	// Preliminary Function Set instruction - used only to set the 4-bit mode.
// The number of lines or the font cannot be set at this time since the controller is still in the
//  8-bit mode, but the data transfer mode can be changed since this parameter is determined by one 
//  of the upper four bits of the instruction.
 
    _delay_us(80);                                  // 40uS delay (min)

// The next three instructions are specified in the data sheet as part of the initialization routine, 
//  so it is a good idea (but probably not necessary) to do them just as specified and then redo them 
//  later if the application requires a different configuration.

// Display On/Off Control instruction
    lcd_exec_instruction_display_control(0, 0, 0);        // turn display OFF
    _delay_us(80);                                  // 40uS delay (min)

// Clear Display instruction
	lcd_exec_instruction_clear_display();		// clear display RAM
    _delay_ms(4);                                   // 1.64 mS delay (min)

// ; Entry Mode Set instruction
	lcd_exec_instruction_entry_mode_set(1, 0);
    _delay_us(80);                                  // 40uS delay (min)

// This is the end of the LCD controller initialization as specified in the data sheet, but the display
//  has been left in the OFF condition.  This is a good time to turn the display back ON.
 
// Display On/Off Control instruction
    lcd_exec_instruction_display_control(1, 1, 1);        // turn display ON
    _delay_us(80);                                  // 40uS delay (min)
}

void lcd_display(char *str)
{
	uint8_t n = 0;

	lcd_exec_instruction_set_ddram_address(LCD_DDRAM_LINE_ONE_ADDRESS);

	for (char *c = str; c && *c != '\0' && n < LCD_COL_COUNT; c++, n++) {
		if (n == LCD_COL_COUNT) {
			lcd_exec_instruction_set_ddram_address(LCD_DDRAM_LINE_TWO_ADDRESS);
		}
		lcd_exec_instruction_write_data(*c);
		_delay_us(80);                              // 40 uS delay (min)
	}
}

void lcd_display_1st_line(char *str)
{
	uint8_t n = 0;

	lcd_exec_instruction_set_ddram_address(LCD_DDRAM_LINE_ONE_ADDRESS);

	for (char *c = str; c && *c != '\0' && n < LCD_COL_COUNT; c++, n++) {
		lcd_exec_instruction_write_data(*c);
		_delay_us(80);                              // 40 uS delay (min)
	}
}

void lcd_display_2nd_line(char *str)
{
	uint8_t n = 0;

	lcd_exec_instruction_set_ddram_address(LCD_DDRAM_LINE_TWO_ADDRESS);

	for (char *c = str; c && *c != '\0' && n < LCD_COL_COUNT; c++, n++) {
		lcd_exec_instruction_write_data(*c);
		_delay_us(80);                              // 40 uS delay (min)
	}
}

void lcd_printf(char *fmt, ...)
{
	va_list args;
	char lcd_buf[LCD_COL_COUNT + 1] = { 0 };

	va_start(args, fmt);
	vsnprintf(lcd_buf, sizeof(lcd_buf), fmt, args);
	va_end(args);

	lcd_display(lcd_buf);
}

void lcd_configure_control_ports_as_output(lcd_configuration_t *c)
{
	lcd_configure_port_as_output(&c->lcd_port_rs);
	lcd_configure_port_as_output(&c->lcd_port_rw);
	lcd_configure_port_as_output(&c->lcd_port_en);
}

void lcd_configure_data_7_4_ports_as_output(lcd_configuration_t *c)
{
	lcd_configure_port_as_output(&c->lcd_port_d7);
	lcd_configure_port_as_output(&c->lcd_port_d6);
	lcd_configure_port_as_output(&c->lcd_port_d5);
	lcd_configure_port_as_output(&c->lcd_port_d4);
}

void lcd_configure_data_7_4_ports_as_input(lcd_configuration_t *c)
{
	lcd_configure_port_as_input(&c->lcd_port_d7);
	lcd_configure_port_as_input(&c->lcd_port_d6);
	lcd_configure_port_as_input(&c->lcd_port_d5);
	lcd_configure_port_as_input(&c->lcd_port_d4);
}

void lcd_configure_data_3_0_ports_as_output(lcd_configuration_t *c)
{
	lcd_configure_port_as_output(&c->lcd_port_d3);
	lcd_configure_port_as_output(&c->lcd_port_d2);
	lcd_configure_port_as_output(&c->lcd_port_d1);
	lcd_configure_port_as_output(&c->lcd_port_d0);
}

void lcd_configure_data_3_0_ports_as_input(lcd_configuration_t *c)
{
	lcd_configure_port_as_input(&c->lcd_port_d3);
	lcd_configure_port_as_input(&c->lcd_port_d2);
	lcd_configure_port_as_input(&c->lcd_port_d1);
	lcd_configure_port_as_input(&c->lcd_port_d0);
}

void lcd_configure_data_ports_as_output(lcd_configuration_t *c)
{
	lcd_configure_data_7_4_ports_as_output(c);
	if (c->want_8_bit_mode) {
		lcd_configure_data_3_0_ports_as_output(c);
	}
}

void lcd_configure_data_ports_as_input(lcd_configuration_t *c)
{
	lcd_configure_data_7_4_ports_as_input(c);
	if (c->want_8_bit_mode) {
		lcd_configure_data_3_0_ports_as_input(c);
	}
}

void lcd_transmit_4(uint8_t nibble)
{
	lcd_configure_data_ports_as_output(lcd_config);

	lcd_set_port(&lcd_config->lcd_port_d4, (nibble & 0x01 ? 1 : 0));  
	lcd_set_port(&lcd_config->lcd_port_d5, (nibble & 0x02 ? 1 : 0));  
	lcd_set_port(&lcd_config->lcd_port_d6, (nibble & 0x04 ? 1 : 0));  
	lcd_set_port(&lcd_config->lcd_port_d7, (nibble & 0x08 ? 1 : 0));

	lcd_latch();
}

void lcd_transmit_8(uint8_t word)
{
	lcd_configure_data_ports_as_output(lcd_config);

	if (lcd_config->want_8_bit_mode) {

		// Use all 8 data ports

		lcd_set_port(&lcd_config->lcd_port_d0, (word & 0x01 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d1, (word & 0x02 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d2, (word & 0x04 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d3, (word & 0x08 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d4, (word & 0x10 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d5, (word & 0x20 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d6, (word & 0x40 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d7, (word & 0x80 ? 1 : 0));

		lcd_latch();

	} else {

		// Use only D7-D4 ports for data transfer

		// Transmit upper nibble
		lcd_set_port(&lcd_config->lcd_port_d4, (word & 0x10 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d5, (word & 0x20 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d6, (word & 0x40 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d7, (word & 0x80 ? 1 : 0));

		lcd_latch();

		// Transmit lower nibble
		lcd_set_port(&lcd_config->lcd_port_d4, (word & 0x01 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d5, (word & 0x02 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d6, (word & 0x04 ? 1 : 0));  
		lcd_set_port(&lcd_config->lcd_port_d7, (word & 0x08 ? 1 : 0));

		lcd_latch();
	}
}

uint8_t lcd_read_8(void)
{
	uint8_t data = 0;

	lcd_configure_data_ports_as_input(lcd_config);

	if (lcd_config->want_8_bit_mode) {

		// Use all 8 data ports

		lcd_latch();

		data |= (lcd_read_port(&lcd_config->lcd_port_d0) ? 1 : 0) << 0;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d1) ? 1 : 0) << 1;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d2) ? 1 : 0) << 2;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d3) ? 1 : 0) << 3;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d4) ? 1 : 0) << 4;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d5) ? 1 : 0) << 5;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d6) ? 1 : 0) << 6;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d7) ? 1 : 0) << 7;  

	} else {

		// Use only D7-D4 ports for data transfer

		// Read upper nibble
		lcd_latch();

		data |= (lcd_read_port(&lcd_config->lcd_port_d7) ? 1 : 0) << 7;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d6) ? 1 : 0) << 6;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d5) ? 1 : 0) << 5;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d4) ? 1 : 0) << 4;  

		// Read lower nibble
		lcd_latch();

		data |= (lcd_read_port(&lcd_config->lcd_port_d7) ? 1 : 0) << 3;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d6) ? 1 : 0) << 2;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d5) ? 1 : 0) << 1;  
		data |= (lcd_read_port(&lcd_config->lcd_port_d4) ? 1 : 0) << 0;  
	}

	return data;
}

void lcd_exec_instruction_clear_display(void)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x01);
}

void lcd_exec_instruction_return_home(void)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x02);
}

void lcd_exec_instruction_entry_mode_set(uint8_t increment, uint8_t left_2_right)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x04 | (increment ? 0x2 : 0) | (left_2_right ? 0x1 : 0));
}

void lcd_exec_instruction_display_control(uint8_t display_on, uint8_t cursor_on, uint8_t cursor_blinking_on)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x08 | (display_on ? 0x4 : 0) | (cursor_on ? 0x2 : 0) | (cursor_blinking_on ? 0x1 : 0));
}

void lcd_exec_instruction_cursor_shift(uint8_t left)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x10 | 0x00 | (left ? 0 : 0x04));
}

void lcd_exec_instruction_display_shift(uint8_t left)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x10 | 0x80 | (left ? 0 : 0x04));
}

void lcd_exec_instruction_function_set(uint8_t interface_data_length_8bit, uint8_t display_2_lines, uint8_t dotes_5x10)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x20 | (interface_data_length_8bit ? 0x10 : 0) | (display_2_lines ? 0x08 : 0) | (dotes_5x10 ? 0x04 : 0));
}

void lcd_exec_instruction_function_reset(uint8_t interface_data_length_8bit)
{
	lcd_signal_command_tx();
	lcd_transmit_4(0x02 | (interface_data_length_8bit ? 0x01 : 0));
}

void lcd_exec_instruction_set_cgram_address(uint8_t address)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x40 | (address & 0x3f));
}

void lcd_exec_instruction_set_ddram_address(uint8_t address)
{
	lcd_signal_command_tx();
	lcd_transmit_8(0x80 | (address & 0x7f));
}

uint8_t lcd_exec_instruction_read_busy_flag(void)
{
	uint8_t data = 0;

	lcd_signal_command_rx();
	data = lcd_read_8();
	lcd_latch();
	return (data & (1 << 7) ? 1 : 0); 
}

uint8_t lcd_exec_instruction_read_address_counter(void)
{
	uint8_t data = 0;

	lcd_signal_command_rx();
	data = lcd_read_8();
	return data & 0x7f;
}

uint8_t lcd_exec_instruction_read_busy_flag_and_address_counter(uint8_t *busy_flag, uint8_t *address)
{
	uint8_t bf = 0;
	uint8_t data = 0;

	lcd_signal_command_rx();
	data = lcd_read_8();
	*address = data & 0x7f;
	bf = (data & (1 << 7) ? 1 : 0); 
	*busy_flag = bf;
	return bf;
}

void lcd_exec_instruction_write_data(uint8_t data)
{
	lcd_signal_data_tx();
	lcd_transmit_8(data);
}

uint8_t lcd_exec_instruction_read_data(void)
{
	lcd_signal_data_rx();
	return lcd_read_8();
}

void lcd_signal_command_tx()
{
	lcd_set_port_low(&lcd_config->lcd_port_rs);		// command
	lcd_set_port_low(&lcd_config->lcd_port_rw);		// write
}

void lcd_signal_command_rx()
{
	lcd_set_port_low(&lcd_config->lcd_port_rs);		// command
	lcd_set_port_high(&lcd_config->lcd_port_rw);	// read
}

void lcd_signal_data_tx()
{
	lcd_set_port_high(&lcd_config->lcd_port_rs);	// data
	lcd_set_port_low(&lcd_config->lcd_port_rw);		// write
}

void lcd_signal_data_rx()
{
	lcd_set_port_high(&lcd_config->lcd_port_rs);	// data
	lcd_set_port_high(&lcd_config->lcd_port_rw);	// read
}

void lcd_latch()
{
	lcd_set_port_high(&lcd_config->lcd_port_en);
	_delay_us(1);
	lcd_set_port_low(&lcd_config->lcd_port_en);
	_delay_us(1);
}

uint8_t lcd_read_port(lcd_port_t *p)
{
    return ((*p->pin) & (1 << p->bit)) ? 1 : 0;
}

void lcd_set_port_low(lcd_port_t *p)
{
	(*p->port) &= ~(1 << p->bit);
}

void lcd_set_port_high(lcd_port_t *p)
{
	(*p->port) |= (1 << p->bit);
}

void lcd_set_port(lcd_port_t *p, uint8_t value)
{
	value ? lcd_set_port_high(p) : lcd_set_port_low(p);
}

void lcd_configure_port_as_input(lcd_port_t *p)
{
	(*p->ddr) &= ~(1 << p->bit);
}

void lcd_configure_port_as_output(lcd_port_t *p)
{
	(*p->ddr) |= (1 << p->bit);
}
