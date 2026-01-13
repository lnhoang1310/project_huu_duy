#include "lcd.h"
#include <stdio.h>

void lcd_send_cmd(LCD_HandleTypedef* lcd, char cmd){
	char upper_nibble, lower_nibble;
	uint8_t data_t[4];
	upper_nibble = (cmd & 0xF0); // get 4 bit upper
	lower_nibble = (cmd << 4) & 0xF0; // get 4 bit lower

	data_t[0] = upper_nibble | 0x0C; // en = 1, rw = 0, rs = 0
	data_t[1] = upper_nibble | 0x08; // en = 0, rw = 0, rs = 0
	data_t[2] = lower_nibble | 0x0C; // en = 1, rw = 0, rs = 0
	data_t[3] = lower_nibble | 0x08; // en = 0, rw = 0, rs = 0

	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, (uint8_t*)data_t, 4, 100);
}

void lcd_send_data(LCD_HandleTypedef* lcd, char data){
	char upper_nibble, lower_nibble;
	uint8_t data_t[4];
	upper_nibble = (data & 0xF0); // get 4 bit upper
	lower_nibble = (data << 4) & 0xF0; // get 4 bit lower

	data_t[0] = upper_nibble | 0x0D; // en = 1, rw = 0, rs = 1
	data_t[1] = upper_nibble | 0x09; // en = 0, rw = 0, rs = 1
	data_t[2] = lower_nibble | 0x0D; // en = 1, rw = 0, rs = 1
	data_t[3] = lower_nibble | 0x09; // en = 0, rw = 0, rs = 1

	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, (uint8_t*)data_t, 4, 100);
}

void lcd_clear(LCD_HandleTypedef* lcd){
	lcd_send_cmd(lcd, 0x01);
	HAL_Delay(2);
}

void lcd_gotoxy(LCD_HandleTypedef* lcd, uint8_t col, uint8_t row){
	uint8_t cmd_address;
	switch(row){
		case 0:
			cmd_address = 0x80 + col; // first row
			break;
		case 1:
			cmd_address = 0xC0 + col; // second row
			break;
		case 2:
			cmd_address = 0x94 + col;
			break;
		case 3:
			cmd_address = 0xD4 + col;
			break;
		default: return; // ignore invalid row number
	}
	lcd_send_cmd(lcd, cmd_address);
}

void lcd_init(LCD_HandleTypedef* lcd, I2C_HandleTypeDef* _hi2c, uint16_t _address){
	// set data lcd
	lcd->hi2c = _hi2c;
	lcd->address = _address;

	HAL_Delay(50); // wait for lcd power-up
	lcd_send_cmd(lcd, 0x30); // wake up command
	HAL_Delay(5);
	lcd_send_cmd(lcd, 0x30); // wake up command
	HAL_Delay(1);
	lcd_send_cmd(lcd, 0x30); // wake up command
	HAL_Delay(10);
	lcd_send_cmd(lcd, 0x20); // set to 4-bit mode
	HAL_Delay(10);

	// LCD configuration commands
	lcd_send_cmd(lcd, 0x28);  // 4-bit mode, 2 lines, 5x8 font
	HAL_Delay(1);
	lcd_send_cmd(lcd, 0x08);  // Display off, cursor off, blink off
	HAL_Delay(1);
	lcd_send_cmd(lcd, 0x01);  // Clear display
	HAL_Delay(2);
	lcd_send_cmd(lcd, 0x06);  // Entry mode: cursor moves right
	HAL_Delay(1);
	lcd_send_cmd(lcd, 0x0C);  // Display on, cursor off, blink off
	HAL_Delay(10);
}

void lcd_send_string(LCD_HandleTypedef* lcd, char* str){
	while(*str) lcd_send_data(lcd, *str++);
}

void lcd_send_char(LCD_HandleTypedef* lcd, char c){
	lcd_send_data(lcd, c);
}

void lcd_send_float(LCD_HandleTypedef* lcd, float num){
	char buffer[16];
	sprintf(buffer, "%4.1f", num);
	lcd_send_string(lcd, buffer);
}

void lcd_send_int(LCD_HandleTypedef* lcd, uint32_t num){
	char buffer[16];
	sprintf(buffer, "%4.1d", num);
	lcd_send_string(lcd, buffer);
}

void lcd_create_char(LCD_HandleTypedef* lcd, uint8_t location, uint8_t charmap[]){
	location &= 0x7;
	lcd_send_cmd(lcd, 0x40 | (location << 3));
	for(uint8_t i=0; i<8; i++){
		lcd_send_data(lcd, charmap[i]);
	}
}
