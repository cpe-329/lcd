#ifndef LCD_H_
#define LCD_H_


#define DB4 0b00000001  // P4.0
#define DB5 0b00000010  // P4.1
#define DB6 0b00000100  // P4.2
#define DB7 0b00001000  // P4.3
#define RS  0b00010000  // P4.4 aka D/I
#define RW  0b00100000  // P4.5
#define EN  0b01000000  // P4.6
#define LCD_DB_PINS (DB7 | DB6 | DB5 | DB4)
#define LCD_PINS_MASK (LCD_DB_PINS | EN | RW | RS)  // P4.0-4.6

#define CMD_CLR_DISP 0x01
#define CMD_HOME 0x02  //  Send cursor to home position
//  lcd Mode options
#define CMD_MODE_8_BIT 0x30  // 0b00110000
#define CMD_MODE_4_BIT 
#define CMD_MODE_2_LINE 0x28
#define CMD_CURSOR_ON 0x0A
#define CMD_CURSOR_BLINK 0x09
#define CMD_DISPLAY_ON 0x0C
#define CMD_CURSOR_RIGHT 0x06  //  Cursor moves to right
#define CMD_CURSOR_LEFT 0x04  //  Cursor moves to left
#define CMD_SET_CURSOR 0x80  //  Set cursor position to SET_CURSOR ADDRESS

void lcd_init();
void lcd_command(char i);
void lcd_write(char i);
static void Nybble();

void lcd_init_8bit(void); // TODO: remove this


#endif /* LCD_H_ */
