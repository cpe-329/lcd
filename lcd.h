#ifndef LCD_H_
#define LCD_H_

//  1   2  3   4   5     6    7   8   9  10   11   12  13    14   15   16 
// VSS VDD V0  RS  RW    E   DB0 DB1 DB2 DB3  DB4  DB5  DB6  DB7 LED+ LED- 
// GND 3V3    P4.4 P4.5 P4.6                 P4.0 P4.1 P4.2 P4.3  5V  GND

#define P4_0 0b00000001
#define P4_1 0b00000010  // P4.1
#define P4_2 0b00000100  // P4.2
#define P4_3 0b00001000  // P4.3
#define P4_4 0b00010000  // P4.4 aka Data/Instr
#define P4_5 0b00100000  // P4.5
#define P4_6 0b01000000  // P4.6

#define DB4 P4_0  // P4.0
#define DB5 P4_1  // P4.1
#define DB6 P4_2  // P4.2
#define DB7 P4_3  // P4.3
#define RS  P4_4  // P4.4 aka Data/Instr
#define RW  P4_5  // P4.5
#define EN  P4_6  // P4.6

#define LCD_DB_PINS (DB7 | DB6 | DB5 | DB4)
#define LCD_PINS_MASK (EN | RW | RS| LCD_DB_PINS)  // P4.0-4.6

//  lcd Mode options
#define CMD_STARTUP       (0b00100000)
#define CMD_SET_2L_4B     (0b11000000)
#define CMD_DISP_CTL_OFF  (0b00001000)
#define CMD_DISP_CTL_D    (0b00001100)
#define CMD_DISP_CTL_DC   (0b00001110)
#define CMD_DISP_CTL_DCB  (0b00001111)
#define CMD_DISP_CTL_INIT CMD_DISP_CTL_D 
#define CMD_DISP_CLR      (0b00000001)
#define CMD_ENTRY_MODE    (0b00000110)
#define CMD_HOME          (0b00000010)

// Require wrappers
void Clear_LCD();  // clear the display
void Home_LCD();  // move the cursor to the top left of the LCD
void Write_char_LCD(unsigned char i);  // write a character on the LCD


void lcd_init();
void lcd_command(char i);
void lcd_write(char i);

inline void lcd_home();
inline void lcd_clear();
inline void lcd_disp_on();
inline void lcd_disp_off();
inline void lcd_blink_on();
inline void lcd_blink_off();
inline void lcd_cursor_on();
inline void lcd_cursor_off();

static inline void Nybble();
static inline void lcd_db_write(unsigned char i);

void inline set_RS();
void inline clear_RS();
void inline set_RW();
void inline clear_RW();

#endif /* LCD_H_ */
