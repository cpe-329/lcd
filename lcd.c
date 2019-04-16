
#include "lcd.h"
#include "delay.h"
#include "msp.h"
// clear the display
void Clear_LCD(){
    lcd_clear();
}
// move the cursor to the top left of the LCD
void Home_LCD(){
    lcd_home();
}
 // write a character on the LCD
void Write_char_LCD(unsigned char i){
    lcd_write(i);
} 

void lcd_init(){
    // Setup GPIO pins
    P4->SEL0 &= ~LCD_PINS_MASK;  // Initialize all P4 pins
    P4->SEL1 &= ~LCD_PINS_MASK;  // as GPIO 
    P4->DIR |= LCD_PINS_MASK;  // Set pins to output mode
    P4->OUT &= ~LCD_PINS_MASK; // Clear pins
    
    // Power up delay
    delay_ms_auto(32);  // wait >30ms during power up 
    
    // Function set
    lcd_command(CMD_STARTUP);  // Wake up signal?
    lcd_command(CMD_STARTUP);  // Wake up signal?
    lcd_command(CMD_SET_2L_4B); // Select mode
    lcd_command(CMD_SET_2L_4B); // Select mode
    
    // Wait >39us
    delay_ms_auto(1);
    // delay_us_auto(40);  // Datasheet delay, lies

    // Display ON/OFF control
    lcd_command(CMD_DISP_CTL_INIT);

    // Wait >39us
    delay_ms_auto(1);
    // delay_us_auto(40);  // Datasheet delay, lies

    // Display Clear
    lcd_command(CMD_DISP_CLR);

    // Wait >1.53ms
    delay_ms_auto(2);

    //Entry Mode Set
    lcd_command(CMD_ENTRY_MODE);

    delay_ms_auto(30);
}

void lcd_command(char i){
    lcd_db_write(i >> 4); // put shifted data on output Port
    clear_RS(); // D/I=LOW : send instruction
    clear_RW(); // R/W=LOW : Write
    Nybble(); // Send lower 4 bits
    lcd_db_write(i); // put shifted data on output Port
    Nybble(); // Send upper 4 bits
}


void lcd_write(char i){
    lcd_db_write(i >> 4); // put data on output Port
    set_RS();  // D/I=HIGH : send data
    clear_RW();  // R/W=LOW : Write
    Nybble(); // Clock lower 4 bits
    lcd_db_write(i); // put shifted data on output Port
    Nybble(); // Clock upper 4 bits
    delay_ms_auto(1);  // Wait >37us
    // delay_us_auto(40);  // Datasheet delay, lies
}

// Wrapper for lcd_command with long delay
static inline void lcd_command_delay(unsigned char i){
    lcd_command(i);
    delay_ms_auto(30);
}

// Set cursor to home, does not clear
inline void lcd_home(){
    lcd_command_delay(CMD_HOME);
}

// Set cursor to home, clears display
inline void lcd_clear(){
    lcd_command_delay(CMD_DISP_CLR);
}

// Turn display on, without cursor or blinking
inline void lcd_disp_on(){
    lcd_command_delay(CMD_DISP_CTL_D);
}

// Turn display off, hides text
inline void lcd_disp_off(){
    lcd_command_delay(CMD_DISP_CTL_OFF);
}

// Turn display, cursor and blinking on
inline void lcd_blink_on(){
    lcd_command_delay(CMD_DISP_CTL_DCB);
}

// Turn display and cursor on, no blinking
inline void lcd_blink_off(){
    lcd_command_delay(CMD_DISP_CTL_DC);
}

// Turn display and cursor on, no blinking
inline void lcd_cursor_on(){
    lcd_command_delay(CMD_DISP_CTL_DC);
}

// Turn cursor and blinking off
inline void lcd_cursor_off(){
    lcd_command_delay(CMD_DISP_CTL_D);
}

// Write a char to lcd data bus
static inline void lcd_db_write(unsigned char i){
    P4->OUT &= ~LCD_DB_PINS;
    P4->OUT |= i & LCD_DB_PINS;
}

// Clock data into lcd driver
static inline void Nybble(){
    P4->OUT |= EN;
    NOP //Delay(1);    // enable pulse width >= 300ns
    P4->OUT &= ~EN;    // Clock enable: falling edge
}

// Bring RS / D/I LCD pin HIGH
static void inline set_RS(){
    P4->OUT |= RS;
}

// Bring RS / D/I LCD pin LOW
void inline clear_RS(){
    P4->OUT &= ~RS;
}

// Bring RW LCD pin HIGH
void inline set_RW(){
    P4->OUT |= RW;
}

// Bring RW LCD pin HIGH
void inline clear_RW(){
    P4->OUT &= ~RW;
}

