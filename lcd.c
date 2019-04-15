
#include "lcd.h"
#include "delay.h"
#include "msp.h"

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
    // delay_us_auto(40);

    // Display ON/OFF control
    // lcd_command(0);  // Not actually needed?
    lcd_command(CMD_DISP_CTL_INIT);

    // Wait >39us
    delay_ms_auto(1);
    // delay_us_auto(40);

    // Display Clear
    // lcd_command(0);  // Not actually needed?
    lcd_command(CMD_DISP_CLR);

    // Wait >1.53ms
    delay_ms_auto(2);

    //Entry Mode Set
    // lcd_command(0);  // Actually needed
    lcd_command(CMD_ENTRY_MODE);

    delay_ms_auto(30);
}

void lcd_command(char i){
    char upper = i >> 4;
    char lower = i;
    lcd_db_write(upper); // put shifted data on output Port
    clear_RS(); // D/I=LOW : send instruction
    clear_RW(); // R/W=LOW : Write
    Nybble(); // Send lower 4 bits
    lcd_db_write(lower); // put shifted data on output Port
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
    // delay_us_auto(40);
}

inline void lcd_home(){
    lcd_command(CMD_HOME);
    delay_ms_auto(2); // Wait >1.5ms
}

inline void lcd_clear(){
    lcd_command(0);
    lcd_command(CMD_DISP_CLR);
    delay_ms_auto(30); // Wait >1.5ms
}

inline void lcd_disp_on(){
    lcd_command(CMD_DISP_CTL_D);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);
}

inline void lcd_disp_off(){
    lcd_command(CMD_DISP_CTL_OFF);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);
}

inline void lcd_blink_on(){
    lcd_command(CMD_DISP_CTL_DCB);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);
}

inline void lcd_blink_off(){
    lcd_command(CMD_DISP_CTL_DC);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);
}
inline void lcd_cursor_on(){
    lcd_command(CMD_DISP_CTL_DC);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);
}

inline void lcd_cursor_off(){
    lcd_command(CMD_DISP_CTL_D);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);
}


static inline void lcd_db_write(unsigned char i){
    P4->OUT &= ~LCD_DB_PINS;
    P4->OUT |= i & LCD_DB_PINS;
}

static inline void Nybble(){
    P4->OUT |= EN;
    NOP //Delay(1);    // enable pulse width >= 300ns
    P4->OUT &= ~EN;    // Clock enable: falling edge
}

void inline set_RS(){
    P4->OUT |= RS;
}

void inline clear_RS(){
    P4->OUT &= ~RS;
}

void inline set_RW(){
    P4->OUT |= RW;
}

void inline clear_RW(){
    P4->OUT &= ~RW;
}

