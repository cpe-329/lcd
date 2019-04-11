
#include "lcd.h"
#include "delay.h"


//  8-bit mode, 2 line, cursor on
void lcd_init(){
    P4->SEL0 &= ~LCD_PINS_MASK;  // Initialize all P4 pins
    P4->SEL1 &= ~LCD_PINS_MASK;  // as GPIO 
    P4->DIR |= LCD_PINS_MASK;  // Set pins to output mode

    P4->OUT &= ~LCD_PINS_MASK;
    
    // P1 = 0;
    // P3 = 0;
    delay_ms_auto(50);    // Wait >40 msec after power is applied
    P1 = 0x30;    // put 0x30 on the output port
    Delay(30);    // must wait 5ms, busy flag not available
    Nybble();    // command 0x30 = Wake up
    Delay(10);    // must wait 160us, busy flag not available
    Nybble();    // command 0x30 = Wake up #2
    Delay(10);    // must wait 160us, busy flag not available
    Nybble();    // command 0x30 = Wake up #3
    Delay(10);    // can check busy flag now instead of delay
    P1 = 0x20;    // put 0x20 on the output port
    Nybble();    // Function set: 4-bit interface
    lcd_command(0x28);    // Function set: 4-bit/2-line
    lcd_command(0x10);    // Set cursor
    lcd_command(0x0F);    // Display ON; Blinking cursor
    lcd_command(CMD_CURSOR_RIGHT);    // Entry Mode set
}

void lcd_command(char i){
    P1 = i; // put data on output Port
    D_I =0; // D/I=LOW : send instruction
    R_W =0; // R/W=LOW : Write
    Nybble(); // Send lower 4 bits
    i = i<<4; // Shift over by 4 bits
    P1 = i; // put data on output Port
    Nybble(); // Send upper 4 bits
}

void lcd_write(char i){
    P1 = i; // put data on output Port
    D_I =1; // D/I=HIGH : send data
    R_W =0; // R/W=LOW : Write
    Nybble(); // Clock lower 4 bits
    i = i<<4; // Shift over by 4 bits
    P1 = i; // put data on output Port
    Nybble(); // Clock upper 4 bits
}

static void Nybble(){
    E = 1;
    NOP //Delay(1);    // enable pulse width >= 300ns
    E = 0;    // Clock enable: falling edge
}

void lcd_init_8bit(void){
    //  Setup GPIO for P3 and P4 to use lcd
    // P3->SEL0 &= ~(RS | RW | EN);
    P3->SEL1 &= ~(RS | RW | EN);
    P4->SEL0 = 0x00;
    P4->SEL1 = 0x00;
    P3->DIR |= RS | RW | EN; //  make P3 pins output for control
    P4->DIR = 0XFF;  //  make P4 pins output for data

    P3->OUT &= ~EN;  //  Set Enable low
    delay_ms(50);  //  wait >40 ms for lcd to power up

    lcd_command(CMD_MODE_8_BIT | CMD_MODE_2_LINE); //  wake up initialization
    lcd_command (CMD_MODE_8_BIT | CMD_MODE_2_LINE); //  set 8-bit data, 2-line
    lcd_command (CMD_DISPLAY_ON | CMD_CURSOR_ON); //  display and cursor on
    lcd_command (CMD_CLR_DISP); //  clear screen
    lcd_command (CMD_CURSOR_RIGHT); //  move cursor right after each char
}
