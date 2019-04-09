/*
 * lcd.c
 *
 *  Created on: Apr 9, 2019
 *      Author: sfshaw
 */

//8-bit Initialization:
/**********************************************************/
void lcd_command_8bit(char i){
    P1 = i; //put data on output Port
    D_I =0; //D/I=LOW : send instruction
    R_W =0; //R/W=LOW : Write
    E = 1;
    Delay(1); //enable pulse width >= 300ns
    E = 0; //Clock enable: falling edge
}
/**********************************************************/
void lcd_write_8bit(char i){
    P1 = i; //put data on output Port
    D_I =1; //D/I=HIGH : send data
    R_W =0; //R/W=LOW : Write
    E = 1;
    Delay(1); //enable pulse width >= 300ns
    E = 0; //Clock enable: falling edge
}
/**********************************************************/
void lcd_init_8bit(){
    E = 0;
    Delay(100); //Wait >40 msec after power is applied
    lcd_command(0x30); //command 0x30 = Wake up
    lcd_write(30); //must wait 5ms, busy flag not available
    lcd_command(0x30); //command 0x30 = Wake up #2
    lcd_write(10); //must wait 160us, busy flag not available
    lcd_command(0x30); //command 0x30 = Wake up #3
    lcd_write(10); //must wait 160us, busy flag not available
    lcd_command(0x38); //Function set: 8-bit/2-line
    lcd_command(0x10); //Set cursor
    lcd_command(0x0c); //Display ON; Cursor ON
    lcd_command(0x06); //Entry mode set
}
/**********************************************************/
// 4-bit Initialization:
/**********************************************************/
void command(char i)
{
    P1 = i; //put data on output Port
    D_I =0; //D/I=LOW : send instruction
    R_W =0; //R/W=LOW : Write
    Nybble(); //Send lower 4 bits
    i = i<<4; //Shift over by 4 bits
    P1 = i; //put data on output Port
    Nybble(); //Send upper 4 bits
}
/**********************************************************/
void write(char i)
{
    P1 = i; //put data on output Port
    D_I =1; //D/I=HIGH : send data
    R_W =0; //R/W=LOW : Write
    Nybble(); //Clock lower 4 bits
    i = i<<4; //Shift over by 4 bits
    P1 = i; //put data on output Port
    Nybble(); //Clock upper 4 bits
}
/**********************************************************/
void Nybble()
{
    E = 1;
    Delay(1);    //enable pulse width >= 300ns
    E = 0;    //Clock enable: falling edge
}
/**********************************************************/
void init()
{
    P1 = 0;
    P3 = 0;
    Delay(100);    //Wait >40 msec after power is applied
    P1 = 0x30;    //put 0x30 on the output port
    Delay(30);    //must wait 5ms, busy flag not available
    Nybble();    //command 0x30 = Wake up
    Delay(10);    //must wait 160us, busy flag not available
    Nybble();    //command 0x30 = Wake up #2
    Delay(10);    //must wait 160us, busy flag not available
    Nybble();    //command 0x30 = Wake up #3
    Delay(10);    //can check busy flag now instead of delay
    P1= 0x20;    //put 0x20 on the output port
    Nybble();    //Function set: 4-bit interface
    command(0x28);    //Function set: 4-bit/2-line
    command(0x10);    //Set cursor
    command(0x0F);    //Display ON; Blinking cursor
    command(0x06);    //Entry Mode set
}
/**********************************************************/

#define HOME 0x02
#define MODE_8_BIT 0x30
#define MODE 2 LINE ex28
#define CURSOR ON 0x0A
#define CURSOR BLINK 0x09
#define DISPLAY ON Exec
#define CURSOR RIGHT ex06
#define CURSOR LEFT ex04
#define SET CURSOR ex80

// Send cursor to home position
// LCD Mode options

// Cursor moves to right
// Cursor moves to left
// Set cursor position to SET_CURSOR ADDRESS

void lcd_init_8bit(void);
void lcd_command_8bit(unsigned char command);
void lcd_write_8bit(unsigned char letter);

int main(void) {

    LCD_init();
    LCD_write('H');
    LCD_write('e');
    LCD_write('l');
    LCD_write('l');
    LCD_write1('o');

}
// 8-bit mode, 2 line, cursor on

void lcd_init_8bit(void) {
    // Setup GPIO for P3 and P4 to use LCD
    P3->SELO RS RW EN);
    P3->SEL1 &= ~(RS RW EN);
    P4->SELO = 0x00;
    P4->SEL1 = 0x00;
    P3->DIR = RS RW EN; // make P3 pins output for control
    P4->DIR OXFF;
    // make P4 pins output for data

    P3-> OUT&EN;  // Set Enable low
    delay_ms(50);  // wait >40 ms for LCD to power up

    LCD_command(MODE_8_BIT MODE_2_LINE); // wake up initialization
    LCD_command (MODE_8_BIT MODE_2_LINE); // set 8-bit data, 2-line
    LCD_command (DISPLAY_CURSOR ON); // display and cursor on
    LCD_Command (CLR DISP); // clear screen
    LCD_command (CURSOR RIGHT); // move cursor right after each char
}
