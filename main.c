#include "msp.h"

#include "watchdog.h"
#include "lcd.h"
#include "delay.h"
/**
 * main.c
 */
void main(void)
{
	DISABLE_WATCHDOG		// stop watchdog timer

	init_led1();
	toggle_led1();
	
	lcd_init();
    lcd_write('A');
    lcd_write(' ');
    lcd_write('H');
    lcd_write('e');
    lcd_write('l');
    lcd_write('l');
    lcd_write1('o');
}
