#include "msp.h"

#include "watchdog.h"
#include "lcd.h"
#include "led.h"
#include "delay.h"

#define FREQ FREQ_24_MHZ

void main(void)
{
	DISABLE_WATCHDOG		// stop watchdog timer
	init_dco();
	set_dco(FREQ);
	init_led1();

    led1_on();
	lcd_init();

    // lcd_cursor_off();
    lcd_blink_on();

	lcd_write('A');
    
    delay_ms(1000, FREQ);

    lcd_write(' ');
    lcd_write('H');
    lcd_write('e');
    lcd_write('l');
    lcd_write('l');
    lcd_write('o');

    delay_ms(1000, FREQ);
    lcd_blink_off();
    lcd_clear();

    lcd_write('d');
    lcd_write('o');
    lcd_write('n');
    lcd_write('u');
    lcd_write('t');
    lcd_write('s');

    delay_ms(1000, FREQ);

    // lcd_disp_off();

    led1_off();
}
