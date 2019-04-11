#ifndef __LED_H__
#define __LED_H__

#include "msp.h"

#define BIT_0 0x01

#define PIN_SEL0_GPIO 0x01
#define PIN_SEL1_GPIO 0x01
#define GPIO_1_MODE_OUTPUT 0x01
#define LED1_STATE_ON 0x01

#define PIN_RGB_SEL0_GPIO 0b00000111
#define PIN_RGB_SEL1_GPIO 0b00000111
#define GPIO_RGB_MODE_OUTPUT 0b00000111
#define LED_RGB_STATE_ON 0b00000111

// Setup LED1
inline void init_led1(void);

// Toggle LED1
inline void toggle_led1(void);

// Setup RGB LED
void init_led_rgb(void);

// Set 3 bits of RGB LED
void set_led_rgb_value(int value);

#endif
