#include "led.h"
#include "delay.h"

// Setup LED1
inline void init_led1(void){
    P1->SEL0 &= ~BIT_0;  // Set sel0 bit low for GPIO
    P1->SEL1 &= ~BIT_0;  // Set sel1 bit low for GPIO
    P1->DIR |= GPIO_1_MODE_OUTPUT;  // Set P1.0 to output mode 
    P1->OUT &= ~LED1_STATE_ON;  //  Set LED1 state to off
}

// Toggle LED1
inline void toggle_led1(void){
    P1->OUT ^= LED1_STATE_ON;  // XOR LED1 state to toggle
}


// Setup RGB LED
void init_led_rgb(void){
    P1->SEL0 &= ~PIN_RGB_SEL0_GPIO;  // Set sel0 bits low for GPIO
    P1->SEL1 &= ~PIN_RGB_SEL1_GPIO;  // Set sel1 bits low for GPIO
    P2->DIR |= GPIO_RGB_MODE_OUTPUT;  // Set P2.0 - P2.2 to output mode
    P2->OUT &= ~LED_RGB_STATE_ON;  // Set RGB LED state to off
}

// Set 3 bits of RGB LED
void set_led_rgb_value(int value){
    P2->OUT &= ~LED_RGB_STATE_ON;  // Clear previous RGB LED state
    P2->OUT |= (value & LED_RGB_STATE_ON);  // Set RGB LED state
}

