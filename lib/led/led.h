#ifndef LED_H
#define LED_H

#include <stdlib.h>
#include "pico/stdlib.h"

#define GREEN_LED_PIN 11 // GPIO para LED verde
#define BLUE_LED_PIN 12  // GPIO para LED azul
#define RED_LED_PIN 13   // GPIO para LED vermelho

void init_led(uint8_t pin);
void init_leds();
void turn_on_leds();
void turn_off_leds();
void set_led_green();
void set_led_blue();
void set_led_red();
void set_led_yellow();
void set_led_cyan();
void set_led_magenta();

#endif // LED_H
