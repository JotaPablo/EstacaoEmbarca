#ifndef BUTTON_H
#define BUTTON_H

#include "pico/stdlib.h"

// Pinos padrão para os botões
#define BUTTON_A 5
#define BUTTON_B 6
#define BUTTON_SW 22

extern volatile uint32_t last_debounce_time;
extern const uint8_t debounce_delay;

// Inicializa um botão em qualquer pino
void button_init(uint8_t pin);

// Inicializa os botões padrão A, B e SW com pull-up interno
void button_init_predefined(bool A, bool B, bool SW);

// Configura interrupções para os botões A, B e SW
void button_setup_irq(bool A, bool B, bool SW, gpio_irq_callback_t callback);

// Retorna true se o botão estiver pressionado (nível baixo)
bool button_is_pressed(uint8_t pin);


#endif // BUTTON_H
