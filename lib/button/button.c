#include "button.h"

volatile uint32_t last_debounce_time = 0;
const uint8_t debounce_delay = 250;

void button_init(uint8_t pin){
    gpio_init(pin);
    gpio_set_dir(pin,GPIO_IN);
    gpio_pull_up(pin);
}

void button_init_predefined(bool A,bool B,bool SW){

    if(A){
        button_init(BUTTON_A);
    }
    if (B)
    {
        button_init(BUTTON_B);
    }
    if (SW)
    {
        button_init(BUTTON_SW);
    }

}

void button_setup_irq(bool A,bool B,bool SW,gpio_irq_callback_t callback){

    gpio_set_irq_callback(callback);

    if (A && gpio_get(BUTTON_A))
    {
        gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, true);
    }
    if (B && gpio_get(BUTTON_B))
    {
        gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
    }
    if (SW && gpio_get(BUTTON_SW))
    {
        gpio_set_irq_enabled(BUTTON_SW, GPIO_IRQ_EDGE_FALL, true);
    }
    
}

bool button_is_pressed(uint8_t pin){

    return !gpio_get(pin);  // true se pressionado (nível baixo)

}
