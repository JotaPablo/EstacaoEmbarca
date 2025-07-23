#include "neopixel.h"
#include "ws2812b.pio.h" // Biblioteca gerada pelo arquivo .pio durante compilação.
#include <stdio.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"


// Buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio = pio0;
uint sm;
uint32_t np_led_value;


/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {

    uint offset = pio_add_program(pio0, &ws2812b_program);
    sm = pio_claim_unused_sm(pio0, true);
    ws2812b_program_init(pio0, sm, offset, LED_PIN);

    // Limpa buffer de pixels.
    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        npSetLED(i, 0, 0, 0);
    }
}

void npFill(const uint8_t r, const uint8_t g, const uint8_t b){
    for (uint i = 0; i < LED_COUNT; ++i) {
        npSetLED(i, r, g, b);
    }
}

/**
 * Transforma os dados em 
 */

uint32_t encode_rgb(npLED_t cor){

    unsigned char R = (unsigned char)cor.R;
    unsigned char G = (unsigned char)cor.G;
    unsigned char B = (unsigned char)cor.B;

    return (G << 24) | (R << 16) | (B << 8);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {

    // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
    for (uint i = 0; i < LED_COUNT; ++i) {
        uint32_t np_led_value = encode_rgb(leds[i]);
        pio_sm_put_blocking(np_pio, sm, np_led_value);
    }
}
// Calcula o índice na matriz de LEDs
// Linhas pares(0, 2, 4): esquerda para direita; ímpares(1, 3): direita para esquerda.
int npGetIndex(int x, int y) {
    if (y % 2 == 0) {
        return y * 5 + (4 - x); // Linha par (esquerda para direita).
    } else {
        return y * 5 + x; // Linha ímpar (direita para esquerda).
    }
}

void npFillCollumns(int x, int altura, uint8_t r, uint8_t g, uint8_t b){
    for (int y = 0; y <= altura - 1; y++) {
        npSetLED(npGetIndex(x, y), r, g, b);
    }
}
