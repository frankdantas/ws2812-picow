#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#define WS2812_PIN 7
#define IS_RGBW false

PIO pio;//Objeto PIO para a matriz de led
uint sm;//Objeto SM stateMachine para a matriz de led
uint offset = 0;//Memory offset para a matriz de led

static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void show_queue(uint8_t len, uint8_t max_len){
    uint8_t intensidadeCor = 2;
    for(int i = 0; i < max_len; i++){
        if(i < len){
            put_pixel(pio, sm, urgb_u32(intensidadeCor, 0, 0));
        }else{
            put_pixel(pio, sm, urgb_u32(0, 0, 0));
        }
    }
    sleep_us(10);
}

int main()
{
    stdio_init_all();
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, WS2812_PIN, 1, true);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    while (true) {
        int r = rand() % 25;
        show_queue(r, 25);
        sleep_ms(1000);
    }

    pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
}
