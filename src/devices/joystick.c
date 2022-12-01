#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "stdbool.h"

#define SEL_IN 27
#define X_IN 29
#define Y_IN 28

int main() {
    stdio_init_all();
    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(X_IN);
    adc_gpio_init(Y_IN);
    gpio_init(SEL_IN);
    gpio_set_dir(SEL_IN, GPIO_IN);
    gpio_pull_up(SEL_IN);

    bool z_axis = false;
    bool prev_sel = false;
    bool curr_sel = false;

    while (1) {
        curr_sel = !gpio_get(SEL_IN);

        // If we just pressed the joystick
        if (!prev_sel && curr_sel) {
            z_axis = !z_axis;
            if (z_axis) {
                printf("Switched to Z-axis\n");
            } else {
                printf("Switched to Y-axis\n");
            }
        }

        // Select ADC input 3 (GPIO29)
        adc_select_input(3);
        uint16_t X = adc_read();
        // Select ADC input 2 (GPIO28)
        adc_select_input(2);
        uint16_t Y = adc_read();

        if (z_axis) {
            if (X >= 3000) {
                printf("X RIGHT\n");
            } else if (X <= 1000) {
                printf("X LEFT\n");
            } else if (Y >= 3000) {
                printf("Z UP\n");
            } else if (Y <= 1000) {
                printf("Z DOWN\n");
            } else {
                printf("---\n");
            }
        } else {
            if (X >= 3000) {
                printf("X RIGHT\n");
            } else if (X <= 1000) {
                printf("X LEFT\n");
            } else if (Y >= 3000) {
                printf("Y UP\n");
            } else if (Y <= 1000) {
                printf("Y DOWN\n");
            } else {
                printf("---\n");
            }
        }

        prev_sel = curr_sel;
        sleep_ms(100);
    }
}
