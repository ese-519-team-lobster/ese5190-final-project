#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "stdbool.h"

#define DELTA_X 20
#define DELTA_Y 20
#define DELTA_Z 10

static bool prev_sel = false;
static bool toggle_z_axis = false;
static int Y_IN = 28;
static int X_IN = 29;
static int SEL_IN = 27;

void joystick_init(int x_pin, int y_pin, int sel_pin) {
    Y_IN = y_pin;
    X_IN = x_pin;
    SEL_IN = sel_pin;
    adc_init();
    adc_gpio_init(X_IN);
    adc_gpio_init(Y_IN);
    gpio_init(SEL_IN);
    gpio_set_dir(SEL_IN, GPIO_IN);
    gpio_pull_up(SEL_IN);
}

void get_joystick_inputs(int * x_axis, int * y_axis, int * z_axis) {
    bool curr_sel = !gpio_get(SEL_IN);

    // If joystick is pressed
    if (!prev_sel && curr_sel) {
        *z_axis = !z_axis;
    }

    // Select ADC input 3 (GPIO29)
    adc_select_input(3);
    uint16_t X = adc_read();
    // Select ADC input 2 (GPIO28)
    adc_select_input(2);
    uint16_t Y = adc_read();

    if (X >= 3000) {
        printf("X RIGHT\n");
        *x_axis = DELTA_X;
    } else if (X <= 1000) {
        printf("X LEFT\n");
        *x_axis = -DELTA_X;
    }

    if (z_axis) {
       
        if (Y >= 3000) {
            printf("Z UP\n");
            *z_axis = DELTA_Z;
        } else if (Y <= 1000) {
            printf("Z DOWN\n");
            *z_axis = -DELTA_Z;
        }
    } else {
        if (Y >= 3000) {
            printf("Y UP\n");
            *y_axis = DELTA_Y;
        } else if (Y <= 1000) {
            printf("Y DOWN\n");
            *y_axis = -DELTA_Y;
        }
    }

    prev_sel = curr_sel;
}

void get_test_inputs(int * x_axis, int * y_axis, int * z_axis) {
    static int counter = 0;
    if(counter >= 0 && counter < 5){
        *x_axis = 5;
        *y_axis = 0;
        *z_axis = 0;
    }
    else if(counter >= 5 && counter < 10) {
        *x_axis = 0;
        *y_axis = 5;
        *z_axis = 0;
    }
    else if(counter >= 10 && counter < 15) {
        *x_axis = 0;
        *y_axis = 0;
        *z_axis = 5;
    }
    else if(counter >= 15 && counter < 20) {
        *x_axis = -5;
        *y_axis = 0;
        *z_axis = 0;
    }
    else if(counter >= 20 && counter < 25) {
        *x_axis = 0;
        *y_axis = -5;
        *z_axis = 0;
    }
    else if(counter >= 25 && counter < 30) {
        *x_axis = 0;
        *y_axis = 0;
        *z_axis = -5;
    }
    
    counter++;
    if(counter >= 30) {counter = 0;}
}
