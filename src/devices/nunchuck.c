/*
    adapted from https://gist.github.com/lsalmon/fad583622c4bdcb93f0f5722792a143f
*/

#include "nunchuck.h"
#include <stdio.h>

static bool prev_c = false;
static bool prev_z = false;

static bool toggle_z_axis = false;

static uint8_t js_deadband = 20;
static uint8_t js_midpoint = 128;
#define DELTA 5

static void read_nunchuck_data(nunchuck * device, nunchuck_output * output) {
    
    uint8_t buf[6];
    uint8_t cmd = 0x00;
    sleep_us(50);
    i2c_write_blocking(device->i2c, device->addr, &cmd, 1, false);
    sleep_us(250);

    i2c_read_blocking(device->i2c, device->addr, buf, 6, false);

    output->joystick_x_axis = buf[0];
    output->joystick_y_axis = buf[1];

    // Accelerometer data on 10 bits -> 2 bits shift
    // The 2 lower bits for each axis are in the misc buffer
    output->accelerometer_x_axis = (buf[2] << 2) + ((buf[5] & 0x0C) >> 2);
    output->accelerometer_y_axis = (buf[3] << 2) + ((buf[5] & 0x30) >> 4);
    output->accelerometer_z_axis = (buf[4] << 2) + ((buf[5] & 0xC0) >> 6);

    // First two bits of the misc buffer are the buttons
    // 0 : pressed - 1 : released
    output->z_button = (buf[5] & 0x01) == 0x01 ? false : true;
    output->c_button = (buf[5] & 0x02) == 0x02 ? false : true;

}

void get_nunchuck_inputs(nunchuck * device, int * x_axis, int * y_axis, int * z_axis, bool * z_button) {
    *x_axis = 0;
    *y_axis = 0;
    *z_axis = 0;
    nunchuck_output output;
    read_nunchuck_data(device, &output);

    bool curr_c = output.c_button;
    bool curr_z = output.z_button;

    // If c is pressed
    if (!prev_c && curr_c) {
        toggle_z_axis = !toggle_z_axis;
    }

    // If z is pressed
    if (!prev_z && curr_z) {
        *z_button = true;
    }
    else {
        *z_button = false;
    }

    // Select ADC input 3 (GPIO29)
    uint8_t X = output.joystick_x_axis;
    // Select ADC input 2 (GPIO28)
    uint8_t Y = output.joystick_y_axis;

    printf("x_nunchuck:%d\ty_nunchuck:%d\n", X, Y);
    if (X >= js_midpoint + js_deadband) {
        printf("X RIGHT\n");
        *x_axis = DELTA;
    } else if (X <= js_midpoint - js_deadband) {
        printf("X LEFT\n");
        *x_axis = -DELTA;
    }

    if (toggle_z_axis) {
       
        if (Y >= js_midpoint + js_deadband) {
            printf("Z UP\n");
            *z_axis = DELTA;
        } else if (Y <= js_midpoint - js_deadband) {
            printf("Z DOWN\n");
            *z_axis = -DELTA;
        }
    } else {
        if (Y >= js_midpoint + js_deadband) {
            printf("Y UP\n");
            *y_axis = DELTA;
        } else if (Y <= js_midpoint - js_deadband) {
            printf("Y DOWN\n");
            *y_axis = -DELTA;
        }
    }

    prev_c = curr_c;
    prev_z = curr_z;
}



void config_nunchuck(nunchuck * device) {

    device->addr = 0x52;
    uint8_t buf[2];
    
    // Init nunchuck
    buf[0] = 0xF0;
    buf[1] = 0x55;
    i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
    
    sleep_us(50);
     

    // Disable encryption on sensor data output
    buf[0] = 0xFB;
    buf[1] = 0x00;
    i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
    sleep_us(50);

    uint8_t buf2[6];
    uint8_t cmd = 0xFA;
    i2c_write_blocking(device->i2c, device->addr, &cmd, 1, false);
    sleep_us(50);
    i2c_read_blocking(device->i2c, device->addr, buf2, 6, false);

}