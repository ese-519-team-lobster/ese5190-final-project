#include "hardware/i2c.h"

typedef struct {
  i2c_inst_t * i2c;
  uint8_t addr;
} nunchuck;

typedef struct {
      uint8_t joystick_x_axis;
      uint8_t joystick_y_axis;
      uint8_t accelerometer_x_axis;
      uint8_t accelerometer_y_axis;
      uint8_t accelerometer_z_axis;
      bool z_button;
      bool c_button;
} nunchuck_output;


void config_nunchuck(nunchuck * device);
void get_nunchuck_inputs(nunchuck * device, int * x_axis, int * y_axis, int * z_axis, bool * z_button);
