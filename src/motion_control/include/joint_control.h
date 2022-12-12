
#include "pico/stdlib.h"
#include "ik_calculations.h"

#define SERVO_SMOOTH_COEFF 0.3

typedef struct {
    int channel;
    uint16_t pwm_min;
    uint16_t pwm_max;
} Servo;

typedef struct {
    Servo servo;
    Link kinematic_link;
    double current_cmd_angle;
} Joint;

uint16_t angle_to_pwm(Joint * joint, bool smooth);
uint16_t get_servo_position(Servo * servo, double pos); 