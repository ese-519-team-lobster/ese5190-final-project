
#include "pico/stdlib.h"
#include "ik_calculations.h"


typedef struct {
    int channel;
    uint16_t pwm_min;
    uint16_t pwm_max;
} Servo;

typedef struct {
    Servo servo;
    Link kinematic_link;
} Joint;

uint16_t angle_to_pwm(Joint * joint);
uint16_t get_twostate_servo(Servo * servo, bool isMin); 