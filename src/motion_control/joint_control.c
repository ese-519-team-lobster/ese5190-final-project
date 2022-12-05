
#include "joint_control.h"
#include "pico/stdlib.h"

//Returns the PWM value that corresponds to the current Joint.kinematic_link.angle value,
//based on the min/max angle & PWM (linear interpolation)
uint16_t angle_to_pwm(Joint * joint) {
    return joint->servo.pwm_min + (joint->kinematic_link.angle - joint->kinematic_link.min_angle) * (joint->servo.pwm_max - joint->servo.pwm_min) / (joint->kinematic_link.max_angle - joint->kinematic_link.min_angle);
}
uint16_t get_twostate_servo(Servo * servo, bool isMin) {
    return isMin ? servo->pwm_min : servo->pwm_max;
}