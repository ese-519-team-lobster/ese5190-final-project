
#include "joint_control.h"
#include "pico/stdlib.h"
#include "ik_calculations.h"

//Returns the PWM value that corresponds to the current Joint.kinematic_link.angle value,
//based on the min/max angle & PWM (linear interpolation)
uint16_t angle_to_pwm(Joint * joint) {
    return joint->servo.pwm_min + (rad2deg(joint->kinematic_link.angle) - rad2deg(joint->kinematic_link.min_angle)) * (joint->servo.pwm_max - joint->servo.pwm_min) / (rad2deg(joint->kinematic_link.max_angle) - rad2deg(joint->kinematic_link.min_angle));
}
uint16_t get_servo_position(Servo * servo, double pos) {
    return servo->pwm_min + (pos) * (servo->pwm_max - servo->pwm_min) / (100);
}