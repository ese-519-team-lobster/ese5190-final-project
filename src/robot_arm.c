#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/i2c.h"

#include "proj_version.h"
#include "ik_calculations.h"
#include "robot_params.h"
#include "joint_control.h"

#include "PCA9685.h"
#include "joystick.h"

#define I2C_SDA_PIN 22
#define I2C_SCL_PIN 23
#define I2C_INSTANCE i2c1

#define PCA9685_ADDR 0x40
#define PCA9685_OSC_FREQ 26600000 //tuned to the PCA9685 we have on hand

#define ARM_JOINT_COUNT 5

PCA9685 pwm_driver;
Chain arm_chain;

Joint joints[ARM_JOINT_COUNT];
Servo gripper;
bool gripper_closed;

typedef struct  {
  double x;
  double y;
  double z;  
  double phi;
} Cart_pos;

Cart_pos curr_pos;
Cart_pos new_pos;

void commands() {
    //send commands to servos
    
    //joints
    for(int i = 0; i < ARM_JOINT_COUNT; ++i) {
        printf("ch%d-%d,\t", joints[i].servo.channel, angle_to_pwm(&joints[i]));
        setPWM(&pwm_driver, joints[i].servo.channel, 0, angle_to_pwm(&joints[i]));
    }
    //gripper
    //setpwm(&pwm_driver, gripper.channel, 0, get_twostate_servo(&gripper, gripper_closed));
    printf("ch%d-%d\n", gripper.channel, get_twostate_servo(&gripper, gripper_closed));

    //send image to LCD
    //TODO

    //send my regards to Broadway
}

void read_inputs() {
    //joystick input
    int dx, dy, dz;
    //get_joystick_inputs(&dx, &dy, &dz);
    get_test_inputs(&dx, &dy, &dz);
    new_pos.x = curr_pos.x + fix2double(dx, 0);
    new_pos.y = curr_pos.y + fix2double(dy, 0);
    new_pos.z = curr_pos.z + fix2double(dz, 0);
    new_pos.phi = curr_pos.phi;


    //camera input
    //TODO
}

void image_processing() {
    //todo
}

void inverse_kinematics() {
    double base, shoulder, elbow, wrist_bend;
    if(solve_ik(&arm_chain, new_pos.x, new_pos.y, new_pos.z, &base, &shoulder, &elbow, &wrist_bend, new_pos.phi)) {
        curr_pos = new_pos;
        joints[0].kinematic_link.angle = base;
        joints[1].kinematic_link.angle = shoulder;
        joints[2].kinematic_link.angle = elbow;
        joints[3].kinematic_link.angle = wrist_bend;
        // printf("   SOLUTION - x: %.1lf\ty: %.1lf\tz: %.1lf   \tbase: %.1lf\tshoulder: %.1lf\telbow: %.1lf\twrist: %.1lf \t", 
        //     new_pos.x, 
        //     new_pos.y, 
        //     new_pos.z, 
        //     rad2deg(arm_chain.base_rotation->angle), 
        //     rad2deg(arm_chain.shoulder->angle), 
        //     rad2deg(arm_chain.elbow->angle), 
        //     rad2deg(arm_chain.wrist_bend->angle));
    }
    else
    {
        printf("NO SOLUTION - x: %.1lf\ty: %.1lf\tz: %.1lf   \t", new_pos.x, new_pos.y, new_pos.z);
    }
}

void ik_test(double x, double y, double z) {
    bool sol_found = solve_ik_direct(&arm_chain, x, y, z, FREE_ANGLE);
    printf("solved: %s\tbase: %.1lf\tshoulder: %.1lf\telbow: %.1lf\twrist: %.1lf\n", 
        sol_found ? "yes" : "no", 
        rad2deg(arm_chain.base_rotation->angle), 
        rad2deg(arm_chain.shoulder->angle), 
        rad2deg(arm_chain.elbow->angle), 
        rad2deg(arm_chain.wrist_bend->angle));
}

void calibrate() {
    uint16_t pwm_mins[ARM_JOINT_COUNT];
    uint16_t pwm_maxs[ARM_JOINT_COUNT];
    uint16_t pwm_curr[ARM_JOINT_COUNT];
    char input;
    int joint = 0;

    uint16_t pwm_val = 550;
    printf("setting J%d: %d      \r", joint, pwm_val);
    bool loop = true;

    //approximate 
    pwm_curr[0] = 350;
    pwm_curr[1] = 300;
    pwm_curr[2] = 370;
    pwm_curr[3] = 200;
    pwm_curr[4] = 310;

    for (int i = 0; i < ARM_JOINT_COUNT; i++) {
        setPWM(&pwm_driver, joints[i].servo.channel, 0, pwm_curr[i]);
    }

    while(loop) {

        setPWM(&pwm_driver, joints[joint].servo.channel, 0, pwm_curr[joint]);
        printf("setting J%d: %d     \r", joint, pwm_curr[joint]);

        input = getchar();
        switch(input) {
            case 'w':
                pwm_curr[joint] += 1;
                break;
            case 's':
                pwm_curr[joint] -= 1;
                break;
            case 'e':
                pwm_curr[joint] += 5;
                break;
            case 'd':
                pwm_curr[joint] -= 5;
                break;
            case 'r':
                pwm_curr[joint] += 10;
                break;
            case 'f':
                pwm_curr[joint] -= 10;
                break;
            case 'o':
                pwm_maxs[joint] = pwm_curr[joint];
                break;
            case 'p':
                pwm_mins[joint] = pwm_curr[joint];
                break;
            case 'n':
                joint += 1;
                break;
            case 'b':
                joint -= 1;
                break;
            case 'z':
                loop = false;
                break;
        }
        if (joint > 5) {joint = 5;}
        if (joint < 0) {joint = 0;}
        pwm_curr[joint] = (pwm_curr[joint] > 100 && pwm_curr[joint] < 650) ? pwm_curr[joint] : 500;
        //printf("ch%d-%d,\t", joints[i].servo.channel, pwm_val);
    }

    printf("\n");
    for (int i = 0; i < ARM_JOINT_COUNT; i++) {
        printf("J%d max: %d\tmin: %d\n", joint, pwm_maxs[i], pwm_mins[i]);

    }
}

void setup() {

    //init I2C
    i2c_init(I2C_INSTANCE, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    //init the PCS9685 PWM driver
    pwm_driver = get_pca9685_def(I2C_INSTANCE, PCA9685_ADDR, PCA9685_OSC_FREQ);
    pca9685_begin(&pwm_driver, 0);
    setPWMFreq(&pwm_driver, 50);

    //init IK and Servo maps
    joints[0].servo.channel = 0;
    joints[0].servo.pwm_max = BASE_ROTATION_MAX_PWM;
    joints[0].servo.pwm_min = BASE_ROTATION_MIN_PWM;
    joints[0].kinematic_link = init_link(BASE_HEIGHT, deg2rad(BASE_ROTATION_MIN_ANGLE), deg2rad(BASE_ROTATION_MAX_ANGLE));

    joints[1].servo.channel = 1;
    joints[1].servo.pwm_max = SHOULDER_MAX_PWM;
    joints[1].servo.pwm_min = SHOULDER_MIN_PWM;
    joints[1].kinematic_link = init_link(UPPER_ARM_LENGTH, deg2rad(SHOULDER_MIN_ANGLE), deg2rad(SHOULDER_MAX_ANGLE));

    joints[2].servo.channel = 2;
    joints[2].servo.pwm_max = ELBOW_MAX_PWM;
    joints[2].servo.pwm_min = ELBOW_MIN_PWM;
    joints[2].kinematic_link = init_link(FOREARM_LENGTH, deg2rad(ELBOW_MIN_ANGLE), deg2rad(ELBOW_MAX_ANGLE));

    joints[3].servo.channel = 3;
    joints[3].servo.pwm_max = WRIST_BEND_MAX_PWM;
    joints[3].servo.pwm_min = WRIST_BEND_MIN_PWM;
    joints[3].kinematic_link = init_link(HAND_LENGTH, deg2rad(WRIST_BEND_MIN_ANGLE), deg2rad(WRIST_BEND_MAX_ANGLE));

    joints[4].servo.channel = 4;
    joints[4].servo.pwm_max = WRIST_ROTATION_MAX_PWM;
    joints[4].servo.pwm_min = WRIST_ROTATION_MIN_PWM;
    joints[4].kinematic_link = init_link(0, deg2rad(WRIST_ROTATION_MIN_ANGLE), deg2rad(WRIST_ROTATION_MAX_ANGLE));

    gripper.channel = 5;
    gripper.pwm_max = GRIPPER_MAX_PWM;
    gripper.pwm_min = GRIPPER_MIN_PWM;

    
    init_chain(&arm_chain,
        &joints[0].kinematic_link,
        &joints[1].kinematic_link,
        &joints[2].kinematic_link,
        &joints[3].kinematic_link,
        &joints[4].kinematic_link);

    //initialize the cartesian position
    new_pos.x = 150;
    new_pos.y = 0;
    new_pos.z = 96.5;
    new_pos.phi = FREE_ANGLE;

    //call the IK function to initialize the joint angles, since output function is first in loop
    inverse_kinematics();
}

int main() {
    stdio_init_all();

    while (!stdio_usb_connected());
    printf("Version: %d\nPress any key to begin program...\n", PROJECT_VERSION);

    char temp = getchar();
    setup();
    printf("oscfreq: %d\n", pwm_driver.oscillator_freq);
    //test function
    calibrate();
    //end test code

    while(1) {
        commands();
        read_inputs();
        //figure out how to use second core for this?
        image_processing();
        inverse_kinematics();
        sleep_ms(1000);
    }

    return 1;
}