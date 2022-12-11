#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "hardware/vreg.h"

#include "proj_version.h"
#include "ik_calculations.h"
#include "robot_params.h"
#include "joint_control.h"

#include "PCA9685.h"
#include "joystick.h"
#include "point_track.h"

#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define I2C_INSTANCE i2c1

#define PCA9685_ADDR 0x40
#define PCA9685_OSC_FREQ 26600000 //tuned to the PCA9685 we have on hand

#define ARM_JOINT_COUNT 5

#define DEBUG_OUTPUT

PCA9685 pwm_driver;
Chain arm_chain;

Joint joints[ARM_JOINT_COUNT];
Servo gripper;
double gripper_pos;
bool gripper_closed;

typedef struct  {
  double x;
  double y;
  double z;  
  double phi;
  double wr_rot;
} Cart_pos;

Cart_pos curr_pos;
Cart_pos new_pos;

bool setup_failed = false;

void print_arm_pos(Cart_pos * cart) {
    #ifdef DEBUG_OUTPUT
        printf("x: %.1lf\ty: %.1lf\tz: %.1lf\twr: %.1lf\tphi_cmd: %.1lf\tphi_cal: %.1lf   \tbase: %.1lf\tshoulder: %.1lf\telbow: %.1lf\twrist: %.1lf \twr: %.1lf", 
            cart->x, 
            cart->y, 
            cart->z,
            cart->wr_rot,
            cart->phi,
            rad2deg(arm_chain.current_phi),
            rad2deg(arm_chain.base_rotation->angle), 
            rad2deg(arm_chain.shoulder->angle), 
            rad2deg(arm_chain.elbow->angle), 
            rad2deg(arm_chain.wrist_bend->angle),
            rad2deg(arm_chain.wrist_rotate->angle));
    #endif
}

void commands() {
    //send commands to servos
    
    //joints
    for(int i = 0; i < ARM_JOINT_COUNT; ++i) {
        #ifdef DEBUG_OUTPUT
            //printf("ch%d-%d,\t", joints[i].servo.channel, angle_to_pwm(&joints[i])); //TODO convert to degrees
        #endif
        setPWM(&pwm_driver, joints[i].servo.channel, 0, angle_to_pwm(&joints[i]));
    }
    //gripper
    setPWM(&pwm_driver, gripper.channel, 0, get_servo_position(&gripper, gripper_pos));
    #ifdef DEBUG_OUTPUT
        //printf("ch%d-%d", gripper.channel, get_servo_position(&gripper, gripper_pos));
    #endif

    //send image to LCD
    //TODO

    //send my regards to Broadway
}

void read_inputs() {
    //joystick input
    int dx, dy, dz, dw, dg;
    double dwr;
    //get_joystick_inputs(&dx, &dy, &dz);
    //get_test_inputs(&dx, &dy, &dz);
    get_console_inputs(&dx, &dy, &dz, &dw, &dwr, &dg);
    new_pos.x = curr_pos.x + fix2double(dx, 0);
    new_pos.y = curr_pos.y + fix2double(dy, 0);
    new_pos.z = curr_pos.z + fix2double(dz, 0);
    new_pos.phi = deg2rad(rad2deg(curr_pos.phi) + fix2double(dw, 0));
    new_pos.wr_rot = deg2rad(rad2deg(curr_pos.wr_rot) + dwr);
    gripper_pos = gripper_pos + fix2double(dg, 0);

    //camera input
    //TODO
}

bool inverse_kinematics() {
    double base, shoulder, elbow, wrist_bend, wrist_rotate;
    if(solve_ik(&arm_chain, new_pos.x, new_pos.y, new_pos.z, new_pos.wr_rot, &base, &shoulder, &elbow, &wrist_bend, &wrist_rotate, new_pos.phi)) {
        curr_pos = new_pos;
        joints[0].kinematic_link.angle = base;
        joints[1].kinematic_link.angle = shoulder;
        joints[2].kinematic_link.angle = elbow;
        joints[3].kinematic_link.angle = wrist_bend;
        //joints[4].kinematic_link.angle = wrist_rotate;
        joints[4].kinematic_link.angle = wrist_rotate; // deg2rad(90);
        #ifdef DEBUG_OUTPUT
        printf("SOLUTION:%.1lf,\ty:%.1lf,\tz:%.1lf,\tbase:%.1lf,\tshoulder:%.1lf,\telbow:%.1lf,\twrist:%.1lf,\twrist_rot:%.1lf\n",
            new_pos.x, 
            new_pos.y, 
            new_pos.z, 
            rad2deg(arm_chain.base_rotation->angle), 
            rad2deg(arm_chain.shoulder->angle), 
            rad2deg(arm_chain.elbow->angle), 
            rad2deg(arm_chain.wrist_bend->angle),
            rad2deg(arm_chain.wrist_rotate->angle));
        #endif
        return true;
    }
    else
    {
        #ifdef DEBUG_OUTPUT
        printf("\nNO SOLUTION - ");//x: %.1lf\ty: %.1lf\tz: %.1lf\tphi: %.1lf\tphi_cal: %.1lf   \t", new_pos.x, new_pos.y, new_pos.z, new_pos.phi, arm_chain.current_phi);
        print_arm_pos(&new_pos);
        print_arm_pos(&curr_pos);
        #endif
        return false;
    }
}

void ik_test(double x, double y, double z) {
    //bool sol_found = solve_ik_direct(&arm_chain, x, y, z, FREE_ANGLE);
    // printf("solved: %s\tbase: %.1lf\tshoulder: %.1lf\telbow: %.1lf\twrist: %.1lf\n", 
    //     sol_found ? "yes" : "no", 
    //     rad2deg(arm_chain.base_rotation->angle), 
    //     rad2deg(arm_chain.shoulder->angle), 
    //     rad2deg(arm_chain.elbow->angle), 
    //     rad2deg(arm_chain.wrist_bend->angle));
}

void calibrate() {
    uint16_t pwm_mins[ARM_JOINT_COUNT+1];
    uint16_t pwm_maxs[ARM_JOINT_COUNT+1];
    uint16_t pwm_curr[ARM_JOINT_COUNT+1];
    int input = 0;
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
    pwm_curr[5] = get_servo_position(&gripper, 50);

    pwm_mins[0] = BASE_ROTATION_MIN_PWM;
    pwm_mins[1] = SHOULDER_MIN_PWM;
    pwm_mins[2] = ELBOW_MIN_PWM;
    pwm_mins[3] = WRIST_BEND_MIN_PWM;
    pwm_mins[4] = WRIST_ROTATION_MIN_PWM;
    pwm_mins[5] = GRIPPER_MIN_PWM;

    pwm_maxs[0] = BASE_ROTATION_MAX_PWM;
    pwm_maxs[1] = SHOULDER_MAX_PWM;
    pwm_maxs[2] = ELBOW_MAX_PWM;
    pwm_maxs[3] = WRIST_BEND_MAX_PWM;
    pwm_maxs[4] = WRIST_ROTATION_MAX_PWM;
    pwm_maxs[5] = GRIPPER_MAX_PWM;

    for (int i = 0; i < ARM_JOINT_COUNT; i++) {
        setPWM(&pwm_driver, joints[i].servo.channel, 0, pwm_curr[i]);
    }
    setPWM(&pwm_driver,gripper.channel,0,pwm_curr[5]);

    while(loop) {

        if(joint <5) {
        setPWM(&pwm_driver, joints[joint].servo.channel, 0, pwm_curr[joint]);
        }
        else {
            setPWM(&pwm_driver, gripper.channel, 0, pwm_curr[5]);
        }
        printf("setting J%d: %d     ", joint, pwm_curr[joint]);

        do{
        input = getchar_timeout_us(0);
        //getchar();
        }while(input == PICO_ERROR_TIMEOUT);
        //printf("  char got: %c", (char)input);
        switch((char)input) {
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
            case 'p':
                pwm_maxs[joint] = pwm_curr[joint];
                printf("max saved!");
                break;
            case 'o':
                pwm_mins[joint] = pwm_curr[joint];
                printf("min saved!");
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
            case 'k':
                pwm_curr[joint] = pwm_mins[joint];
                break;
            case 'l':
                pwm_curr[joint] = pwm_maxs[joint];
                break;
            default:
                break;
        }
        //printf("eval'd char");
        if (joint > 5) {joint = 5;}
        if (joint < 0) {joint = 0;}
        pwm_curr[joint] = (pwm_curr[joint] > 50 && pwm_curr[joint] < 700) ? pwm_curr[joint] : 300;
        //printf("ch%d-%d,\t", joints[i].servo.channel, pwm_val);
        printf("\r");
    }

    printf("\n");
    for (int i = 0; i < ARM_JOINT_COUNT+1; i++) {
        printf("J%d max: %d\tmin: %d\n", i, pwm_maxs[i], pwm_mins[i]);

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

    //init joystick
    joystick_init(29, 28, 27);

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
    new_pos.x = FOREARM_LENGTH;
    new_pos.y = 0;
    new_pos.z = 65;
    new_pos.phi = deg2rad(0);
    new_pos.wr_rot = deg2rad(90);
    gripper_pos = 50;
    print_arm_pos(&new_pos);

    //call the IK function to initialize the joint angles, since output function is first in loop
    if(!inverse_kinematics()) {setup_failed = true;}

    //start the camera/image processing on the second core
    multicore_launch_core1(core1_entry);

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != MULTICORE_FLAG_VALUE)
        printf("core1 unexpected flag value\n");
    else {
        multicore_fifo_push_blocking(setup_failed ? MULTICORE_FLAG_TERMINATE : MULTICORE_FLAG_VALUE);
        printf("core1 started\n");
    }
}

void test() {
    uint8_t regaddr = PCA9685_PRESCALE;
    uint8_t buf;
    i2c_write_blocking(pwm_driver.i2c, pwm_driver.addr, &regaddr, 1, true);
    int ret = i2c_read_blocking(pwm_driver.i2c, pwm_driver.addr, &buf, 1, false);
    printf("new prescale val: %d\nret: %d\n", buf, ret);
    buf = pca9685_read_prescale(&pwm_driver);
    printf("new prescale val2: %d\n", buf);
    buf = pca9685_read_prescale(&pwm_driver);
    printf("new prescale val3: %d\n", buf);
    //setPWM(&pwm_driver, 15, 0, 600);
    //setPWM(&pwm_driver, 0, 0, 600);
    //while(1){}
}

int main() {
    stdio_init_all();

    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(1000);
    set_sys_clock_khz(250000, true);

    setup();
    #ifdef DEBUG_OUTPUT
    printf("oscfreq: %d\n", pwm_driver.oscillator_freq);
    #endif

    while (!stdio_usb_connected());
    printf("Version: %d\nPress any key to begin program...\n", PROJECT_VERSION);

    char temp = getchar();

    //send a signal to the other core to start the image processing logic
    multicore_fifo_push_blocking(MULTICORE_FLAG_VALUE);

    //test function
    calibrate();
    //end test code

    while(1) {
        commands();
        read_inputs();
        inverse_kinematics();
        sleep_ms(100);
    }

    return 1;
}