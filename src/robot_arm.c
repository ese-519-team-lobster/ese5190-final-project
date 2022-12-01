#include "pico/stdlib.h"
#include <stdio.h>
#include "proj_version.h"
#include "ik_calculations.h"


Chain arm_chain;

void setup() {
    init_chain(&arm_chain,
        init_link(0, deg2rad(0), deg2rad(180)),
        init_link(147, deg2rad(0), deg2rad(180)),
        init_link(130, deg2rad(0), deg2rad(180)),
        init_link(116, deg2rad(0), deg2rad(180)),
        init_link(0, deg2rad(0), deg2rad(180)));
}

void commands() {

}

void read_inputs() {

}

void image_processing() {

}

void inverse_kinematics() {

}

void test() {
    double a0, a1, a2, a3;
    bool sol_found = solve_ik(&arm_chain, 250.0, 250.0, 100.0, &a0, &a1, &a2, &a3, FREE_ANGLE);
    printf("solved: %s\tbase: %.1lf\tshoulder: %.1lf\telbow: %.1lf\twrist: %.1lf\n", sol_found ? "yes" : "no", rad2deg(a0), rad2deg(a1), rad2deg(a2), rad2deg(a3));
}


int main() {
    stdio_init_all();

    while (!stdio_usb_connected());
    printf("Version: %d\n\n", PROJECT_VERSION);

    setup();

    //test function
    test();
    //end test code

    while(1) {
        commands();
        read_inputs();
        //figure out how to use second core for this
        image_processing();
        inverse_kinematics();
    }

    return 1;
}