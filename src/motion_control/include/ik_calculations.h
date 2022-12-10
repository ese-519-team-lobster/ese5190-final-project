/*

    Adapted from https://github.com/cgxeiji/CGx-InverseK

*/



#include <pico/double.h>

#pragma once

#define PI 3.14159265359
#define HALF_PI 1.5707963268
#define DOUBLE_PI 6.28318530718
#define DEGREE_STEP 0.01745329251

#define FREE_ANGLE 999.9

// Defines a link in a kinematic chain. Represents a joint followed by a linear structure.
// Angles are in radians
typedef struct {
    double length; //distance the structure extends past the joint.
    double min_angle; //
    double max_angle;
    double angle; //current angle in radians
} Link;

typedef struct chain_ {
    Link * base_rotation;
    Link * shoulder;
    Link * elbow;
    Link * wrist_bend; //The next joint (wrist_rotate) doesn't change the distance from this joint to the tip of the effector, so include that length here
    Link * wrist_rotate; //Length can be neglected
    double current_phi;
} Chain;

double rad2deg(double rad);
double deg2rad(double deg);

Link init_link(double length, double min_angle, double max_angle);
bool angle_valid(Link * L, double angle);

void init_chain(Chain * C, Link * base, Link * shoulder, Link * elbow, Link * wrist_bend, Link * wrist_rotate);
bool solve_ik(Chain * C, double x, double y, double z, double * base, double * shoulder, double * elbow, double * wrist, double phi);
bool solve_ik_direct(Chain * C, double x, double y, double z, double phi);

