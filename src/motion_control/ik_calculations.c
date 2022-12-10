/*

    Adapted from https://github.com/cgxeiji/CGx-InverseK

*/

#include "ik_calculations.h"
//for debug prints:
#ifdef DEBUG_OUTPUT
#include <stdio.h>
#endif

double rad2deg(double rad) {
    return (rad + HALF_PI) * 180 / PI;
}

double deg2rad(double deg) {
    return deg / 180.0 * PI - HALF_PI;
}


Link init_link(double length, double min_angle, double max_angle) {
    Link ret;
    ret.length = length;
    ret.min_angle = min_angle;
    ret.max_angle = max_angle;
    ret.angle = deg2rad(90);
    return ret;
}

bool angle_valid(Link * L, double angle) {
    return (angle >= L->min_angle) && (angle <= L->max_angle);
}

void init_chain(Chain * C, Link * base, Link * shoulder, Link * elbow, Link * wrist_bend, Link * wrist_rotate) {
    C->base_rotation = base;
    C->shoulder = shoulder;
    C->elbow = elbow;
    C->wrist_bend = wrist_bend;
    C->wrist_rotate =  wrist_rotate;
    C->current_phi = 0.0;
	#ifdef DEBUG_OUTPUT
    printf("base: %.1lf\tshoulder: %.1lf\telbow: %.1lf\twrist: %.1lf\n", 
		rad2deg(C->base_rotation->angle), 
		rad2deg(C->shoulder->angle), 
		rad2deg(C->elbow->angle), 
		rad2deg(C->wrist_bend->angle));
	#endif
}

static bool cosine_rule(double opposite, double adjacent1, double adjacent2, double * angle) {
    double delta = 2 * adjacent1 * adjacent2;
	
	if (delta == 0) return false;
	
	double cos = (adjacent1*adjacent1 + adjacent2*adjacent2 - opposite*opposite) / delta;
	
	if ((cos > 1) || (cos < -1)) return false;
	
	*angle = acos(cos);
	
	return true;
}

// Solve the angles for XY with a fixed attack angle
static bool solve_fixed(Chain * C, double x, double y, double phi, double * shoulder, double * elbow, double * wrist) {
    // Adjust coordinate system for base as ground plane
	double _r = sqrt(x*x + y*y);
	double _theta = atan2(y, x);
	double _x = _r * cos(_theta - HALF_PI); 
	double _y = _r * sin(_theta - HALF_PI);
	double _phi = phi - HALF_PI;
	
	// Find the coordinate for the wrist
	double xw = _x - C->wrist_bend->length * cos(_phi);
	double yw = _y - C->wrist_bend->length * sin(_phi);
	
	// Get polar system
	double alpha = atan2(yw, xw);
	double R = sqrt(xw*xw + yw*yw);
	
	// Calculate the inner angle of the shoulder
	double beta;
	if(!cosine_rule(C->elbow->length, R, C->shoulder->length, &beta)) return false;
	
	// Calcula the inner angle of the elbow
	double gamma;
	if(!cosine_rule(R, C->shoulder->length, C->elbow->length, &gamma)) return false;
	
	// Solve the angles of the arm
	double _shoulder, _elbow, _wrist;
	_shoulder = alpha - beta;
	_elbow = PI - gamma;
	_wrist = _phi - _shoulder - _elbow;
	
	// Check the range of each hinge
	if (!angle_valid(C->shoulder, _shoulder) || !angle_valid(C->elbow, _elbow) || !angle_valid(C->wrist_bend, _wrist)) {
		// If not in range, solve for the second solution
		_shoulder += 2 * beta;
		_elbow *= -1;
		_wrist = _phi - _shoulder - _elbow;
		
		// Check the range for the second solution
		if (!angle_valid(C->shoulder, _shoulder) || !angle_valid(C->elbow, _elbow) || !angle_valid(C->wrist_bend, _wrist)) return false;
	}
	
	// Return the solution
	*shoulder = _shoulder;
	*elbow = _elbow;
	*wrist = _wrist;
	
	return true;
}


// Solve the angles for XY with a free attack angle
static bool solve_free(Chain * C, double x, double y, double * shoulder, double * elbow, double * wrist) {
	if (solve_fixed(C, x, y, C->current_phi, shoulder, elbow, wrist)) return true;
	for (double phi = -DOUBLE_PI; phi < DOUBLE_PI; phi += DEGREE_STEP) {
		if (solve_fixed(C, x, y, phi, shoulder, elbow, wrist)) {
			C->current_phi = phi;
			return true;
		} 
	}
	
	return false;
}

// Solve the angles for XYZ with a fixed attack angle
bool solve_ik(Chain * C, double x, double y, double z, double * base, double * shoulder, double * elbow, double * wrist, double phi) {
	// Solve the angle of the base
	double _r = sqrt(x*x + y*y);
	double _base = atan2(y, x);
	
	// Check the range of the base
	if (!angle_valid(C->base_rotation, _base)) {
		// If not in range, flip the angle
		_base += (_base < 0) ? PI : -PI;
		_r *= -1;
		if (phi != FREE_ANGLE) {
			phi =  PI - phi;
		}
	}
	
	// Solve XY (RZ) for the arm plane
	if (phi == FREE_ANGLE) {
		if (!solve_free(C, _r, z - C->base_rotation->length, shoulder, elbow, wrist)) return false;
	} else {
		if (!solve_fixed(C, _r, z - C->base_rotation->length, phi, shoulder, elbow, wrist)) return false;
	}
	
	// If there is a solution, return the angles
	*base = _base;
	
	return true;
}

// Solve the angles for XYZ with or without a fixed attack angle. Pass FREE_ANGLE for no attack angle.
// Store the result directly in the provided Chain instead of individual doubles.
bool solve_ik_direct(Chain * C, double x, double y, double z, double phi) {
	double base, shoulder, elbow, wrist;
    if (solve_ik(C, x, y, z, &base, &shoulder, &elbow, &wrist, phi)) {
		C->base_rotation->angle = base;
		C->shoulder->angle = shoulder;
		C->elbow->angle = elbow;
		C->wrist_bend->angle = wrist;
		return true;
	}
	else {
		return false;
	}
}