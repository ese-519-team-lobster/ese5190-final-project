
#pragma once

//Lengths of each piece of the arm

//defining base height (distance between root of kinematic chain and the shoulder joint)
//as 0 for now so that the "floor" is actually about 6cm above the table
//to avoid crashing the arm during testing
#define BASE_HEIGHT 0 
#define UPPER_ARM_LENGTH 147
#define FOREARM_LENGTH 130 
#define WRIST_LENGTH 58 //From the WRIST_BEND joint to the WRIST_ROTATE servo spline
#define GRIPPER_LENGTH 58 //From the WRIST_ROTATE servo spline to the end of the claw
#define HAND_LENGTH (GRIPPER_LENGTH + WRIST_LENGTH)

//Individual servo angle limits, for IK calculations

#define BASE_ROTATION_MIN_ANGLE 0.0
#define BASE_ROTATION_MAX_ANGLE 180.0

#define SHOULDER_MIN_ANGLE 5.0
#define SHOULDER_MAX_ANGLE 175.0

#define ELBOW_MIN_ANGLE 0.0
#define ELBOW_MAX_ANGLE 180.0

#define WRIST_BEND_MIN_ANGLE 0.0
#define WRIST_BEND_MAX_ANGLE 180.0

#define WRIST_ROTATION_MIN_ANGLE 0.0
#define WRIST_ROTATION_MAX_ANGLE 180.0

#define GRIPPER_MIN_ANGLE 0.0
#define GRIPPER_MAX_ANGLE 180.0

// Individual servo PWM limits

#define BASE_ROTATION_MIN_PWM 200
#define BASE_ROTATION_MAX_PWM 600

#define SHOULDER_MIN_PWM 200
#define SHOULDER_MAX_PWM 550

#define ELBOW_MIN_PWM 200
#define ELBOW_MAX_PWM 550

#define WRIST_BEND_MIN_PWM 175
#define WRIST_BEND_MAX_PWM 600

#define WRIST_ROTATION_MIN_PWM 200
#define WRIST_ROTATION_MAX_PWM 600

#define GRIPPER_MIN_PWM 300
#define GRIPPER_MAX_PWM 500

