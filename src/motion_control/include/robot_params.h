
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

#define BASE_ROTATION_MIN_ANGLE 23.0
#define BASE_ROTATION_MAX_ANGLE 198.0 

#define SHOULDER_MIN_ANGLE 0.0
#define SHOULDER_MAX_ANGLE 180.0

#define ELBOW_MIN_ANGLE -60.0
#define ELBOW_MAX_ANGLE 110.0 

#define WRIST_BEND_MIN_ANGLE -30.0
#define WRIST_BEND_MAX_ANGLE 140.0

#define WRIST_ROTATION_MIN_ANGLE 10.0
#define WRIST_ROTATION_MAX_ANGLE 170.0

#define GRIPPER_MIN_POS 0.0
#define GRIPPER_MAX_POS 100.0

// Individual servo PWM limits

#define BASE_ROTATION_MIN_PWM 500 //swapped 
#define BASE_ROTATION_MAX_PWM 100

#define SHOULDER_MIN_PWM 150
#define SHOULDER_MAX_PWM 450

#define ELBOW_MIN_PWM 485 //swapped 
#define ELBOW_MAX_PWM 160

#define WRIST_BEND_MIN_PWM 95
#define WRIST_BEND_MAX_PWM 450

#define WRIST_ROTATION_MIN_PWM 130
#define WRIST_ROTATION_MAX_PWM 475

#define GRIPPER_MIN_PWM 100
#define GRIPPER_MAX_PWM 350

