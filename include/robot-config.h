#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_
#include "vex.h"
using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;

extern inertial IMU;
extern motor Motor_Roller1;
extern motor Motor_Roller2;
extern motor Motor_Roller3;
extern motor Motor_L1;
extern motor Motor_L2;
extern motor Motor_L3;
extern motor Motor_R1;
extern motor Motor_R2;
extern motor Motor_R3;
extern motor_group Left_Motors;
extern motor_group Right_Motors;
extern digital_out descore;
extern digital_out hook;
extern digital_out store;

void initRobot(void);
void brainprint();
#endif