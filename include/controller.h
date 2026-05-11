#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "robot-config.h"
extern double t;
extern bool L1, L2, R1, R2, X, Y, A, B,
    LEFT, RIGHT, UP, DOWN,
    last_L1, last_L2, last_R1, last_R2, last_X, last_Y, last_A, last_B,
    last_LEFT, last_RIGHT, last_UP, last_DOWN;
extern int A1, A2, A3, A4;
void defineController();

#endif