#ifndef DIFFERENTIAL_BASE_H_
#define DIFFERENTIAL_BASE_H_
#include "string"

float getLeftPos();
float getRightPos();
float getForwardPos();
void resetLeftPos();
void resetRightPos();
void resetForwardPos();
float getHeading();

void moveLeft(float);
void moveLeftVel(float);
void lockLeft(void);
void unlockLeft(void);
void moveRight(float);
void moveRightVel(float);
void lockRight(void);
void unlockRight(void);
void moveForward(float);
void moveClockwise(float);
void moveclock(float _input);
void lockBase(void);
void unlockBase(void);

void pidForwardRel(float _target);
void pidForwardAbs(float _target);
void pidRotateRel(float _target);
void pidRotateAbs(float _target);

void turn_to_angle(float _target,float _max_voltage);
void turn_angle(float _angle,float _max_voltage);
void drive_distance(float _distance,float _max_pct);

#endif