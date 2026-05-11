#include "differential-base.h"

#include "PID.h"
#include "parameters.h"
#include "robot-config.h"
#include "vex-tools.h"
#include "vex.h"

#define left_pos (Motor_L1.position(deg) + Motor_L2.position(deg)+Motor_L3.position(deg)) / 3.0 * 82.55 * M_PI / 360.0 *0.75;
#define right_pos (Motor_R1.position(deg) + Motor_R2.position(deg)+Motor_R3.position(deg)) / 3.0 * 82.55 * M_PI / 360.0 *0.75;
#define heading IMU.rotation() / IMU_10 * 3600;
static float left_pos_last = 0, right_pos_last = 0;

/**
 * return the position of left side of base (mm from last reset position)
 * according to encoder value
 * @return unit: mm
 */
float getLeftPos() { return left_pos_last - left_pos; }
/**
 * return the position of right side of base (mm from last reset position)
 * according to encoder value
 * @return unit: mm
 */
float getRightPos() { return right_pos - right_pos_last; }
/**
 * return the vertical position of base (mm from last reset position) according
 * to encoder value
 * @return unit: mm
 */
float getForwardPos() { return (getLeftPos() + getRightPos()) / 2; }
/**
 * reset encoder on the left side of the base
 */
void resetLeftPos() { left_pos_last = left_pos; }
/**
 * reset encoder on the right side of the base
 */
void resetRightPos() { right_pos_last = right_pos; }
/**
 * reset encoders on both side of the base
 */
void resetForwardPos() {
  resetLeftPos();
  resetRightPos();
}
/**
 * return the heading angle of robot in deg (+360 after a full clockwise turn,
 * -360 after a counter-clockwise turn)
 * @return float with 1 decimal point
 */
float getHeading() { return heading; }
/**
 * powers all motors on left side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
 */
void moveLeft(float _input) {
  if (fabs(_input) > 100)
    _input = sign(_input) * 100;
  Left_Motors.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}
/**
 * move left motors with speed feedback control
 * @param _input ranges from -100 : 100
 */
void moveLeftVel(float _input) {
  if (fabs(_input) > 60 )
    _input = sign(_input) * 60 ;
  Left_Motors.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}
/**
 * locks all motors on left side of base
 */
void lockLeft(void) {
  Left_Motors.stop(vex::brakeType::brake);
}
/**
 * unlocks all motors on left side of base
 */
void unlockLeft(void) {
  Left_Motors.stop(vex::brakeType::coast);
}
/**
 * powers all motors on right side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
 */
void moveRight(float _input){
  if (fabs(_input) > 100 )
    _input = sign(_input) * 100;
  Right_Motors.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}
/**
 * move right motors with speed feedback control
 * @param _input ranges from -100 : 100
 */
void moveRightVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 60 )
    _input = sign(_input) * 60 ;
  Right_Motors.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}
/**
 * locks all motors on right side of base
 */
void lockRight(void) {
  Right_Motors.stop(vex::brakeType::brake);
}
/**
 * unlocks all motors on right side of base
 */
void unlockRight(void) {
  Right_Motors.stop(vex::brakeType::coast);
}
/**
 * move forward with _input% power
 * @param _input ranges from -100 : 100
 */
void moveForward(float _input) {
  moveLeft(_input);
  moveRight(_input);
}
/**
 * rotate clockwise with _input% power
 * @param _input ranges from -100 : 100
 */
void moveClockwise(float _input) {
  moveLeft(_input);
  moveRight(-_input);
}

void moveclock(float _input){
  Left_Motors.spin(directionType::rev,(int)_input*1000,voltageUnits::mV);
  Right_Motors.spin(directionType::fwd,-(int)_input*1000,voltageUnits::mV);
}
/**
 * stop the base with hold mode
 */
void lockBase(void) {
  lockLeft();
  lockRight();
}
/**
 * stop the base with coast mode
 */
void unlockBase(void) {
  unlockLeft();
  unlockRight();
}

void pidRotateRel(float _target) { pidRotateAbs(getHeading() + _target); }
void pidRotateAbs(float _target) { 
  auto pid = PID();
  MyTimer timer;
  // data transfer to prevend from huge angle rotation
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(5); // use if sentance to define the I coeff
  pid.setErrorTolerance(0.5);
  pid.setDTolerance(15);
  pid.setJumpTime(50);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1000) {
    pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1], BASE_ROTATE_PID[2]);
    pid.update(getHeading());
    moveclock(pid.getOutput());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}

void pidForwardRel(float _target) { pidForwardAbs(getForwardPos() + _target); }
void pidForwardAbs(float _target) {
  auto pid = PID();
  pid.setCoefficient(BASE_FORWARD_PID[0], BASE_FORWARD_PID[1], BASE_FORWARD_PID[2]);
  pid.setTarget(_target);
  pid.setIMax(20);
  pid.setIRange(5);
  pid.setErrorTolerance(3); // 设定误差小于多少的时候可以跳出循环
  pid.setDTolerance(30);    // 设定速度小于多少的时候车可以跳出循环
  pid.setJumpTime(20);
  while (!pid.targetArrived()) {
    pid.update(getForwardPos());
    moveRight(pid.getOutput());
    moveLeft(pid.getOutput());
    this_thread::sleep_for(5);
  }
  lockBase();
}

void drive_distance(float _distance,float _max_pct) {
  auto pid= PID();
  MyTimer timer;
  float start_average_pos=(getLeftPos()+getRightPos())/2.0;
  float average_pos=start_average_pos;
  pid.setCoefficient(0.28,0.05,0.1);
  pid.setTarget(_distance);
  pid.setIMax(20);
  pid.setIRange(5);
  pid.setErrorTolerance(3); 
  pid.setDTolerance(30);   
  pid.setJumpTime(20); 
  timer.reset();
  while(!pid.targetArrived()&& timer.getTime()<1750){
    average_pos=(getLeftPos()+getRightPos())/2.0;
    float pos=average_pos-start_average_pos;
    pid.update(pos);
    float _output=clamp(pid.getOutput(),-_max_pct,_max_pct);
    moveRight(_output);
    moveLeft(_output);
    this_thread::sleep_for(5);
  }
  lockBase();
}

void turn_to_angle(float _angle,float _max_voltage){
  auto pid=PID();
  float _target=reduce_negetive_180_to_180(_angle-getHeading());
  float start_heading=getHeading();
  float current_heading=start_heading;
  MyTimer timer;
  pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1], BASE_ROTATE_PID[2]);
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(5);
  pid.setErrorTolerance(0.3);
  pid.setDTolerance(15);
  pid.setJumpTime(50);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1250){
    current_heading=getHeading();
    float heading_diff=reduce_negetive_180_to_180(current_heading-start_heading);
    pid.update(heading_diff);
    float _output=clamp(pid.getOutput(),-_max_voltage,_max_voltage);
    moveclock(_output);
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}

void turn_angle(float _angle,float _max_voltage){
  auto pid=PID();
  float start_heading=getHeading();
  float target_heading=reduce_0_to_360(start_heading+_angle);
  float current_heading=start_heading;
  MyTimer timer;
  pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1], BASE_ROTATE_PID[2]);
  pid.setTarget(target_heading);
  pid.setIMax(15);
  pid.setIRange(5);
  pid.setErrorTolerance(0.5);
  pid.setDTolerance(15);
  pid.setJumpTime(50);
  timer.reset();
  while( !pid.targetArrived() && timer.getTime() < 1000){
    current_heading=getHeading();
    float heading_diff=reduce_0_to_360(current_heading-start_heading);
    pid.update(heading_diff);
    float _output=clamp( pid.getOutput(),-_max_voltage,_max_voltage);
    moveclock(_output);
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}