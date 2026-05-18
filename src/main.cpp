#include "basic-functions.h"
#include "controller.h"
#include "differential-base.h"
#include "parameters.h"
#include "robot-config.h"
#include "vex-tools.h"
#include "vex.h"
#include "v5_apiuser.h"

using namespace vex;
using namespace std;

#ifdef COMPETITION
competition Competition;
#endif
static bool is_robot_init = false;
static bool Pitch_control = 1;

void handle_control(){
  while (1) {
    /*-------------Controller Setup-------------*/
    if (is_robot_init) {
      defineController();
    }
    /*-------------Base Movement Control-------------*/
    A3 = std::abs(A3) < JOYSTICK_DEADZONE ? 0 : A3;
    A1 = std::abs(A1) < JOYSTICK_DEADZONE ? 0 : A1;
    A1=A1*0.4;
    if (std::abs(A3 + A1) > 0)
      moveLeftVel(A3 + A1);
    else
      unlockLeft();
    if (std::abs(A3 - A1) > 0)
      moveRightVel(A3 - A1);
    else
      unlockRight();
    if(L1){
      spinRoller(60);
    }else if(L2){
      spinRoller(-60);
    }else if(R2){
      spinshooter_m(80);
    }else if(R1){
      spinshooter_l(100);
    }else{
      Motor_Roller1.stop(coast);
      Motor_Roller2.stop(coast);
      Motor_Roller3.stop(coast);
    }
    wait(50, msec);
  }
}

void air_control(){
  while(1){
    descore_control();
    hook_control();
    store_control();
    wait(150,msec);
  }
}

void autonomous(void){
  spinRoller(60);
  /*//吐中塔
  drive_distance(970,50);
  wait(300,msec);
  turn_to_angle(45,12);                                                                    
  spinshooter_m(100);
  drive_distance(70,100);
  wait(500,msec);
  spinshooter_m(0);

  //吸球
  drive_distance(-1230,40);
  wait(300,msec);
  turn_to_angle(180,4);
  descore.set(1);
  wait(300,msec);
  spinRoller(80);
  drive_distance(410,30);
  wait(500,msec);
  spinRoller(0);
  
  //吐长控制桥
  drive_distance(-550,50);    
  descore.set(0);
  wait(300,msec);                                 
  turn_to_angle(359,4);                                             
  wait(300,msec);
  drive_distance(165,70);
  spinshooter_l(100);
  wait(1000,msec);
  spinshooter_l(0);
  drive_distance(50,100);
  drive_distance(-50,100);

  /*
  //吸球
  drive_distance(-150,70);
  wait(300,msec);
  turn_to_angle(270,4);
  wait(300,msec);
  spinRoller(-100);
  wait(1000,msec);
  spinRoller(0);
  turn_to_angle(180,4);
  wait(200,msec);
  drive_distance(200,50);
  descore.set(1);
  wait(300,msec);
  spinRoller(80);
  drive_distance(400,30);
  wait(500,msec);
  spinRoller(0);  
  
  //吐长控制桥
  drive_distance(-550,50);
  descore.set(0);
  wait(300,msec);                                       
  turn_to_angle(360,4);                                           
  wait(300,msec);
  drive_distance(165,70);
  spinshooter_l(100);
  wait(1500,msec);
  spinshooter_l(0);
  drive_distance(50,100);
  drive_distance(-50,100);
  drive_distance(-600,40);
  turn_to_angle(105,4);
  drive_distance(1500,70);*/
}

void usercontrol(void){
  thread handle_thread(handle_control);
  thread air_thread(air_control);
}

int main(){
  initRobot();
  is_robot_init = true;
  while (true) {
    wait(100, msec);
  }
  #ifdef COMPETITION
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  #endif
  return 0;
}
