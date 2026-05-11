#include "basic-functions.h"

#include "parameters.h"
#include "queue"
#include "robot-config.h"
#include "vex-tools.h"
#include "vex.h"
#include "controller.h"

bool descore_status=false;
bool hook_status=false;
bool store_status=false;
/**
 * 吸球
 */
void spinRoller(float _input){
  _input = fabs(_input) >100 ? sign(_input) *100 :_input;
  if (!_input) {
    Motor_Roller1.stop(coast);
    Motor_Roller2.stop(coast);
  }else{
    Motor_Roller1.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
    Motor_Roller2.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  }
}

/**
 * 第二层
 */
void spinshooter_m(float _input){
  _input = fabs(_input) >100 ? sign(_input) *100 :_input;
  if (!_input) {
    Motor_Roller1.stop(coast);
    Motor_Roller2.stop(coast);
    Motor_Roller3.stop(coast);
  }else{
    Motor_Roller1.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
    Motor_Roller2.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
    Motor_Roller3.spin(directionType::rev, (int)127 * _input, voltageUnits::mV);
  }
}

/**
 * 第三层
 */
void spinshooter_l(float _input){
  _input = fabs(_input) >100 ? sign(_input) *100 :_input;
  if (!_input) {
    Motor_Roller1.stop(coast);
    Motor_Roller2.stop(coast);
    Motor_Roller3.stop(coast);
  }else{
    Motor_Roller1.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
    Motor_Roller2.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
    Motor_Roller3.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  }
}

/**
 * 铲斗控制
 */
void descore_control(){
  if(UP){
    descore_status=!descore_status;
    if(descore_status){
      descore.set(1); 
    }else{
      descore.set(0);
    }
  }
}
/**
 * 钩子控制
 */
void hook_control(){
  if(X){
    hook_status=!hook_status;
    if(hook_status){
      hook.set(1); 
    }else{
      hook.set(0);
    }
  }
}
/**
 * 储球控制
 */
void store_control(){
  if(Y){
    store_status=!store_status;
    if(store_status){
      store.set(1); 
    }else{
      store.set(0);
    }
  }
}

double easeInOutRobotSensitivity(double input){
  // 保证输出在 [0, 100] 范围内
    bool sign =input<0;
    double normalizedInput=abs(input)/100.0;
    double t=std::min(100.0, normalizedInput * normalizedInput * (3 - 2 * normalizedInput) * 100.0);
    return (sign?-t:t);
}