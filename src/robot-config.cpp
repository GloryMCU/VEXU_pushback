#include "robot-config.h"
#include "vex-tools.h"
#include "differential-base.h"

using namespace vex;

brain Brain;
controller Controller = controller(primary);

bool RemoteControlCodeEnabled = true;

//吸轮方向为前，左马达反转，右马达正转
motor Motor_L1=motor(PORT1,ratio6_1,true);
motor Motor_L2=motor(PORT2,ratio6_1,true);
motor Motor_L3=motor(PORT3,ratio6_1,true);
motor Motor_R1=motor(PORT6,ratio6_1,false);
motor Motor_R2=motor(PORT13,ratio6_1,false);
motor Motor_R3=motor(PORT17,ratio6_1,false);
motor_group Left_Motors=motor_group(Motor_L1,Motor_L2,Motor_L3);
motor_group Right_Motors=motor_group(Motor_R1,Motor_R2,Motor_R3);
motor Motor_Roller1=motor(PORT14,ratio6_1,false); //控制吸轮马达
motor Motor_Roller2=motor(PORT15,ratio6_1,true);  //传动马达
motor Motor_Roller3=motor(PORT19,ratio6_1,false); //控制吐球马达
inertial IMU = inertial(PORT7); //陀螺仪
digital_out descore=digital_out(Brain.ThreeWirePort.F); //铲斗
digital_out hook=digital_out(Brain.ThreeWirePort.E); //钩子
digital_out store=digital_out(Brain.ThreeWirePort.H); //储球



/**
 * @brief 初始化机器人设备，如：陀螺仪、颜色传感器等……
 */
void initRobot(void) {
  Controller.Screen.setCursor(5, 1);
  Controller.Screen.print("%19s", "IMU Calibrating...");
  IMU.startCalibration();
  while (IMU.isCalibrating()) {
    this_thread::sleep_for(5);
  }
  Controller.Screen.setCursor(5, 1);
  Controller.Screen.print("%19s", "IMU Ready!");
}

void brainprint(){
  while(true){
    float _heading = getHeading();
    _heading =reduce_negetive_180_to_180(_heading);
    Controller.Screen.clearScreen();
    Controller.Screen.print("%19s%.1f", "Heading: ", _heading);
    this_thread::sleep_for(5);
  }
}