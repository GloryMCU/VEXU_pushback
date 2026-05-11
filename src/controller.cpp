#include "controller.h"
double t;
int A1, A2, A3, A4;
bool L1, L2, R1, R2, X, Y, A, B, LEFT, RIGHT, UP, DOWN, last_L1, last_L2, last_R1, last_R2, last_X, last_Y, last_A, last_B, last_LEFT, last_RIGHT, last_UP, last_DOWN;

/**
 * @brief 更新手柄按键和摇杆的输入，注意刷新率应该和手动线程一致。
 */
void defineController() {
  last_L1 = L1;
  last_L2 = L2;
  last_R1 = R1;
  last_R2 = R2;
  last_X = X;
  last_Y = Y;
  last_A = A;
  last_B = B;
  last_LEFT = LEFT;
  last_RIGHT = RIGHT;
  last_UP = UP;
  last_DOWN = DOWN;
  t = Brain.Timer.value() * 1000.0;
  A1 = Controller.Axis1.position(vex::percentUnits::pct);
  A2 = Controller.Axis2.position(vex::percentUnits::pct);
  A3 = Controller.Axis3.position(vex::percentUnits::pct);
  A4 = Controller.Axis4.position(vex::percentUnits::pct);
  L1 = Controller.ButtonL1.pressing();
  L2 = Controller.ButtonL2.pressing();
  R1 = Controller.ButtonR1.pressing();
  R2 = Controller.ButtonR2.pressing();
  X = Controller.ButtonX.pressing();
  Y = Controller.ButtonY.pressing();
  A = Controller.ButtonA.pressing();
  B = Controller.ButtonB.pressing();
  LEFT = Controller.ButtonLeft.pressing();
  RIGHT = Controller.ButtonRight.pressing();
  UP = Controller.ButtonUp.pressing();
  DOWN = Controller.ButtonDown.pressing();
}
