#include "autonomous.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"



const int pistonInterval = 400;

void autonInit(void) {
  triggerStatus(false);
  setPistonE(false);
  resetHeading();
  resetForwardPos();
  MyGps.resetForwardPosGps();
}


void auton_left_1(){
  //with the new robot 3
  MyTimer autotimer;
  autotimer.reset();

  setFlywheelSpeed(2850);
  voltageForward(-30);
  this_thread::sleep_for(100);
  voltageForward(-15);
  setIntakeSpeed(100);
  this_thread::sleep_for(100);
  setIntakeSpeed(0);
  
  PIDPosForwardAbs(50);
  setPistonI(true);
  setIntakeSpeed(100);
  MyGps.gpsPIDMove(380, 460, -1, 70);
  setPistonI(false);
  this_thread::sleep_for(850);

  //MyGps.gpsPIDMove(600, 1000);
  MyGps.gpsAim(-400, 2900, 4, 0.3, 35);
  this_thread::sleep_for(100);
  autonShoot(3, pistonInterval+150, 2810, 2850, 2810, 2850); 
  setFlywheelSpeed(2810);
  
  MyGps.gpsPIDMove(1090, 1000, -1, 70.0);
  PIDAngleRotateAbs(0, 4, 0.4, 35, 1); 
  posForwardAbsWithHeading(70, -700, 0);
  PIDPosCurveAbs(-300, -100, 3);

  MyGps.gpsPIDMove(350, 1000, 1, 85.0);
  MyGps.gpsAim(-400, 2900, 4, 0.3, 35);
  this_thread::sleep_for(100);
  autonShoot(3, pistonInterval+200, 2770, 2810, 2770, 2810); 
  setFlywheelSpeed(0);

  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
  }

  void auton_left_2(){
  MyTimer autotimer;
  autotimer.reset();

  setFlywheelSpeed(2860);
  setPistonI(true);
  setIntakeSpeed(100);

  posForwardAbsWithHeading(70, -380, 0);
  setPistonI(false);
  PIDPosForwardAbs(-480);
  setIntakeSpeed(100);
  this_thread::sleep_for(600);

  MyGps.gpsAim(-1600, -2700, 3, 0.4, 35);
  autonShoot(3, pistonInterval, 2820, 2860, 2820, 2860); //2720, 2760

  PIDAngleRotateAbs(-130, 4, 0.4, 35, 1);
  posForwardAbsWithHeading(70, -270, -130);
  voltageForward(-30);
  this_thread::sleep_for(250);
  voltageForward(-10);  
  setIntakeSpeed(100);
  this_thread::sleep_for(70);
  setIntakeSpeed(0);

  PIDPosForwardAbs(-300);
  setIntakeSpeed(100);

  setFlywheelSpeed(2920);
  setPistonI(true);

  MyGps.gpsPIDMove(-230, -330, -1, 90);
  setPistonI(false);
  setIntakeSpeed(100);
  this_thread::sleep_for(800);

  MyGps.gpsAim(-1600, -2700, 4, 0.4, 35);
  autonShoot(3, pistonInterval, 2880, 2910, 2880, 2910); //2850, 2900

  setFlywheelSpeed(2880);
  MyGps.gpsPIDMove(-200, 200, -1, 50);
  MyGps.gpsPIDMove(-160, -630, 1, 70);
  this_thread::sleep_for(100);
  MyGps.gpsAim(-1500, -2700, 4, 0.4, 35);
  autonShoot(3, pistonInterval, 2840, 2870, 2840, 2870); //2810, 2840
  
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
  }


void auton_right_1(){
 MyTimer autotimer;
  autotimer.reset();

  setFlywheelSpeed(2890);
  setPistonI(true);
  setIntakeSpeed(100);

  posForwardAbsWithHeading(70, -300, 0.1);
  setPistonI(false);
  PIDPosForwardAbs(-460);
  this_thread::sleep_for(500);

  MyGps.gpsAim(1150, -2550, 4, 0.4, 45);
  autonShoot(3, pistonInterval-40, 2850, 2890, 2830, 2870);

  setFlywheelSpeed(2840);
  PIDAngleRotateAbs(130, 4, 0.4, 35, 1.5);
  posForwardAbsWithHeading(50, -360, 130);
  setIntakeSpeed(80);
  voltageForward(-20);
  this_thread::sleep_for(200);
  voltageForward(-10);  
  this_thread::sleep_for(50);
  setIntakeSpeed(100);
  PIDPosForwardAbs(-150);

  MyGps.gpsPIDMove(1300, -380, -1, 60);
  MyGps.gpsAim(1150, -2550, 4, 0.4, 45);
  autonShoot(3, pistonInterval, 2800, 2840, 2790, 2830);

  setFlywheelSpeed(2840);
  float tmpAngle = MyGps.getAngle(850, 350, -1);
  float tmpDist = MyGps.getDistance(850, 350);
  PIDAngleRotateAbs(tmpAngle, 4, 0.4, 35, 3);
  posForwardAbsWithHeading(70, -tmpDist, tmpAngle);
  PIDPosCurveAbs(-100, -300, 20);
  tmpAngle = MyGps.getAngle(600, -550);
  tmpDist = MyGps.getDistance(600, -550);

  PIDAngleRotateAbs(tmpAngle, 4, 0.4, 45, 4);
  posForwardAbsWithHeading(90, tmpDist, tmpAngle);
  //MyGps.gpsPIDMove(600, -500, 1, 90);
  MyGps.gpsAim(1250, -2550, 3.2, 0.4, 45);
  autonShoot(3, pistonInterval, 2800, 2840, 2790, 2830); 

  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}


void auton_right_2(){

}

void auton_AWP(){
  //resets 
 MyTimer autotimer;
  autotimer.reset();

  setFlywheelSpeed(2790);
  voltageForward(-30);
  this_thread::sleep_for(100);
  voltageForward(-15);
  setIntakeSpeed(100);
  this_thread::sleep_for(100);
  setIntakeSpeed(0);
  
  PIDPosForwardAbs(50);
  setPistonI(true);
  setIntakeSpeed(100);
  MyGps.gpsPIDMove(360, 400, -1, 70);
  setPistonI(false);
  this_thread::sleep_for(850);

  //MyGps.gpsPIDMove(600, 1000);
  MyGps.gpsAim(-400, 2900, 4, 0.3, 35);
  this_thread::sleep_for(100);
  autonShoot(3, pistonInterval+150, 2750, 2790, 2750, 2790); 
  setFlywheelSpeed(2850);
  // 
  MyGps.gpsPIDMove(1620, 1870, -1, 60);
  MyGps.gpsAim(-400, 2900, 4, 0.3, 45);
  this_thread::sleep_for(100);
  autonShoot(3, pistonInterval+150, 2810, 2850, 2810, 2850);
  setIntakeSpeed(100);

  MyGps.gpsPIDMove(2000, 2230, -1, 100);
  PIDAngleRotateAbs(-85.0, 4, 0.1, 65, 2);
  //posForwardAbsWithHeading(50, -50, -90);

  setIntakeSpeed(0);
  voltageForward(-60);
  this_thread::sleep_for(100);
  voltageForward(-20);
  setIntakeSpeed(100, 1);
  this_thread::sleep_for(250);
  setIntakeSpeed(0);
  
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}


void auton_low(){
  //resets 
  MyTimer autotimer;
  autotimer.reset();

  setFlywheelSpeed(3020);
  setIntakeSpeed(100);
  posForwardAbsWithHeading(25,-900, 0);
  setIntakeSpeed(0);
  posForwardAbsWithHeading(80,-1300, 0);
  PIDPosForwardAbs(-1500);
  setIntakeSpeed(-100);
  this_thread::sleep_for(200);
  setIntakeSpeed(100);
  MyGps.gpsAim(-1650, 2700, 3, 0.2, 50);
  autonShoot(3, pistonInterval+200);

}

// #ifdef ROBOT1
void runAuton(int auton_choose) {
  
  setAutonMode();
  autonFlipper(true);
  autonInit();

  if (auton_choose == 1) auton_AWP();  //2+6 AWP
  else if (auton_choose == 2) auton_left_1(); //1+6 left
  else if (auton_choose == 3) auton_left_2();  //1+9 left
  else if (auton_choose == 4) auton_right_1(); //1+9 right
  else if (auton_choose == 5) auton_right_2(); 
  else if (auton_choose == 6) auton_low();  // 1+6 INNER
}
// #endif
