#include "skill.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"

const int pistonInterval = 400;

void skillInit() {
  triggerStatus(false);
  setPistonE(false);
  MyGps.resetForwardPosGps();
}

void skill(){
  MyTimer autotimer;
  autotimer.reset();
  
  setFlywheelSpeed(2650);
  voltageForward(-30);
  this_thread::sleep_for(100);
  voltageForward(-10);
  setIntakeSpeed(100);
  this_thread::sleep_for(300);
  setIntakeSpeed(0);

  MyGps.gpsPIDMove(0, 140);
  setIntakeSpeed(100);

  PIDAngleRotateAbs(108, 3.0, 0.4, 35, 1);
  PIDPosForwardAbs(-520);
  PIDAngleRotateAbs(90, 3.0, 0.4, 35, 1);
  posForwardAbsWithHeading(50, -20, 90);
  setIntakeSpeed(0);
  
  voltageForward(-30);
  this_thread::sleep_for(300);
  voltageForward(5);
  this_thread::sleep_for(200);
  voltageForward(-10);
  setIntakeSpeed(100);
  this_thread::sleep_for(250);
  setIntakeSpeed(0);

  posForwardAbsWithHeading(80, 100, 120);
  setIntakeSpeed(100);
  setFlywheelSpeed(2600);
  MyGps.gpsAimMove(1950, 30, 1450);

  //Shoot1
  autonShoot(3, pistonInterval);
  setIntakeSpeed(0);
  MyGps.gpsPIDMove(840, 440, -1, 60, "heading");
  setIntakeSpeed(100);
  resetForwardPos();
  posForwardAbsWithHeading(20, -350, IMU.rotation());
  //this_thread::sleep_for(500);

  MyGps.gpsAimMove(2250, 50, 700);

  //Shoot2
  autonShoot(3, pistonInterval);

  MyGps.gpsPIDMove(300, 570, -1, 60, "heading");
  resetForwardPos();
  posForwardAbsWithHeading(30, -420, IMU.rotation());
  this_thread::sleep_for(500);
  MyGps.gpsAimMove(2200, 50, 1600);

  //Shoot3
  setIntakeSpeed(-100);
  autonShoot(3, pistonInterval);
  setIntakeSpeed(100);

  resetForwardPos();
  PIDPosForwardAbs(-400);
  MyGps.gpsPIDMove(2390, 1710, -1, 50);
  MyGps.gpsAimMove(2500, 100, 700);

  //Shoot4
  autonShoot(3, pistonInterval);
  setIntakeSpeed(0);

  MyGps.gpsPIDMove(2200, 1900, -1, 50);
  setIntakeSpeed(100);
  resetForwardPos();
  posForwardAbsWithHeading(20, -500, IMU.rotation());

  PIDPosForwardAbs(-550);
  this_thread::sleep_for(100);
  PIDPosForwardAbs(-250);
  PIDAngleRotateAbs(185, 2, 0.3, 35);
  resetHeading();
  posForwardAbsWithHeading(50, -100, 0);
  
  setIntakeSpeed(0);
  voltageForward(-30);
  this_thread::sleep_for(400);
  voltageForward(5);
  this_thread::sleep_for(200);
  voltageForward(-10);
  setIntakeSpeed(100);
  this_thread::sleep_for(200);
  MyGps.setForwardPosGps(0, -25);
  setIntakeSpeed(10);

  

  MyGps.gpsPIDMove(0, 800);
  PIDAngleRotateAbs(90, 3, 0.4, 28, 1);
  posForwardAbsWithHeading(50, -350, 90);
  
  voltageForward(-30);
  this_thread::sleep_for(600);
  voltageForward(5);
  this_thread::sleep_for(100);
  voltageForward(-5);
  setIntakeSpeed(100);
  this_thread::sleep_for(300);
  setIntakeSpeed(0);

  

  MyGps.setForwardPosGps(-500, 800);
  resetForwardPos();
  PIDPosForwardAbs(150);
  setIntakeSpeed(-100);
  MyGps.gpsPIDMove(1000, 200);
  MyGps.gpsAimMove(2350, 250, 500);
  setIntakeSpeed(100);

  //Shoot5
  autonShoot(3, pistonInterval);


  MyGps.gpsPIDMove(900, 700, -1, 50, "heading");
  setIntakeSpeed(100);
  resetForwardPos();
  posForwardAbsWithHeading(20, -350, IMU.rotation());
  this_thread::sleep_for(500);
  MyGps.gpsAimMove(2350, 350, 700);
  setIntakeSpeed(-100);

  //Shoot6
  autonShoot(3, pistonInterval);
  setIntakeSpeed(100);

  resetForwardPos();
  PIDPosForwardAbs(-370);
  MyGps.gpsPIDMove(2550, 2000, -1, 50);
  MyGps.gpsAimMove(2550, 110, 500);

  //Shoot7
  autonShoot(3, pistonInterval);
  setIntakeSpeed(100);
  
  MyGps.gpsPIDMove(2550, 2500, -1,  90);

  PIDAngleRotateAbs(-135,3, 0.4, 28, 1);
  //Extension
  setPistonE(true);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
  this_thread::sleep_for(100000);
}

void runSkill(){
    skillInit();
    skill();
}
