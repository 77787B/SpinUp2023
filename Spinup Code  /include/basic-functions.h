#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "robot-config.h"
#include "parameters.h"
#include "vex.h"

// using namespace vex;
//autonchecker
void autonFlipper(bool);
// Calculation
float abbs(float);
float deg2rad(float);
float rad2deg(float);
float sqrf(float);
int sign(float);
float deg2range(float);

// Output functions
void moveLeft(float);
void moveLeftVel(float);
void lockLeft(void);
void unlockLeft(void);
void moveRight(float);
void moveRightVel(float);
void lockRight(void);
void unlockRight(void);
void moveForward(float);
void moveForwardVel(float);
void moveClockwise(float);
void lockBase(void);
void unlockBase(void);
float getCrtVel();

// Input functions

float getLeftPos();
float getRightPos();
float getForwardPos();
void resetLeftPos();
void resetRightPos();
void resetForwardPos();

float getHeading();
void resetHeading();
void resetHeading(float);
float getPitch();

// Piston
void setPistonS(bool);
void setPistonE(bool);
void setPistonA(bool);
void setPistonI(bool);

// Flywheeel
float getFlywheelSpeed();
bool flywheelSpeedInRange(float, float, float);
void setFlywheelSpeed(int, bool=1);
void flywheelPID();
void change_pstatus();

// Intake
void setIntakeSpeed(float, bool=0);
void intake();

// Trigger
bool IsTriggerStart();
int getShootCount();
void clearShootCount();
void triggerStatus(bool, int=PISTONINTERVAL, float=0, float=3600);
void trigger();

//autoaim
void auto_aim(void);
void set_aim_status(bool);
void get_signature_coor();
bool set_vision_data();
float vision_set_flywheel_speed();

void speed_volt(float, int);

#endif
