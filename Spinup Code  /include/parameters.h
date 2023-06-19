#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "vex.h"

static int CRTCOLOR = 1; // 1 is red, 0 is blue

const float Movement_LowerLimit = 5;
const float Joystick_LowerDeadzone = 5;
volatile static float gpsPosX, gpsPosY, gpsHeading;
volatile static float vision_obj_x, vision_obj_y, vision_obj_height, vision_obj_width;

static const float PI = 3.1416;
//0: red, 1:blue, 2:both
static volatile int target_blue = 0;

#ifdef ROBOT1
  static const float WHEEL_DIAMETER = 3.25;
  static const float CHASSIS_GEAR_RATIO = 0.6;
  static const float CHASSISRADIUS = 155;
  static const float kGyro = 1800.0 / 1800.0;
  static const float FLYWHEELSPEED = 2600; // 2600
  static const float FLYWHEELSPEED_LOW = 2500;
  static const float PISTONINTERVAL =  350; //350 200
  static const float PISTONPUSHTIME = 180; //180 100
  static const float AIMSHOOTPIXEL = 140;
  static const float SHOOTOFFSET = 2;
  const float SHOOTER_FIT_PARA[2] = {39.7, -118};
#endif

#ifdef ROBOT2
  static const float WHEEL_DIAMETER = 3.25;
  static const float CHASSIS_GEAR_RATIO = 0.6;
  static const float CHASSISRADIUS = 155;
  static const float kGyro = 1800.0 / 1800.0;
  static const float FLYWHEELSPEED = 2700; // 2600
  static const float FLYWHEELSPEED_LOW = 2600;
  static const float PISTONINTERVAL = 350;
  static const float PISTONPUSHTIME = 180;
  static const float AIMSHOOTPIXEL = 140;
  static const float SHOOTOFFSET = 2.8;
  const float SHOOTER_FIT_PARA[2] = {40.3, -183};

#endif

#ifdef ROBOT3
  static const float WHEEL_DIAMETER = 2.75;
  static const float CHASSIS_GEAR_RATIO = 0.75;
  static const float CHASSISRADIUS = 142;
  static const float kGyro = 1800.0 / 1800.0;
  static const float FLYWHEELSPEED = 2450; //2350
  static const float FLYWHEELSPEED_LOW = 2350;
  static const float FLYWHEELSPEED_HIGH = 2700;
  static const float PISTONINTERVAL = 300;
  static const float PISTONPUSHTIME = 80;
  static const float AIMSHOOTPIXEL = 250;
  static const float SHOOTOFFSET = -6;
  const float SHOOTER_FIT_PARA[2] = {31.8, 356.5}; 
#endif

#endif
