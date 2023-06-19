#include "vex.h"
#include "robot-config.h"
#include "basic-functions.h"
#include "parameters.h"
#include "PID.h"
#include "my-timer.h"
#include "ezize.h"
#include "iostream"
#include "roundRobinQueue.h"
using namespace std;

bool autonChecker = false;

void autonFlipper(bool _checker) { autonChecker = _checker; }

float abbs(float x) { return x >= 0 ? x : -x; }

float deg2rad(float deg) { return deg / 180.0 * PI; }

float rad2deg(float rad) { return rad / PI * 180.0; }

float sqrf(float x) { return x * x; }

int sign(float _input) {
  if (_input > 0) return 1;
  else if (_input < 0) return -1;
  else return 0;
}

float deg2range(float _degree){
  int _cir = int(_degree / 360);
  _degree -= _cir * 360;
  if (_degree > 180.0) _degree -= 360;
  if (_degree < -180.0) _degree += 360;
  return _degree;
}

void moveLeft(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseLM.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseLB.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void moveLeftVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

void lockLeft(void) {
  // locks all motors on left side of base
  Motor_BaseLF.stop(vex::brakeType::hold);
  Motor_BaseLM.stop(vex::brakeType::hold);
  Motor_BaseLB.stop(vex::brakeType::hold);
}

void unlockLeft(void) {
  // unlocks all motors on left side of base
  Motor_BaseLF.stop(vex::brakeType::coast);
  Motor_BaseLM.stop(vex::brakeType::coast);
  Motor_BaseLB.stop(vex::brakeType::coast);
}

void moveRight(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseRM.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseRB.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void moveRightVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int) _input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int) _input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

void lockRight(void) {
  // locks all motors on right side of base
  Motor_BaseRF.stop(vex::brakeType::hold);
  Motor_BaseRM.stop(vex::brakeType::hold);
  Motor_BaseRB.stop(vex::brakeType::hold);
}

void unlockRight(void) {
  // unlocks all motors on right side of base
  Motor_BaseRF.stop(vex::brakeType::coast);
  Motor_BaseRM.stop(vex::brakeType::coast);
  Motor_BaseRB.stop(vex::brakeType::coast);
}

void moveForward(float _input) {
  // move forward with _input% power
  moveLeft(_input);
  moveRight(_input);
}

void moveForwardVel(float _input) {
  // move forward with _input% speed
  moveLeftVel(_input);
  moveRightVel(_input);
}

void moveClockwise(float _input) {
  // rotate clockwise with _input% power
  moveLeft(_input);
  moveRight(-_input);
}
void lockBase(void) {
  // lock the base
  lockLeft();
  lockRight();
}

void unlockBase(void) {
  // unlock the base
  unlockLeft();
  unlockRight();
}

float getCrtVel(){
  return (Motor_BaseLM.velocity(pct) + Motor_BaseRM.velocity(pct)) / 2;
}

static volatile float _leftPosLast = 0, _rightPosLast = 0;

float getLeftPos() {
  // return the position of left side of base (mm from last reset position) according to encoder value
  return (Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360) - _leftPosLast; // return mm
}

float getRightPos() {
  // return the position of right side of base (mm from last reset position) according to encoder value
  return (Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360) - _rightPosLast; // return mm
  //return 0;
}

float getForwardPos() {
  // return the vertical position of base (mm from last reset position) according to encoder value
  return (getLeftPos() + getRightPos()) / 2;
}

void resetLeftPos() {
  // reset encoder on the left side of the base
  _leftPosLast = (Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360);
}

void resetRightPos() {
  // reset encoder on the right side of the base
  _rightPosLast = (Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360);
}

void resetForwardPos() {
  // reset encoders on both side of the base
  resetLeftPos();
  resetRightPos();
}

static float headingOffset = 0;

float getHeading() {
  // return the heading angle of robot in deg (+360 after a full clockwise turn, -360 after a counter-clockwise turn) 
  return IMU.rotation() + headingOffset;
}

void resetHeading() {
  IMU.resetRotation();
  headingOffset = 0;
}

void resetHeading(float _offset) {
  // reset current heading angle of robot 
  IMU.resetRotation();
  headingOffset = _offset;
}

float getPitch() {
  // return the pitch angle of robot in deg
  return IMU.pitch();
}

void setPistonS(bool _input) {
  // set Shooter trigger piston accordingly
  if (_input)  PistonS.off();
  else PistonS.on();
}

void setPistonE(bool _input) {
  // set Expansion piston accordingly
  if (_input)  PistonE.off();
  else PistonE.on();
}

void setPistonA(bool _input) {
  // set Expansion piston accordingly
  if (_input)  PistonA.off();
  else PistonA.on();
}

void setPistonI(bool _input) {
  // set Expansion piston accordingly
  if (_input)  PistonI.off();
  else PistonI.on();
}

bool shoot_start = false;
bool shoot_status = false;
bool pistonS_start = false;
int shoot_count = 0;
float flywheel_lower_bound = 0, flywheel_upper_bound = 3600;
static MyTimer trigger_timer;

// static float lastRotation = 0; 

float getFlywheelSpeed(){
  float speed = 0, crt_speed = 0, max = -9999, min = 9999;
  for (int i = 0; i < 15; i++){
    crt_speed = (Motor_Flywheel1.velocity(rpm) * 6);
    if (crt_speed > max) max = crt_speed;
    if (crt_speed < min) min = crt_speed;
    speed += crt_speed;
  }
  speed = speed - max - min;
  return speed / 13;
  /*float crtRotation = Motor_Flywheel1.position(deg);
  float crtSpeed = (crtRotation - lastRotation) / 20 * 60000 / 360 * 6;
  lastRotation = crtRotation;
  return crtSpeed;*/
}

bool flywheelSpeedInRange(float speed, float lower_bound, float upper_bound){
  if (speed > lower_bound && speed < upper_bound) return 1;
  return 0;
}

float flywheel_rotation_speed;
float flywheel_voltage;
bool printstatus;
bool flywheelmode = 0;

void setFlywheelSpeed(int flywheelspeed, bool mode){
  flywheel_rotation_speed = flywheelspeed;
  flywheel_voltage =   (flywheel_rotation_speed-SHOOTER_FIT_PARA[1]) / SHOOTER_FIT_PARA[0];
  if (mode == 1) flywheelmode = 1;
  else flywheelmode = 0;
}

void change_pstatus(){
  printstatus=!printstatus;
}

static float currFlywheelSpeed = 0;

void flywheelPID(){
  float output, output_supply;
  auto pid = PID();
  bool firstTime = 0;

  pid.setIRange(1000);
  pid.setIMax(60);
  pid.setErrorTolerance(0);
  bool speedOK = false;
  while(true) {
    // set power supply for every shot
    currFlywheelSpeed = getFlywheelSpeed();
    float speed_error = (flywheel_rotation_speed - currFlywheelSpeed);
    
    if (trigger_timer.getTime() < 10) firstTime = 1;
    if (speed_error < 50) firstTime = 0;
    /*if (firstTime == 1){
      if(trigger_timer.getTime() > PISTONPUSHTIME && trigger_timer.getTime() < PISTONINTERVAL) output_supply = 0; // 50
      else if (speed_error > 50) output_supply = 0; // 30
      else output_supply = 0;
    }
    else{
      if (speed_error > 250) output_supply = 0;
      else output_supply = 0;
    }*/
    
    if (speed_error > 400 && speedOK) speedOK = false;
    if (speed_error < 0 && !speedOK) speedOK = true;
    float P = 0.05 * speed_error;
    if (fabs(P) > 5) P = sign(P) * 5; 
    output = speedOK ? flywheel_voltage + P : 100;
    
    if (shoot_status) pid.setCoefficient(0.0, 0.0, 0.0);
    else{
      if (!flywheelmode) pid.setCoefficient(0.004, 0.0, 0.0); // i:0.01 , 0.002
      else pid.setCoefficient(0.004, 0.0, 0.0);
    }

    pid.setTarget(flywheel_rotation_speed);
    pid.update(currFlywheelSpeed);
    // output = flywheel_voltage + output_supply;// + pid.getOutput();
    output = 100 < fabs(output)? 100 * sign(output): output;
    if(!flywheel_rotation_speed) Motor_Flywheel1.stop(coast);
    else Motor_Flywheel1.spin(directionType::fwd, (int)130 * output, voltageUnits::mV);
    
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("FlywheelSpeed      : %.1f                 ", currFlywheelSpeed );
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("FlywheelTargetSpeed: %.1f                 ", flywheel_rotation_speed);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("FlywheelVoltage    : %.1f                 ", flywheel_voltage);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Flywheel I         : %.1f                 ", pid.getI());
    this_thread::sleep_for(2);

    // cout << currFlywheelSpeed << endl;
  }
}

float intake_speed = 0;
bool stucked = 0;
bool isroll = 0;

void setIntakeSpeed(float _input, bool _isroll){
  intake_speed = _input;
  isroll = _isroll;
}

void intake() {
  MyTimer stuck_timer;
  while(true){
    if (!stucked && intake_speed > 80 && Motor_Intake1.velocity(pct) < 5 && !isroll){
      stuck_timer.reset();
      stucked = 1;
    }
    if (stuck_timer.getTime() > 600) stucked = 0;

    if (stucked && stuck_timer.getTime() > 400 && Motor_Intake1.velocity(pct) < 5){
      Motor_Intake1.spin(directionType::fwd, (int)-10000, voltageUnits::mV);
      this_thread::sleep_for(300);
      stucked = 0;
    }
      
    if (fabs(intake_speed) > 100) intake_speed = sign(intake_speed) * 100;
    Motor_Intake1.spin(directionType::fwd, (int)130 * intake_speed, voltageUnits::mV);
    this_thread::sleep_for(5);
  }
}

bool IsTriggerStart(){
  return pistonS_start;
}

int getShootCount(){
  return shoot_count;
}

void clearShootCount(){
  shoot_count = 0;
}

int pistonInterval = PISTONINTERVAL;
static bool autoaim = false, aim_reached = false;

void triggerStatus(bool status, int _pistonInterval, float lower_bound, float upper_bound){
  if (!shoot_status && status){ 
    shoot_start = true;
    clearShootCount();
  }
  flywheel_lower_bound = lower_bound;
  flywheel_upper_bound = upper_bound;
  shoot_status = status;
  pistonInterval = _pistonInterval;
}

void trigger(){
  while(true){
    float flywheelspeed = currFlywheelSpeed;
    if (shoot_start == true){
      if (flywheelSpeedInRange(flywheelspeed, flywheel_lower_bound, flywheel_upper_bound)){
        setPistonS(true);
        shoot_count += 1;
        trigger_timer.reset();
        shoot_start = false;
        cout << flywheelspeed << ", " << flywheel_lower_bound << ", " << flywheel_upper_bound << endl;
      }
    }
    else if (shoot_status == true){
      if (trigger_timer.getTime() > pistonInterval && shoot_count < 3 && flywheelSpeedInRange(flywheelspeed, flywheel_lower_bound, flywheel_upper_bound)){
        pistonS_start = 1;
        if (shoot_count == 2)
          this_thread::sleep_for(50);
        setPistonS(true);
        shoot_count += 1;
        trigger_timer.reset();
      }
      else if (trigger_timer.getTime() > PISTONPUSHTIME){
        pistonS_start = 0;
        setPistonS(false);
      }
    }
    else{
      if (trigger_timer.getTime() > PISTONPUSHTIME){
        setPistonS(false);
        pistonS_start = 0;
      }
    }
    this_thread::sleep_for(5);
  }
}

void speed_volt(float v, int x) {
  MyTimer autotimer;
  autotimer.reset();
  
  Motor_Flywheel1.spin(directionType::fwd, (int)130 * v, voltageUnits::mV);
  this_thread::sleep_for(8000);
  float averagespeed = 0;
  autotimer.reset();
  while(autotimer.getTime() < 15000) {
    Motor_Flywheel1.spin(directionType::fwd, (int)130 * v, voltageUnits::mV);
    float speed = Motor_Flywheel1.velocity(rpm) * 6;
    averagespeed += speed;
    this_thread::sleep_for(5);
  }
  averagespeed = averagespeed / 3000;

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Voltage: %.1f Speed: %.1f ", v, averagespeed);
}

void set_aim_status(bool aim_bool){
  autoaim = aim_bool;
  if (aim_bool == false) aim_reached = true;
}

void auto_aim(){
  float aim_output;
  auto pid = PID();
  pid.setTarget(AIMSHOOTPIXEL);
  pid.setDTolerance(5.0);
  pid.setErrorTolerance(1.0);

  while (1){
    if (autoaim){
     get_signature_coor();
      if (vision_obj_x > -100) {
        float xdiff = fabs(vision_obj_x-AIMSHOOTPIXEL);
        pid.setCoefficient(0.0045*xdiff+0.45, 0.0, 10.0);
        pid.update(vision_obj_x);
        aim_output = -pid.getOutput();
        if(pid.targetArrived()) {
          lockBase();
          this_thread::sleep_for(200);
          aim_reached = true;
          pid = PID();
          pid.setTarget(AIMSHOOTPIXEL);
          pid.setDTolerance(5.0);
          pid.setErrorTolerance(1.0);
        }
        else if (xdiff > 1.0) { 
          aim_reached = false;
          moveClockwise(aim_output);
        }
        Brain.Screen.setCursor(12, 1);
        Brain.Screen.print("x: %.1f  y %.1f  h: %.1f w: %.1f ", vision_obj_x, vision_obj_y, vision_obj_height, vision_obj_width);
      } 
    } 
    else aim_reached = false;
    this_thread::sleep_for(2);
  }
}

float vision_set_flywheel_speed(){
  get_signature_coor();
  if (vision_obj_x > -100 && vision_obj_y>75.0) {
    float flywheelspeed = FLYWHEELSPEED-5.0*(vision_obj_y-70.0)+0.255*(vision_obj_y-75.0)*(vision_obj_y-75.0);
    setFlywheelSpeed(flywheelspeed, 0);
    if (flywheelspeed-FLYWHEELSPEED>0) this_thread::sleep_for(flywheelspeed-FLYWHEELSPEED);
    return flywheelspeed;
  }
  return flywheel_rotation_speed;
}

bool set_vision_data(signature s){
    float obj_x =0, obj_y=0, obj_height=0, obj_width =0, cnt=0;
    float ratio = -1.0;

    for (int i=0; i<5; i++) {
      Vision18.takeSnapshot(s);
      if (Vision18.largestObject.exists) {
        obj_x += Vision18.largestObject.centerX;
        obj_y += Vision18.largestObject.centerY;
        obj_height += Vision18.largestObject.height;
        obj_width += Vision18.largestObject.width;
        cnt += 1.0;
      }
    }
 
    if (cnt >0.5) {
      vision_obj_x = obj_x /cnt;
      vision_obj_y = obj_y /cnt;
      vision_obj_height = obj_height /cnt;
      vision_obj_width = obj_width /cnt;
      ratio  = vision_obj_height/vision_obj_width ;
    }

    if (ratio >0.2 && ratio < 0.45 && vision_obj_y >70.0 && vision_obj_y <125.0) return true;
    return false;
}

void get_signature_coor() {
  // Andrew as a code practice,  can you modify this to iterate through all signatures in the array you just defined?
  // forget about color just compare with all signatures
    if (target_blue > 0 ){
      if (set_vision_data(Vision18__SIG_1)==true) return;
      if (set_vision_data(Vision18__SIG_2)==true) return;
      if (set_vision_data(Vision18__SIG_5)==true) return;
    } 
    if (target_blue != 1 ){
      if (set_vision_data(Vision18__SIG_3)==true) return;
      if (set_vision_data(Vision18__SIG_4)==true) return;
    }
    vision_obj_x =-200.0;
    vision_obj_y =-200.0;
    return;
}
