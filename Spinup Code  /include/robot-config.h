using namespace vex;
using signature = vision::signature;
extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Motor_BaseLF;
extern motor Motor_BaseLM;
extern motor Motor_BaseLB;
extern motor Motor_BaseRF;
extern motor Motor_BaseRM;
extern motor Motor_BaseRB;
extern motor Motor_Intake1;
extern motor Motor_Intake2;
extern motor Motor_Flywheel1;
extern motor Motor_Flywheel2;
extern led PistonS;
extern led PistonE;
extern led PistonA;
extern led PistonI;
extern inertial IMU;
// VEXcode devices
// Andew: as a code practice - can you modify this into an array of signatures?   
// eg array<signature, 5> signatures;
extern signature Vision18__SIG_1;
extern signature Vision18__SIG_2;
extern signature Vision18__SIG_3;
extern signature Vision18__SIG_4;
extern signature Vision18__SIG_5;
extern vision Vision18;





/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
