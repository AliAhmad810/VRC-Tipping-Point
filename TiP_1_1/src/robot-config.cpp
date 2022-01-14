#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// VEXcode device constructors
controller Controller = controller(primary);
motor FrontLeftDrive = motor(PORT1, gearSetting::ratio6_1, false);
motor BackLeftDrive = motor(PORT2, gearSetting::ratio6_1, false);
motor FrontRightDrive = motor(PORT3, gearSetting::ratio6_1, true);
motor BackRightDrive = motor(PORT4, gearSetting::ratio6_1, true);
motor LeftLift = motor(PORT5, gearSetting::ratio36_1, true);
motor RightLift = motor(PORT6, gearSetting::ratio36_1, false);
motor Claw = motor(PORT7, gearSetting::ratio36_1, true);
motor Tilter = motor(PORT8, gearSetting::ratio36_1, true);
pneumatics LRamp = pneumatics(Brain.ThreeWirePort.G);
pneumatics RRamp = pneumatics(Brain.ThreeWirePort.H);
encoder LTrack = encoder(Brain.ThreeWirePort.A);
encoder RTrack = encoder(Brain.ThreeWirePort.C);
encoder STrack = encoder(Brain.ThreeWirePort.E);
inertial Inertial = inertial(PORT9);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}