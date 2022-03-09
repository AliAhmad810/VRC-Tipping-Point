#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// VEXcode Devices
controller Controller = controller(primary);

// Global variables and constants
double WHEEL_DIAMETER = 3.25; // Diameter of the wheel (in INCHES)
double PI = M_PI; // Pi
double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * PI; // Calculation to find the circumfrence of the wheels

bool driverBrakeToggle = false;

// Motors
motor DriveMotorLeftFront(PORT12, ratio6_1, true);
motor DriveMotorLeftMiddle(PORT11, ratio6_1, false);
motor DriveMotorLeftBack(PORT13, ratio6_1, true);
motor DriveMotorRightFront(PORT1, ratio6_1, false);
motor DriveMotorRightMiddle(PORT19, ratio6_1, true);
motor DriveMotorRightBack(PORT17, ratio6_1, false);
motor LiftMotor(PORT6, ratio36_1, false);
motor IntakeMotor(PORT9, ratio6_1, false);

// Pneumatics
digital_out PneumaticFrontClaw = digital_out(Brain.ThreeWirePort.G);
digital_out PneumaticBackClamp_1 = digital_out(Brain.ThreeWirePort.E);
digital_out PneumaticBackClamp_2 = digital_out(Brain.ThreeWirePort.C);

// Rotation
inertial Inertial(PORT20);

// Create motor group objects for the left and right sides of the drive train
motor_group LeftDriveMotorGroup(DriveMotorLeftFront, DriveMotorLeftMiddle, DriveMotorLeftBack);
motor_group RightDriveMotorGroup(DriveMotorRightFront, DriveMotorRightMiddle, DriveMotorRightBack);

// Create a smart drive object for autonomous purposes
smartdrive TurnChassisController(LeftDriveMotorGroup, RightDriveMotorGroup, Inertial, WHEEL_CIRCUMFRENCE, 11.5, 11.25, distanceUnits::in, 0.6);
drivetrain DriveChassisController(LeftDriveMotorGroup, RightDriveMotorGroup, WHEEL_CIRCUMFRENCE, 11.5, 11.25, distanceUnits::in, 0.6);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}