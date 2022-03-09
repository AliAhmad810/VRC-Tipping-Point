using namespace vex; 

extern brain Brain;
extern controller Controller;

extern double WHEEL_DIAMETER;
extern double PI;
extern double WHEEL_CIRCUMFRENCE;

extern bool driverBrakeToggle;

extern motor DriveMotorLeftFront;
extern motor DriveMotorLeftMiddle;
extern motor DriveMotorLeftBack;
extern motor DriveMotorRightFront;
extern motor DriveMotorRightMiddle;
extern motor DriveMotorRightBack;
extern motor LiftMotor;
extern motor IntakeMotor;

extern digital_out PneumaticFrontClaw;
extern digital_out PneumaticBackClamp_1;
extern digital_out PneumaticBackClamp_2;

extern inertial Inertial;

extern motor_group LeftDriveMotorGroup;
extern motor_group RightDriveMotorGroup;

extern smartdrive TurningChassisController;
extern drivetrain DriveChassisController;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 *
 */
void vexcodeInit(void);