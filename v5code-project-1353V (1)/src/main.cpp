#include "vex.h"
#include "robot-config.h"
#include <math.h>

using namespace vex;

competition Competition;

task chassisControlTask;
task intakeTask;

void coastDrivetrain(){
  DriveMotorLeftFront.stop(brakeType::coast);
  DriveMotorLeftMiddle.stop(brakeType::coast);
  DriveMotorLeftBack.stop(brakeType::coast);
  DriveMotorRightFront.stop(brakeType::coast);
  DriveMotorRightMiddle.stop(brakeType::coast);
  DriveMotorRightBack.stop(brakeType::coast);
}

void holdDrivetrain(){
  DriveMotorLeftFront.stop(brakeType::hold);
  DriveMotorLeftMiddle.stop(brakeType::hold);
  DriveMotorLeftBack.stop(brakeType::hold);
  DriveMotorRightFront.stop(brakeType::hold);
  DriveMotorRightMiddle.stop(brakeType::hold);
  DriveMotorRightBack.stop(brakeType::hold);
}

void brakeDrivetrain(){
  DriveMotorLeftFront.stop(brakeType::brake);
  DriveMotorLeftMiddle.stop(brakeType::brake);
  DriveMotorLeftBack.stop(brakeType::brake);
  DriveMotorRightFront.stop(brakeType::brake);
  DriveMotorRightMiddle.stop(brakeType::brake);
  DriveMotorRightBack.stop(brakeType::brake);
}

void intake(double velocity) { // Spin intake at the specified speed
  IntakeMotor.spin(directionType::fwd, velocity, velocityUnits::pct);
}

void intakeStop() { // Stop the intake
  IntakeMotor.stop(brakeType::brake);
}

void lift(double velocity) { // Spin the lift at the specified speed
  LiftMotor.spin(directionType::fwd, velocity, velocityUnits::rpm);
}

void liftStop() { // Stop the lift
  LiftMotor.stop(brakeType::brake);
}

bool driveBrakeToggle = false;

void swapBrakeScheme(){
  driveBrakeToggle = !driveBrakeToggle;
}

void resetAllMotors(){
  DriveMotorLeftFront.resetRotation();
  DriveMotorLeftMiddle.resetRotation();
  DriveMotorLeftBack.resetRotation();
  DriveMotorRightFront.resetRotation();
  DriveMotorRightMiddle.resetRotation();
  DriveMotorRightBack.resetRotation();
}

/*****************************************/
/*               PRE-AUTON               */
/*****************************************/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Inertial.calibrate();
  resetAllMotors();

  LiftMotor.setPosition(0, degrees);
  Inertial.setHeading(0, rotationUnits::deg);
  Inertial.setRotation(0, rotationUnits::deg);
}

/******************************************/
/*                                        */
/*               AUTONOMOUS               */
/*                                        */
/******************************************/

// Robot starts at (0,0) with a heading of 0. The robot takes the linear distance 
// traveled as well as its angle to approximate its position on the field. By taking 
// the sine and cosine components of the distance traveled, we can approximate the x 
// and y distance traveled on a 2D plane.

int calculatePosition(){
  return ((DriveMotorLeftFront.position(degrees) + DriveMotorLeftMiddle.position(degrees) + 
          DriveMotorLeftBack.position(degrees) + DriveMotorRightFront.position(degrees) +
          DriveMotorRightMiddle.position(degrees) + DriveMotorRightBack.position(degrees))
          / 6) / 360 * WHEEL_CIRCUMFRENCE;
}

// Target destination to drive to
double targetPosition = 0;
double targetFacingAngle = 0;
double targetLiftAngle = 0;

double currentPosition;
double currentFacingAngle;
double currentLiftAngle;

double maxAllowedSpeed = 1.0;

bool runChassisControl = false;
bool resetDrive = false;

double timeOutValue = 2500;

double LIFT_FLOOR_DISTANCE = 0;

// targetPos in 
void driveTo(double targetPos, double targetAngle, double timeOutLength, double maxSpeed){
  targetPosition = targetPos;
  targetFacingAngle = targetAngle;

  resetAllMotors();

  runChassisControl = true;

  timeOutValue = timeOutLength;

  Brain.resetTimer();

  maxAllowedSpeed = maxSpeed;
}

void turnToAngle(double targetAngle, double timeOutLength){
  targetFacingAngle = targetAngle;
  targetPosition = 0;

  resetAllMotors();

  runChassisControl = true;

  timeOutValue = timeOutLength;

  Brain.resetTimer();
}

double driveError = 0;
double drivePrevError = 0;

double driveMaxError = 0.1;

double driveIntegral = 0;
double driveIntegralBound = 1.5;

double driveDerivative = 0;

double drivekP = 2.0;
double drivekI = 10;
double drivekD = 2.5;

double drivePowerPID = 0;

void drivePID(){

  if(resetDrive){
    resetAllMotors();
  }

  // Current position is equal to the average position of all drive motors
  currentPosition = calculatePosition();

  // Error is equal to the total distance away from the target
  driveError = targetPosition - currentPosition;

/*
  // Only use integral if close enough to target
  if(fabs(driveError) < driveIntegralBound){
    driveIntegral += driveError;
  } else {
    driveIntegral = 0;
  }

  // Reset integral if we pass the target
  if(driveError * drivePrevError < 0){
    driveIntegral = 0;
  }
*/
  driveDerivative = driveError - drivePrevError;

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  // Limit power output to 12V
  if(drivePowerPID > 12) {
    drivePowerPID = 12;
  }

  if(fabs(driveError) < driveMaxError){
    drivePowerPID = 0;
  }

}

double turnError = 0;
double turnPrevError = 0;

double turnMaxError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.09;

double turnDerivative = 0;

double turnkP = 1.0;
double turnkI = 1;
double turnkD = 1;

double turnPowerPID = 0;

void turnPID(){
  resetAllMotors();

  // Current facing direction is equal to the inertial sensor heading
  currentFacingAngle = Inertial.heading();

  // Error is equal to the current facing direction and the target direction
  turnError = currentFacingAngle - targetFacingAngle;

  turnIntegral += turnError;

/*
  if(fabs(turnError) > M_PI){
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * M_PI - turnError);
  }

  // Only use integral if close enough to target
  if(fabs(turnError) < turnIntegralBound){
    turnIntegral += turnError;
  } else {
    turnIntegral = 0;
  }
*/

  // Reset integral if we pass the target
  if(turnError * turnPrevError < 0){
    turnIntegral = 0;
  }

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPowerPID = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  // Limit power output to 12V
  if(turnPowerPID > 12){
    turnPowerPID = 12;
  }

  if(fabs(turnError) < turnMaxError){
    turnPowerPID = 0;
  }
}

double leftDrivePower = 0;
double rightDrivePower = 0;

/* CHASSIS CONTROL TASK */
int chassisControl(){

  // Loop to constantly execute chassis commands
  while(1){

    if(runChassisControl){
      drivePID();
      turnPID();

      leftDrivePower = (drivePowerPID + turnPowerPID) * maxAllowedSpeed;
      rightDrivePower = (drivePowerPID - turnPowerPID) * maxAllowedSpeed;

      DriveMotorLeftFront.spin(directionType::fwd, leftDrivePower, voltageUnits::volt);
      DriveMotorLeftFront.spin(directionType::fwd, leftDrivePower, voltageUnits::volt);
      DriveMotorLeftFront.spin(directionType::fwd, leftDrivePower, voltageUnits::volt);
      DriveMotorRightFront.spin(directionType::fwd, rightDrivePower, voltageUnits::volt);
      DriveMotorRightFront.spin(directionType::fwd, rightDrivePower, voltageUnits::volt);
      DriveMotorRightFront.spin(directionType::fwd, rightDrivePower, voltageUnits::volt);

      if(fabs(driveError) < 0.1 && fabs(turnError) < 0.003){
        runChassisControl = false;
      }

      if(Brain.timer(timeUnits::msec) > timeOutValue){
        runChassisControl = false;
      }

      Brain.Screen.setCursor(3,2);
      Brain.Screen.print("drive error: %f", driveError);

      Brain.Screen.setCursor(5,2);
      Brain.Screen.print("turn error: %f", turnError);

      Brain.Screen.setCursor(7,2);
      Brain.Screen.print("Right Power: %f", rightDrivePower);

      Brain.Screen.setCursor(8,2);
      Brain.Screen.print("Left Power: %f", leftDrivePower);

      Brain.Screen.setCursor(9,2);
      Brain.Screen.print("Right Position: %f", DriveMotorRightFront.position(degrees));

      Brain.Screen.setCursor(10,2);
      Brain.Screen.print("Left Position: %f", DriveMotorLeftFront.position(degrees));
    }

    // What to do when not using the chassis controls
    else{
      DriveMotorLeftFront.stop(brakeType::coast);
      DriveMotorLeftMiddle.stop(brakeType::coast);
      DriveMotorLeftBack.stop(brakeType::coast);
      DriveMotorRightFront.stop(brakeType::coast);
      DriveMotorRightMiddle.stop(brakeType::coast);
      DriveMotorRightBack.stop(brakeType::coast);
    }

    task::sleep(20);

  }

  return 1;
}

bool runIntake = false;
int intakePower = 0;

int intakecontrol(){

  while(1){
    if(runIntake){
      intake(intakePower);
    } else {
      intakeStop();
    }
  }

  return 1;
}

void autoDriveTime(int left, int right, int time){
  DriveMotorLeftFront.spin(directionType::fwd, left, voltageUnits::mV);
  DriveMotorLeftFront.spin(directionType::fwd, left, voltageUnits::mV);
  DriveMotorLeftFront.spin(directionType::fwd, left, voltageUnits::mV);
  DriveMotorRightFront.spin(directionType::fwd, right, voltageUnits::mV);
  DriveMotorRightFront.spin(directionType::fwd, right, voltageUnits::mV);
  DriveMotorRightFront.spin(directionType::fwd, right, voltageUnits::mV);
  task::sleep(time);
  DriveMotorLeftFront.stop(brakeType::coast);
  DriveMotorLeftMiddle.stop(brakeType::coast);
  DriveMotorLeftBack.stop(brakeType::coast);
  DriveMotorRightFront.stop(brakeType::coast);
  DriveMotorRightMiddle.stop(brakeType::coast);
  DriveMotorRightBack.stop(brakeType::coast);
  task::sleep(20);
}

bool runLiftControl = false;
double liftPower = 0;

// Test route to test auto functions
void testAuto(){
  
  task chassisControlTask(chassisControl);

  task::sleep(20);
  driveTo(24, 0, 5000, 1.0);
  waitUntil(runChassisControl == false);
  task::sleep(500);
  turnToAngle(M_PI, 5000);
  
  /*
  bool a = true;
  DriveChassisController.driveFor(directionType::fwd, 12, distanceUnits::in, 210, velocityUnits::pct, a);
  waitUntil(a==false);
  DriveChassisController.driveFor(directionType::rev, 12, distanceUnits::in, 10, velocityUnits::pct, a);
  */
}

// Score rings in alliance goal
void autoWinPointLeft(){
  task::sleep(20);
}

// Score rings in alliance goal and move it off the line
void autoWinPointRight(){
  task::sleep(20);
}

// Score rings in both alliance goals and move it off the line
void autoWinPointTotal(){
  task::sleep(20);
}

// Rush to right neutral goal and bring it back
void goalRushRight(){
  /*
  task chassisControlTask(chassisControl);

  driveTo(80, 0, 5000, 1.0);
  PneumaticFrontClaw.set(true);
  waitUntil(runChassisControl == false);

  PneumaticFrontClaw.set(false);
  driveTo(-50, 0, 5000, 1.0);
  waitUntil(runChassisControl == false);

  turnToAngle(-9*M_PI, 2500);
  waitUntil(runChassisControl == false);
  PneumaticBackClamp_1.set(true);
  PneumaticBackClamp_2.set(true);

  driveTo(-40, 0, 5000, 1.0);
  */
  
  PneumaticFrontClaw.set(true);
  autoDriveTime(12000,12000,1600);
  
  PneumaticFrontClaw.set(false);
  autoDriveTime(-12000,-12000,2000);

  LiftMotor.spin(directionType::fwd, 50, velocityUnits::pct);
  task::sleep(1000);
  LiftMotor.stop(brakeType::hold);

  autoDriveTime(-12000,12000,1000);
  PneumaticBackClamp_1.set(true);
  PneumaticBackClamp_2.set(true);
}

// Rush to middle neutral goal and bring it back
void goalRushMiddle(){
  task::sleep(20);
}

// Rush to left neutral goal and bring it back
void goalRushLeft(){
  PneumaticFrontClaw.set(true);
  autoDriveTime(12000,12000,1800);
  
  PneumaticFrontClaw.set(false);
  autoDriveTime(-12000,-12000,6000);
}

// Rush to right neutral goal and bring it back
// Score rings in alliance goal and move it off the line
void goalRushRightAWP(){
  PneumaticFrontClaw.set(true);
  autoDriveTime(12000,12000,1600);
  
  PneumaticFrontClaw.set(false);
  autoDriveTime(-12000,-12000,2000);

  LiftMotor.spin(directionType::fwd, 50, velocityUnits::pct);
  task::sleep(1000);
  LiftMotor.stop(brakeType::hold);

  autoDriveTime(-12000,12000,1000);
  PneumaticBackClamp_1.set(true);
  PneumaticBackClamp_2.set(true);

  autoDriveTime(-12000, -12000, 1000);
  PneumaticBackClamp_1.set(false);
  PneumaticBackClamp_2.set(false);

  IntakeMotor.spin(directionType::fwd, 75, velocityUnits::pct);
}

// Rush to left neutral goal and bring it back
// Score rings in alliance goal
void goalRushLeftAWP(){
  task::sleep(20);
}

// Rush to right neutral goal and bring it back
// Go to center neutral goal and bring it back
void goalRushRightMid(){
  task::sleep(20);
}

// Rush to left neutral goal and bring it back
// Go to center neutral goal and bring it back
void goalRushLeftMid(){
  task::sleep(20);
}

void autoSkills(){
  PneumaticFrontClaw.set(true);
  autoDriveTime(12000,12000,1600);
  
  PneumaticFrontClaw.set(false);
  autoDriveTime(-12000,-12000,2000);

  LiftMotor.spin(directionType::fwd, 50, velocityUnits::pct);
  task::sleep(1000);
  LiftMotor.stop(brakeType::hold);

  autoDriveTime(-12000,12000,1000);
  PneumaticBackClamp_1.set(true);
  PneumaticBackClamp_2.set(true);

  autoDriveTime(-12000, -12000, 1000);
  PneumaticBackClamp_1.set(false);
  PneumaticBackClamp_2.set(false);

  IntakeMotor.spin(directionType::fwd, 75, velocityUnits::pct);
  task::sleep(4000);

  autoDriveTime(12000,-12000,2000);
  autoDriveTime(12000, 12000, 5000);
}

void autonomous(void) {
  //testAuto();
  //autoWinPointLeft();
  //autoWinPointRight();
  //autoWinPointTotal();
  //goalRushRight();
  //goalRushMiddle();
  goalRushLeft();
  //goalRushRightAWP(); <----
  //goalRushLeftAWP();
  //goalRushRightMid();
  //goalRushLeftMid();
  //autoSkills();
}

/**************************************/
/*                                    */
/*               DRIVER               */
/*                                    */
/**************************************/

void manualDriverControl(){
  if((abs(Controller.Axis3.value()) > 10) || (abs(Controller.Axis4.value()) > 10)){ // Sets a deadzone of 10 for better handling 
    // Left controller stick controls forward-backward movement, right controller stick controls turning movement
    double leftDrivePower = 0.12 * (Controller.Axis3.value() + Controller.Axis4.value());
    double rightDrivePower = 0.12 * (Controller.Axis3.value() - Controller.Axis4.value());
    //leftDrivePower = pow(leftDrivePower, 3) / 1000;
    //rightDrivePower = pow(leftDrivePower, 3) / 1000;

    DriveMotorLeftFront.spin(directionType::fwd, leftDrivePower, velocityUnits::pct);
    DriveMotorLeftMiddle.spin(directionType::fwd, leftDrivePower, velocityUnits::pct);
    DriveMotorLeftBack.spin(directionType::fwd, leftDrivePower, velocityUnits::pct);
    DriveMotorRightFront.spin(directionType::fwd, rightDrivePower, velocityUnits::pct);
    DriveMotorRightMiddle.spin(directionType::fwd, rightDrivePower, velocityUnits::pct);
    DriveMotorRightBack.spin(directionType::fwd, rightDrivePower, velocityUnits::pct);
  } else if (abs(Controller.Axis2.value()) > 10){
    double drivePower = Controller.Axis2.value();
    DriveMotorLeftFront.spin(directionType::fwd, drivePower, velocityUnits::pct);
    DriveMotorLeftMiddle.spin(directionType::fwd, drivePower, velocityUnits::pct);
    DriveMotorLeftBack.spin(directionType::fwd, drivePower, velocityUnits::pct);
    DriveMotorRightFront.spin(directionType::fwd, drivePower, velocityUnits::pct);
    DriveMotorRightMiddle.spin(directionType::fwd, drivePower, velocityUnits::pct);
    DriveMotorRightBack.spin(directionType::fwd, drivePower, velocityUnits::pct);
  } else { // Otherwise, stop drivetrain
    if(driverBrakeToggle){ holdDrivetrain(); }
    else{ coastDrivetrain(); }
  }
}

void manualLiftControl(){
  if(Controller.ButtonL1.pressing()){ // When L1 is pressed, raise the lift (full speed)
    lift(100);
  } else if(Controller.ButtonL2.pressing() && LiftMotor.position(deg) != 0){ // When L2 is pressed, raise the lift (half speed)
    lift(-100);
  } else { // Otherwise, keep the lift in place
    liftStop();
  }
}

void manualIntakeControl(){
  if(Controller.ButtonR1.pressing()){ // When R1 is pressed, intake (full speed)
    intake(-50);
  } else if(Controller.ButtonR2.pressing()){ // When R1 and R2 are pressed, outtake (half speed)
    intake(100);
  } else{ // Otherwise, stop the intake
    intakeStop();
  }
}

void manualFrontClawControl(){ 
  if(Controller.ButtonX.pressing()){ // When X is pressed, close the claw
    PneumaticFrontClaw.set(true);
  } else if(Controller.ButtonY.pressing()){ // When Y is pressed, open the claw
    PneumaticFrontClaw.set(false);
  }
}

void manualBackClampControl(){
  if(Controller.ButtonA.pressing()){ // When A is pressed, close the back clamp
    PneumaticBackClamp_1.set(true);
    PneumaticBackClamp_2.set(true);
  } else if(Controller.ButtonB.pressing()){ // When B is pressed, open the back clamp
    PneumaticBackClamp_1.set(false);
    PneumaticBackClamp_2.set(false);
  }
}

void usercontrol(void) {

  while (1) {
    manualDriverControl();
    manualIntakeControl();
    manualLiftControl();
    manualFrontClawControl();
    manualBackClampControl();
    
    task::sleep(20);
  }
}

/************************************/
/*                                  */
/*               MAIN               */
/*                                  */
/************************************/

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}