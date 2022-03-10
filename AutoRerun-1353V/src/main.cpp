#include "vex.h"
#include "robot-config.h"
#include <math.h>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

using namespace vex;

competition Competition;

/* START OF AUTONOMOUS INDEX */

// Skills auto
int skillsAuto[3000][11] = {};

/* END OF AUTONOMOUS INDEX */

std::ofstream ofs;

std::string toString(int val){
  std::stringstream stream;
  stream << val;
  return stream.str();
}

std::string formatToString(int num){
  if(num == 100){
    return "100,";
  } else if((num>=10) && (num<10)){
    return (toString(num) + ",");
  } else {
    return (toString(num) + ",");
  }
}

// Putting motor values into a spot in a set. Later, the robot will draw from the
// set, based on the motor's position in the set. For instance, let us assume that
// motor x is assigned to position 2 in the set. A set that reads 
// {50, 50, 100, 100, 25, 75, 0, 0, true, false, false} will assign the motor value 
// 100 to motor x

void runMappedAuto(int start, int finish, int auton[3000][11]){
  int i = start;
  while(i < finish){
    if(auton[i][0] != 0){ // Index 0 is front left drive motor
      DriveMotorLeftFront.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      DriveMotorLeftFront.stop();
    }

    if(auton[i][1] != 0){ // Index 1 is middle left drive motor
      DriveMotorLeftMiddle.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      DriveMotorLeftMiddle.stop();
    }

    if(auton[i][2] != 0){ // Index 2 is back left drive motor
      DriveMotorLeftBack.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      DriveMotorLeftBack.stop();
    }

    if(auton[i][3] != 0){ // Index 3 is front right drive motor
      DriveMotorRightFront.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      DriveMotorRightFront.stop();
    }

    if(auton[i][4] != 0){ // Index 4 is middle right drive motor
      DriveMotorRightMiddle.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      DriveMotorRightMiddle.stop();
    }

    if(auton[i][5] != 0){ // Index 5 is back right drive motor
      DriveMotorRightBack.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      DriveMotorRightBack.stop();
    }

    if(auton[i][6] != 0){ // Index 6 is lift motor
      LiftMotor.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      LiftMotor.stop();
    }

    if(auton[i][7] != 0){ // Index 7 is ring intake motor
      IntakeMotor.spin(directionType::fwd, auton[i][0], velocityUnits::pct);
    } else {
      IntakeMotor.stop();
    }

    if(auton[i][8] == 1){ // Index 8 is pneumatic front claw
      PneumaticFrontClaw.set(true);
    } else {
      PneumaticFrontClaw.set(false);
    }

    if(auton[i][9] == 1){ // Index 9 is pneumatic back clamp 1
      PneumaticBackClamp_1.set(true);
    } else {
      PneumaticBackClamp_1.set(false);
    }

    if(auton[i][10] == 1){ // Index 10 is pneumatic back clamp 2
      PneumaticBackClamp_2.set(true);
    } else {
      PneumaticBackClamp_2.set(false);
    }

    task::sleep(20);
    i++;
  }
}

// Stops motors before autonomous

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  DriveMotorLeftFront.stop(brakeType::brake);
  DriveMotorLeftMiddle.stop(brakeType::brake);
  DriveMotorLeftBack.stop(brakeType::brake);
  DriveMotorRightFront.stop(brakeType::brake);
  DriveMotorRightMiddle.stop(brakeType::brake);
  DriveMotorRightBack.stop(brakeType::brake);
  LiftMotor.stop(brakeType::brake);
  IntakeMotor.stop(brakeType::brake);
}

// Stops motors, then starts to run the autonomous
// program written above

void autonomous(void) {
  DriveMotorLeftFront.stop(brakeType::brake);
  DriveMotorLeftMiddle.stop(brakeType::brake);
  DriveMotorLeftBack.stop(brakeType::brake);
  DriveMotorRightFront.stop(brakeType::brake);
  DriveMotorRightMiddle.stop(brakeType::brake);
  DriveMotorRightBack.stop(brakeType::brake);
  LiftMotor.stop(brakeType::brake);
  IntakeMotor.stop(brakeType::brake);
  runMappedAuto(0, 3000, skillsAuto);
}

void usercontrol(void){

  while(true){

    if(Controller.ButtonLeft.pressing()){
      ofs.open("auto1.txt", std::ofstream::out);
      int i = 0;
      while(true && i < 3000){
        if(Brain.SDcard.isInserted()){
          if((abs(Controller.Axis1.value()) > 10 || (abs(Controller.Axis3.value()) > 10))){
            int leftSideValue = (Controller.Axis3.value() + Controller.Axis1.value()) * 0.5;
            int rightSideValue = (Controller.Axis3.value() - Controller.Axis1.value()) * 0.5;

            DriveMotorLeftFront.spin(directionType::fwd, leftSideValue, velocityUnits::pct);
            DriveMotorLeftMiddle.spin(directionType::fwd, leftSideValue, velocityUnits::pct);
            DriveMotorLeftBack.spin(directionType::fwd, leftSideValue, velocityUnits::pct);
            DriveMotorRightFront.spin(directionType::fwd, rightSideValue, velocityUnits::pct);
            DriveMotorRightMiddle.spin(directionType::fwd, rightSideValue, velocityUnits::pct);
            DriveMotorRightBack.spin(directionType::fwd, rightSideValue, velocityUnits::pct);
          } else {
            DriveMotorLeftFront.stop(brakeType::brake);
            DriveMotorLeftMiddle.stop(brakeType::brake);
            DriveMotorLeftBack.stop(brakeType::brake);
            DriveMotorRightFront.stop(brakeType::brake);
            DriveMotorRightMiddle.stop(brakeType::brake);
            DriveMotorRightBack.stop(brakeType::brake);
          }

          if(Controller.ButtonL1.pressing()){
            LiftMotor.spin(directionType::fwd, 100, velocityUnits::pct);
          } else if(Controller.ButtonL2.pressing() && LiftMotor.position(deg) != 0){
            LiftMotor.spin(directionType::fwd, -100, velocityUnits::pct);
          } else {
            LiftMotor.stop(brakeType::hold);
          }

          if(Controller.ButtonR1.pressing()){
            IntakeMotor.spin(directionType::fwd, -50, velocityUnits::pct);
          } else if(Controller.ButtonR2.pressing()){
            IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
          } else {
            IntakeMotor.stop(brakeType::brake);
          }

          if(Controller.ButtonX.pressing()){
            PneumaticFrontClaw.set(true);
          } else if(Controller.ButtonY.pressing()){
            PneumaticFrontClaw.set(false);
          }

          if(Controller.ButtonA.pressing()){
            PneumaticBackClamp_1.set(true);
            PneumaticBackClamp_2.set(true);
          } else if(Controller.ButtonB.pressing()){
            PneumaticBackClamp_1.set(false);
            PneumaticBackClamp_2.set(false);
          }
          
          // Write motor values to SD Card
          ofs << "{";
            if((abs(Controller.Axis1.value()) > 10 || (abs(Controller.Axis3.value()) > 10))){
              ofs << formatToString((Controller.Axis3.position(percentUnits::pct) + Controller.Axis1.position(percentUnits::pct))*0.5);
              ofs << formatToString((Controller.Axis3.position(percentUnits::pct) + Controller.Axis1.position(percentUnits::pct))*0.5);
              ofs << formatToString((Controller.Axis3.position(percentUnits::pct) + Controller.Axis1.position(percentUnits::pct))*0.5);
              ofs << formatToString((Controller.Axis3.position(percentUnits::pct) + Controller.Axis1.position(percentUnits::pct))*0.5);
              ofs << formatToString((Controller.Axis3.position(percentUnits::pct) + Controller.Axis1.position(percentUnits::pct))*0.5);
              ofs << formatToString((Controller.Axis3.position(percentUnits::pct) + Controller.Axis1.position(percentUnits::pct))*0.5);
            } else {
              ofs << formatToString(0);
              ofs << formatToString(0);
              ofs << formatToString(0);
              ofs << formatToString(0);
              ofs << formatToString(0);
              ofs << formatToString(0);
            }

            if(Controller.ButtonL1.pressing()){
              ofs << formatToString(100);
            } else if(Controller.ButtonL2.pressing()){
              ofs << formatToString(-100);
            } else {
              ofs << formatToString(0);
            }

            if(Controller.ButtonR1.pressing()){
              ofs << formatToString(-50);
            } else if(Controller.ButtonR2.pressing()){
              ofs << formatToString(100);
            } else {
              ofs << formatToString(0);
            }

            if(Controller.ButtonX.pressing()){
              ofs << formatToString(1);
            } else if(Controller.ButtonY.pressing()){
              ofs << formatToString(0);
            } else {
              if(PneumaticFrontClaw.value()) ofs << formatToString(1);
              else ofs << formatToString(0);
            }

            if(Controller.ButtonA.pressing()){
              ofs << formatToString(1);
              ofs << formatToString(1);
            } else if(Controller.ButtonB.pressing()){
              ofs << formatToString(0);
              ofs << formatToString(0);
            } else {
              if(PneumaticBackClamp_1.value()){
                ofs << formatToString(1);
                ofs << formatToString(1);
              } else {
                ofs << formatToString(0);
                ofs << formatToString(0);
              }
            }

            ofs << "}";

        } else {
          Brain.Screen.print('A');
        }

        task::sleep(20);
        i++;
      }

      ofs.close();
    }

  }

}

int main(){
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();
  
  while(1){
    task::sleep(10);
  }
}