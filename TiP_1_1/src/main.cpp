/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Tues Dec 28 2021                                          */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller           controller                    
// FrontLeftDrive       motor         1              
// BackLeftDrive        motor         2               
// FrontRightDrive      motor         3              
// BackRightDrive       motor         4                                       
// LeftLift             motor         5               
// LightLift            motor         6               
// Claw                 motor         7               
// Tilter               motor         8              
// LRamp                pneumatics    G            
// RRamp                pneumatics    H   
// LTrack               encoder       A
// RTrack               encoder       C
// STrack               encoder       E
// Inertial             inertial      9
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "robot-config.h"
#include <vex_triport.h>

using namespace vex;

competition Competition;

/*
*   Helper functions
*/

void brakeDrive(){
  FrontLeftDrive.stop(brakeType::brake);
  BackLeftDrive.stop(brakeType::brake);
  FrontRightDrive.stop(brakeType::brake);
  BackRightDrive.stop(brakeType::brake);
}

void coastDrive(){
  FrontLeftDrive.stop(brakeType::coast);
  BackLeftDrive.stop(brakeType::coast);
  FrontRightDrive.stop(brakeType::coast);
  BackRightDrive.stop(brakeType::coast);
}

void holdDrive(){
  FrontLeftDrive.stop(brakeType::hold);
  BackLeftDrive.stop(brakeType::hold);
  FrontRightDrive.stop(brakeType::hold);
  BackRightDrive.stop(brakeType::hold);
}

void powerDrive(double lPower, double rPower){
  FrontLeftDrive.spin(directionType::fwd, lPower, velocityUnits::pct);
  BackLeftDrive.spin(directionType::fwd, lPower, velocityUnits::pct);
  FrontRightDrive.spin(directionType::fwd, rPower, velocityUnits::pct);
  BackRightDrive.spin(directionType::fwd, rPower, velocityUnits::pct);
}

void powerLift(double p){

}

/*
*
*   Driver Functions
*
*/

void manualDriverControl(){
  if((abs(Controller.Axis4.value()) > 10) || (abs(Controller.Axis3.value()) > 10))
  {
    double leftPower = .001*pow(Controller.Axis3.value() - Controller.Axis4.value(),3);
    double rightPower = .001*pow(Controller.Axis3.value() + Controller.Axis4.value(),3);

    powerDrive(leftPower, rightPower);
  }
  else{
    coastDrive();
  }
}

void manualLiftControl(){

}