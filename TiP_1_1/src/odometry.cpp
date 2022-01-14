#include "odometry.h"

// CONSTANTS / Hard-Coded Values
// Radius of tracking wheels in inches
double WHEEL_RADIUS = 2.0;

// Starting angle (relative to field) (RADIANS)
double THETA_START = M_PI;

// The starting x and y coordinates of the bot (INCHES)
  // These distances are relative to some point (0,0) on the field
  // Relative to: BOTTOM LEFT CORNER
double X_START = 0;
double Y_START = 0;

// Distances of tracking wheels from tracking center (INCHES)
double LTrackRadius = 0;
double RTrackRadius = 0;
double STrackRadius = 0;

// Calculated Values (every loop)
// Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
double LPos = 0;
double RPos = 0;
double SPos = 0;

double LPrevPos = 0;
double RPrevPos = 0;
double SPrevPos = 0;

// Distances traveled by tracking wheels each loop (INCHES)
double deltaDistL = 0;
double deltaDistR = 0;
double deltaDistS = 0;

// Distance summations (since last reset)
double totalDeltaDistL = 0;
double totalDeltaDistR = 0;

// The current angle of the bot (RADIANS)
double currentAbsoluteOrientation = THETA_START;
// The previous angle of the bot (RADIANS)
double previousTheta = THETA_START;

// The change in Theta each loop (RADIANS)
double deltaTheta = 0;

// The averange angle Theta (IN RADIANS) throughout the arc
  // currentAbsoluteOrientation + (deltaTheta / 2)
double avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

// The changes in the X and Y positions (INCHES)
/* These are calculated on a local basis each loop, 
then converted to global position changes */
double deltaXLocal = 0;
double deltaYLocal = 0;

// The X and Y offsets converted from their local forms (INCHES)
double deltaXGlobal = 0;
double deltaYGlobal = 0;

// The global position of the bot (INCHES)
double xPosGlobal = X_START;
double yPosGlobal = Y_START;

int positionTracking(){
  while (1){

    // Get encoder values (DEGREES)
    LPos = LTrack.rotation(rotationUnits::deg);
    RPos = RTrack.rotation(rotationUnits::deg);
    SPos = STrack.rotation(rotationUnits::deg);

    // Calculate distance traveled by tracking each wheel (INCHES)
      // Converts degrees to radians
    deltaDistL = ((LPos - LPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    deltaDistR = ((RPos - RPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    deltaDistS = ((SPos - SPrevPos) * M_PI / 180) * WHEEL_RADIUS;

    // Update previous values to be used next loop (DEGREES)
    LPrevPos = LPos;
    RPrevPos = RPos;
    SPrevPos = SPos;

    // Total change in each of the L and R encoders since last reset (INCHES)
    // These are used to calculate the absolute orientation of the bot
    totalDeltaDistL += deltaDistL;
    totalDeltaDistR += deltaDistR;

    // Calculate the current absolute orientation (RADIANS)
    // currentAbsoluteOrientation = THETA_START - ((totalDeltaDistL - totalDeltaDistR) / (LTrackRadius + RTrackRadius));
    currentAbsoluteOrientation = (360 - Inertial.heading(rotationUnits::deg)) * M_PI / 180.0;

    // Calculate the change in the angle of the bot (RADIANS)
    deltaTheta = currentAbsoluteOrientation - previousTheta;

    // Update the previous Theta value (RADIANS)
    previousTheta = currentAbsoluteOrientation;

    // If we didn't turn, then we only translated
    if(deltaTheta == 0){
      deltaXLocal = deltaDistS;
      // Could be either L or R, since if deltaTheta == 0, we assume they're =
      deltaYLocal = deltaDistL;
    }
    // Else, calculate the new local position
    else{
      // Calculate the changes in the X and Y values (INCHES)
      // General equation is:
        //Distance = 2 * Radius * sin(deltaTheta / 2)
      deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistS / deltaTheta) + STrackRadius);
      deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistR / deltaTheta) - RTrackRadius);
    }

    // The average angle of the robot during it's arc (RADIANS)
    avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

    deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
    deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));

    // Wraps angles back around if they ever go under 0 or over 2 pi
    while(currentAbsoluteOrientation >= 2 * M_PI){
      currentAbsoluteOrientation -= 2 * M_PI;
    }

    xPosGlobal += deltaXGlobal;
    yPosGlobal += deltaYGlobal;

    // Loop every 10 milliseconds
    task::sleep(10);
  }
  return 1;
}