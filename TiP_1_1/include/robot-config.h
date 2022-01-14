using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;
extern motor FrontLeftDrive;
extern motor BackLeftDrive;
extern motor FrontRightDrive;
extern motor BackRightDrive;
extern motor LeftLift;
extern motor RightLift;
extern motor Claw;
extern motor Tilter;
extern pneumatics LRamp;
extern pneumatics RRamp;
extern encoder LTrack;
extern encoder RTrack;
extern encoder STrack;
extern inertial Inertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
