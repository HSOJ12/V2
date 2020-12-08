#include "vex.h"
#include "math.h"
//#include "robot-config.cpp"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller (primary);

//Front Intake
motor MotorIntakeR = motor (PORT9);
motor MotorIntakeL = motor (PORT12);

//Top Intake
motor MotorR = motor (PORT16);
motor MotorL = motor (PORT1);

//Drive Base
motor FrontRightDrive = motor (PORT19);
motor BackRightDrive = motor (PORT15);
motor FrontLeftDrive = motor (PORT11);
motor BackLeftDrive = motor (PORT13);

//Sensors
encoder DistanceEncoder = encoder (Brain.ThreeWirePort.A);
line BallSensor = line (Brain.ThreeWirePort.C);
inertial InertialSensor = inertial (PORT14);

//Constants
const float DistanceMultiplier = 1;
//Constants
/*const float RotationMultiplier = 41.7; //Degrees per Inch
const float R = 3; //Inches

void DriveForwards (int Distance)
{
  DistanceEncoder.setPosition(0, rotationUnits::deg);
  if ((RotationMultiplier * Distance) < DistanceEncoder.position(rotationUnits::deg))
  {
    //Drive Forwards
  }
}

void TurnLeft (int Angle)
{
  DistanceEncoder.setPosition(0, rotationUnits::deg);
  if (((R * Angle * M_PI) / (180 * RotationMultiplier)) < DistanceEncoder.position(rotationUnits::deg))
  {
    //Turn
  }
}*/


void pre_auton() 
{
  vexcodeInit();
}

void autonomous() 
{

}

void usercontrol() 
{
  //True False Code
  //bool SensorsOn = true;
  //bool ToggleSensors = false;
  int TurnVelocity = 0;
  int ForwardVelocity = 0;

  while (true) 
  {
    // Sensor Values
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor (1, 1);
    Brain.Screen.print ("Distance Encoder: %.0f Degrees", DistanceEncoder.position(rotationUnits::deg));
    Brain.Screen.setCursor (2, 1);
    Brain.Screen.print ("Ball Sensor: %d Percent", BallSensor.value(percentUnits::pct));
    Brain.Screen.setCursor (3, 1);
    Brain.Screen.setCursor (4, 1);
    Brain.Screen.print ("MotorVelocity %f", ForwardVelocity);
    Brain.Screen.render();
    //

    //Top Intake
    if (abs(Controller1.Axis2.position(percentUnits::pct)) > 10)
    {
      MotorR.spin (directionType::fwd, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
      MotorL.spin (directionType::fwd, -Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
    }
    else
    {
      MotorR.stop (brakeType::coast);
      MotorL.stop (brakeType::coast);
    }
    //

    //Front Intake
    if (Controller1.ButtonR1.pressing() == true)
    {
      MotorIntakeR.spin (directionType::fwd, -100, velocityUnits::pct);
      MotorIntakeL.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing() == true)
    {
      MotorIntakeR.spin (directionType::fwd, 100, velocityUnits::pct);
      MotorIntakeL.spin (directionType::fwd, -100, velocityUnits::pct);
    }
    else
    {
      MotorIntakeR.stop (brakeType::coast);
      MotorIntakeL.stop (brakeType::coast);
    }
    //

    if (ForwardVelocity < Controller1.Axis3.position(percentUnits::pct))
    {
      ForwardVelocity += 10;
      task::sleep (100);
    }    
    else if (ForwardVelocity > Controller1.Axis3.position(percentUnits::pct))
    {
      ForwardVelocity -= 10;
      task::sleep (1000);
    }
    else 
    {
      ForwardVelocity = 0;
    }

    if (TurnVelocity < Controller1.Axis4.position(percentUnits::pct))
    {
      TurnVelocity += 10;
      task::sleep (100);
    }
    else if (TurnVelocity > Controller1.Axis4.position(percentUnits::pct))
    {
      TurnVelocity -= 10;
      task::sleep (1000);
    }
    else 
    {
      TurnVelocity = 0;
    }

    //Drive Base
    if (abs(Controller1.Axis3.position(percentUnits::pct)) + abs(Controller1.Axis4.position(percentUnits::pct)) > 10)
    { 
      FrontRightDrive.spin (directionType::fwd, -ForwardVelocity + TurnVelocity, velocityUnits::pct);
      BackRightDrive.spin (directionType::fwd, -ForwardVelocity + TurnVelocity, velocityUnits::pct);
      FrontLeftDrive.spin (directionType::fwd, ForwardVelocity + TurnVelocity, velocityUnits::pct);
      BackLeftDrive.spin (directionType::fwd, ForwardVelocity + TurnVelocity, velocityUnits::pct);
    }
    else
    {
      FrontRightDrive.stop (brakeType::coast);
      BackRightDrive.stop (brakeType::coast);
      FrontLeftDrive.stop (brakeType::coast);
      BackLeftDrive.stop (brakeType::coast);
    }
    //
    
    //Turn Sensors On And Off
    /*if (Controller1.ButtonUp.pressing() == true)
    {
      ToggleSensors = true;
    }
    else if (Controller1.ButtonR1.pressing() == false && ToggleSensors == true)
    {
      if (SensorsOn == true)
      {
        SensorsOn = false;
      }
      else if (SensorsOn == false)
      {
        SensorsOn= true;
      }
      ToggleSensors = false;
    }*/
    //
  }
}

int main() 
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
}