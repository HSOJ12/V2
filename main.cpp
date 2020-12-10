#include "vex.h"
#include "math.h"
//#include "robot-config.cpp"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller (primary);

//Front Intake
motor IntakeRBottom = motor (PORT9);
motor IntakeLBottom = motor (PORT12);

//Top Intake
motor IntakeRTop = motor (PORT16);
motor IntakeLTop = motor (PORT1);

//Drive Base
motor DriveRFront = motor (PORT19);
motor DriveRBack = motor (PORT15);
motor DriveLFront = motor (PORT11);
motor DriveLBack = motor (PORT13);

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
  //int TurnVelocity = 0;
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
      IntakeRTop.spin (directionType::fwd, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
      IntakeLTop.spin (directionType::fwd, -Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
    }
    else
    {
      IntakeRTop.stop (brakeType::coast);
      IntakeLTop.stop (brakeType::coast);
    }
    //

    //Front Intake
    if (Controller1.ButtonR1.pressing() == true)
    {
      IntakeRBottom.spin (directionType::fwd, -100, velocityUnits::pct);
      IntakeRBottom.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing() == true)
    {
      IntakeRBottom.spin (directionType::fwd, 100, velocityUnits::pct);
      IntakeLBottom.spin (directionType::fwd, -100, velocityUnits::pct);
    }
    else
    {
      IntakeRBottom.stop (brakeType::coast);
      IntakeLBottom.stop (brakeType::coast);
    }
    //

    if (ForwardVelocity < Controller1.Axis3.position(percentUnits::pct))
    {
      ForwardVelocity += 1;
      this_thread::sleep_for(100);
    }    
    else if (ForwardVelocity > Controller1.Axis3.position(percentUnits::pct))
    {
      ForwardVelocity -= 1;
      this_thread::sleep_for(100);
    }
    else if (ForwardVelocity == Controller1.Axis3.position(percentUnits::pct))
    {
        ForwardVelocity = Controller1.Axis3.position(percentUnits::pct);
        this_thread::sleep_for(100);
    }
    else 
    {
      ForwardVelocity = 0;
    }

    /*if (TurnVelocity < Controller1.Axis4.position(percentUnits::pct))
    {
      TurnVelocity += 1;
      task::sleep (100);
    }
    else if (TurnVelocity > Controller1.Axis4.position(percentUnits::pct))
    {
      TurnVelocity -= 1;
      task::sleep (100);
    }
    else 
    {
      TurnVelocity = 0;
    }*/

    //Drive Base
    if (abs(Controller1.Axis3.position(percentUnits::pct)) + abs(Controller1.Axis4.position(percentUnits::pct)) > 10)
    { 
      DriveRFront.spin (directionType::fwd, -ForwardVelocity + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
      DriveRBack.spin (directionType::fwd, -ForwardVelocity + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
      DriveLFront.spin (directionType::fwd, ForwardVelocity + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
      DriveLBack.spin (directionType::fwd, ForwardVelocity + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
    }
    else
    {
      DriveRFront.stop (brakeType::coast);
      DriveRBack.stop (brakeType::coast);
      DriveLFront.stop (brakeType::coast);
      DriveLBack.stop (brakeType::coast);
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