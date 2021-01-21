#include "vex.h"
#include "cmath"
//#include "robot-config.cpp"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller (primary);

//Front Intake
motor IntakeRBottom = motor (PORT18);
motor IntakeLBottom = motor (PORT13);

//Top Intake
motor IntakeLTop = motor (PORT2);
motor IntakeRTop = motor (PORT10);

//Drive Base
motor DriveRFront = motor (PORT20);
motor DriveRBack = motor (PORT19);
motor DriveLFront = motor (PORT11);
motor DriveLBack = motor (PORT12);

//Sensors
encoder DistanceEncoderLeft = encoder (Brain.ThreeWirePort.A);
encoder DistanceEncoderRight = encoder (Brain.ThreeWirePort.C);
line BallSensor = line (Brain.ThreeWirePort.E);
inertial InertialSensor = inertial (PORT14);
limit LimitBall = limit (Brain.ThreeWirePort.H);

double P = 1.1;
double Rad = M_PI/180;
double Deg = 180/M_PI;
double DR = 2;
double DL = 2;
double BaseMultiplier = 2.6;
double Multiplier = 25;

void DriveForwardsNoD (int MaxVelocity, int Distance, int IntakeOn)
{
  DriveLFront.resetPosition ();
  int Velocity = 10;
  task::sleep (100);
  while (DriveLFront.position (rotationUnits::deg) < (Distance * Multiplier))
  {
    if (Velocity >= MaxVelocity)
    {
      Velocity = MaxVelocity;
    }
    else
    {
      Velocity = Velocity * P;
    }
    DriveRFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
    DriveRBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
    DriveLFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
    DriveLBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
    if (IntakeOn == 1)
      {
        IntakeLBottom.spin(directionType::fwd, 100, velocityUnits::pct);
        IntakeRBottom.spin(directionType::fwd, -100, velocityUnits::pct);
      }
      else if (IntakeOn == 2)
      {
        IntakeLBottom.spin(directionType::fwd, -100, velocityUnits::pct);
        IntakeRBottom.spin(directionType::fwd, 100, velocityUnits::pct);
      }
      else
      {
        IntakeLBottom.stop(brakeType::coast);
        IntakeRBottom.stop(brakeType::coast);
      }
    task::sleep (100);
  }
}

void DriveForwardsNoA (int Velocity, int Distance, int IntakeOn)
{
  DriveLFront.resetPosition ();
  task::sleep (100);
  while (DriveLFront.position (rotationUnits::deg) < (Distance * Multiplier))
  {
    DriveRFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
    DriveRBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
    DriveLFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
    DriveLBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
    if (IntakeOn == 1)
      {
        IntakeLBottom.spin(directionType::fwd, 100, velocityUnits::pct);
        IntakeRBottom.spin(directionType::fwd, -100, velocityUnits::pct);
      }
      else if (IntakeOn == 2)
      {
        IntakeLBottom.spin(directionType::fwd, -100, velocityUnits::pct);
        IntakeRBottom.spin(directionType::fwd, 100, velocityUnits::pct);
      }
      else
      {
        IntakeLBottom.stop(brakeType::coast);
        IntakeRBottom.stop(brakeType::coast);
      }
    task::sleep (100);
  }
}

void DriveBackwards (int Velocity, int Distance)
{
  DriveLFront.resetPosition ();
  task::sleep (100);
  while (-DriveLFront.position (rotationUnits::deg) < (Distance * Multiplier))
  {
      DriveRFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveRBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveLFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveLBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      task::sleep (100);
  }
}

void TurnRight (int Velocity, int Degree)
{
  DriveRFront.resetRotation ();
  task::sleep (100);
  while (DriveRFront.rotation(rotationUnits::deg) < (Degree * BaseMultiplier))
  {
      DriveRFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveRBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveLFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveLBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      task::sleep (100);
  }
}

void TurnLeft (int Velocity, int Degree)
{
  DriveLFront.resetRotation ();
  task::sleep (100);
  while ((-DriveLFront.rotation(rotationUnits::deg)) < (Degree * BaseMultiplier))
  {
      DriveRFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveRBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveLFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveLBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
  }
}

void DriveHold ()
{
  DriveRFront.stop(brakeType::hold);
  DriveRBack.stop(brakeType::hold);
  DriveLFront.stop(brakeType::hold);
  DriveLBack.stop(brakeType::hold);
}

void IntakeUp (int Velocity, double Duration)
{
  IntakeRTop.spin (directionType::fwd, Velocity, velocityUnits::pct);
  IntakeLTop.spin (directionType::fwd, -Velocity, velocityUnits::pct);
  task::sleep (Duration * 1000);
}

void StartIntakeUp (int Velocity)
{
  IntakeRTop.spin (directionType::fwd, Velocity, velocityUnits::pct);
  IntakeLTop.spin (directionType::fwd, -Velocity, velocityUnits::pct);
}

void IntakeDown (int Velocity, double Duration)
{
  IntakeRTop.spin (directionType::fwd, -Velocity, velocityUnits::pct);
  IntakeLTop.spin (directionType::fwd, Velocity, velocityUnits::pct);
  task::sleep (Duration * 1000);
}

void IntakeCoast ()
{
  IntakeRTop.stop (brakeType::coast);
  IntakeLTop.stop (brakeType::coast);
}

void pre_auton() 
{
  vexcodeInit();
  InertialSensor.startCalibration ();
}

void autonomous() 
{
  /*StartIntakeUp (100);
  task::sleep (500);
  DriveBackwards (20, 4);
  IntakeCoast ();
  DriveHold ();
  TurnLeft (20, 110);
  DriveHold ();
  DriveForwardsNoD (50, 62, 1);
  DriveHold ();
  TurnRight (20, 90);
  DriveHold ();
  DriveForwardsNoD (20, 9, 1);
  DriveHold();
  StartIntakeUp (100);
  task::sleep (1000);
  DriveBackwards (20, 9);
  DriveHold ();
  IntakeCoast ();
  TurnLeft (20, 190);
  DriveHold ();
  DriveForwardsNoD (75, 20, 1);
  DriveHold ();
  TurnRight (20, 57);
  DriveHold ();
  //DriveForwardsNoD (50, 110, 1);
  //DriveHold ();
  //TurnLeft (20, 10);
  //DriveHold ();*/
  //StartIntakeUp (100);
  StartIntakeUp (100);
  task::sleep (500);
  IntakeCoast ();
  DriveForwardsNoD (50, 35, 1);
  DriveHold ();
  task::sleep (1000);
  TurnRight (10, 10);
  DriveHold ();
  task::sleep (1000);
  DriveForwardsNoA (100, 20, 2);
  DriveHold ();
  task::sleep (3000);
  DriveBackwards (10, 1);
  DriveHold ();
  StartIntakeUp (900);
  task::sleep (3000);
  IntakeCoast ();
  DriveBackwards (20, 70);
  DriveHold ();
  task::sleep (1000);
  DriveForwardsNoD (50, 96, 1);
  DriveHold ();
  TurnLeft (20, 45);
  DriveHold ();
  task::sleep (1000);
  DriveForwardsNoD (50, 50, 1);
  task::sleep (1000);
  StartIntakeUp (100);



}


void usercontrol() 
{
  //True False Code
  //bool IntakeIn = true;
  //bool ToggleIntakeIn = false; 
  while (true) 
  {
    double EncoderRight;
    double EncoderLeft;
    double PreviousEncoderRight = 0;
    double PreviousEncoderLeft = 0;
    double EncoderChangeRight;
    double EncoderChangeLeft;
    double Angle;
    double PreviousAngle = 0;

    EncoderRight = DistanceEncoderRight.position(rotationUnits::deg);
    EncoderLeft = DistanceEncoderLeft.position(rotationUnits::deg);
    EncoderChangeRight = EncoderRight - PreviousEncoderRight;
    EncoderChangeLeft = EncoderLeft - PreviousEncoderLeft;

    Angle = PreviousAngle + ((((EncoderRight - EncoderLeft) / 360) * 2.75 * M_PI)/(DR + DL)) * Deg;

    PreviousEncoderRight = EncoderRight;
    PreviousEncoderLeft = EncoderLeft;
    PreviousAngle = Angle;
    task::sleep (100);

    // Sensor Values
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor (1, 1);
    Brain.Screen.print ("Distance Encoder Left: %.0f Degrees", DistanceEncoderLeft.position(rotationUnits::deg));
    Brain.Screen.setCursor (2, 1);
    Brain.Screen.print ("Distance Encoder Right: %.0f Degrees", DistanceEncoderRight.position(rotationUnits::deg));
    Brain.Screen.setCursor (3, 1);
    Brain.Screen.print ("Ball Sensor: %d Percent", BallSensor.value(percentUnits::pct));
    Brain.Screen.setCursor (4, 1);
    Brain.Screen.setCursor (5, 1);
    Brain.Screen.print ("Ball Sensor: %f Percent", InertialSensor.yaw(rotationUnits::deg));
    Brain.Screen.render();
    //

    if (Controller1.ButtonR2.pressing() == true)
    {
      IntakeRBottom.spin (directionType::fwd, 100, velocityUnits::pct);
      IntakeLBottom.spin (directionType::fwd, -100, velocityUnits::pct);
      //IntakeIn = false;
    }
    else if (Controller1.ButtonR1.pressing() == true)
    {
      IntakeRBottom.spin (directionType::fwd, -100, velocityUnits::pct);
      IntakeLBottom.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      IntakeRBottom.stop (brakeType::coast);
      IntakeLBottom.stop (brakeType::coast);
    }
    //

    if (Controller1.ButtonL1.pressing() == true)
    {
      IntakeRTop.spin (directionType::fwd, 100, velocityUnits::pct);
      IntakeLTop.spin (directionType::fwd, -100, velocityUnits::pct);
      //IntakeIn = false;
    }
    else if (Controller1.ButtonL2.pressing() == true)
    {
      IntakeRTop.spin (directionType::fwd, -100, velocityUnits::pct);
      IntakeLTop.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      IntakeRTop.stop (brakeType::coast);
      IntakeLTop.stop (brakeType::coast);
    }

    //Drive Base
    //Drive Base
    if (abs(Controller1.Axis3.position(percentUnits::pct)) + abs(Controller1.Axis4.position(percentUnits::pct)) > 10)
    { 
      int axis3original = Controller1.Axis3.position(percentUnits::pct);
      int axis3mapped = 0.000000007 * pow(axis3original, 5) + 0.3 * axis3original;
      int axis4original = Controller1.Axis4.position(percentUnits::pct);
      int axis4mapped = 0.000000007 * pow(axis4original, 5) + 0.3 * axis4original;

      DriveRFront.spin (directionType::fwd, -axis3mapped + axis4mapped, velocityUnits::pct);
      DriveRBack.spin (directionType::fwd, -axis3mapped + axis4mapped, velocityUnits::pct);
      DriveLFront.spin (directionType::fwd, axis3mapped + axis4mapped, velocityUnits::pct);
      DriveLBack.spin (directionType::fwd, axis3mapped + axis4mapped, velocityUnits::pct);
    }
    else
    {
      DriveRFront.stop (brakeType::brake);
      DriveRBack.stop (brakeType::brake);
      DriveLFront.stop (brakeType::brake);
      DriveLBack.stop (brakeType::brake);
    }

    if (Controller1.ButtonA.pressing() == true)
    {
      DistanceEncoderLeft.setRotation (0, rotationUnits::deg);
      InertialSensor.startCalibration ();
    }
  }
}

int main() 
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
}