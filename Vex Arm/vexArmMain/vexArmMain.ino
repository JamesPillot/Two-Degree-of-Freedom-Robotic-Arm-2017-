#include <vexArm.h>
#include <vexMotor.h>
#include "math.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/*Constants*/
const double BASE_LENGTH = 8.3; //in centimeters
const double JOINT_LENGTH = 8.5; //in centimeters
const double Y_OFFSET = 8.1; //in centimeters (old one was 8.1 cm, on rail add 6 cm to make 14.1
const double RAD_DEGREE_CONV = 57.29577;
const double DEGREE_RAD_CONV = 0.017453;
const double CM_REV_CONV = 0.0; //fill in when constant is found
const double REV_CM_CONV = 0.0; //fill in when constant is found
const int TIMESTEP = 25; //in milliseconds

/*Objects*/
vexArm leftBase;
vexArm rightBase;
vexArm elbowJoint;
Adafruit_MotorShield shield = Adafruit_MotorShield();
Adafruit_StepperMotor *rail = shield.getStepper(200, 1); //1.8 degrees per step and motor port 2

/*Prototypes*/
void initProcess();
void inputProcess();
void humanMachineInterface();
void control();
void baseMotorsCommand(int theta, float inKp, float inKi, float inKd, float inKv);
void moveRail();
float analogScale(int analogInput, int minAnalog, int maxAnalog, int minimumUnits, int maximumUnits);
float calculateX();
float calculateY();
int calculateThetaOne(float x, float y);
int calculateThetaTwo(float x, float y);
bool wait(int value);
float coordToTarget(int first, int second, int third);

/*Variables*/
float basePos, jointPos;
float currentX, currentY, currentZ;
float desiredX, desiredY, lastDesiredX = 0, lastDesiredY = 0, desiredZ;
int programStep = 0, runningTime = 0;
int thetaOneTarget = 0, thetaTwoTarget = 0;
int lastThetaOneTarget, lastThetaTwoTarget;
int railSteps;
String coordinates;

typedef enum {
  no,
  take
} hmiStates;
hmiStates inputState;

void setup() {
  Serial.begin(9600);
  initProcess();
  //rail -> step(200, FORWARD, SINGLE);
}

void loop() {
  inputProcess();
  humanMachineInterface();
  control();
  runningTime += TIMESTEP;
  delay(TIMESTEP);
}

/*Initialize motors and sensors*/
void initProcess()
{
  rightBase.attachPin(2);
  leftBase.attachPin(3);
  elbowJoint.attachPin(4);
  //rail
  shield.begin();
  rail -> setSpeed(20); //rpm
}

/*Handle all inputs the system recieves, this is primarily from sensors*/
void inputProcess()
{
  basePos = analogScale(analogRead(A0), 53, 695, -48, 90);
  jointPos = analogScale(analogRead(A1), 123, 991, 90, -90);
  leftBase.setProcessVariable((int)basePos);
  rightBase.setProcessVariable((int)basePos);
  elbowJoint.setProcessVariable((int)jointPos);
  currentX = calculateX();
  currentY = calculateY();
  currentZ = railSteps * REV_CM_CONV;
}

/*This is where the commands to the robot go. This may be from a script or placed here*/
void humanMachineInterface()
{
  //Enter in coordinates in 'take:(xx.x, yy.y, zz.z)' format
  /* coordinates = Serial.readString();
    float x = 0.0, y = 0.0, z = 0.0;
    x = coordToTarget(6, 7, 9);
    y = coordToTarget(12, 13, 15);
    z = coordToTarget(18, 19, 21);
    if (coordinates[0] == 't')
    {
     inputState = take;
    }
    // Serial.println(x);
    switch (inputState)
    {
     default:
       break;

     case no:
       desiredX = lastDesiredX;
       desiredY = lastDesiredY;
       break;

     case take:
       desiredX = x;
       desiredY = y;
       inputState = no;
       break;
    }
    lastDesiredX = desiredX;
    lastDesiredY = desiredY;*/
  desiredX = 14.0;
  desiredY = 10.0;
  desiredZ = 0.0;
}

/*Where the logic and control processes happen for the machine*/
void control()
{
  /*Turn on motors once first command has been sent (change to on when ready)*/
  if (desiredX != 0 && desiredY != 0)
  {
    leftBase.setMotorState(on);
    rightBase.setMotorState(on);
    elbowJoint.setMotorState(on);
  }
  /*State machine for getting joints to correct position)*/
  switch (programStep)
  {
    default:
      programStep = 0;
      break;

    case 0: //move base high enough so that elbow joint has room to move
      thetaOneTarget = 20.0;
      programStep++;
      break;

    case 1: //check that base is within range to advance
      if (abs(rightBase.retrieveError()) < 3.5)
      {
        programStep++;
      }
      break;

    case 2: //calculate joint position and move joint to desired position
      thetaTwoTarget = calculateThetaTwo(desiredX, desiredY);
      programStep++;
      break;

    case 3: //check that joint is within range to advance
      if (abs(elbowJoint.retrieveError()) < 3.0)
      {
        programStep++;
      }
      break;

    case 4: //calculate base position now that joint position is set
      thetaOneTarget = calculateThetaOne(desiredX, desiredY);
      if (thetaOneTarget < -48.0)
      {
        thetaOneTarget = -1 * thetaOneTarget;
        programStep = 6;
      }
      else
      {
        programStep++;
      }
      break;

    case 5:
      //rest state do nothing
      break;

    case 6:
      if (abs(rightBase.retrieveError()) < 3.5)
      {
        programStep++;
      }
      break;

    case 7:
      if (abs(thetaTwoTarget) > 95.0)
      {
        thetaTwoTarget = -94.0;
      }
      else
      {
        thetaTwoTarget = -1 * thetaTwoTarget;
      }
      programStep++;
      break;

    case 8:
      if (abs(elbowJoint.retrieveError()) < 3.0)
      {
        programStep++;
      }
      break;

    case 9:
      thetaOneTarget = abs(calculateThetaOne(desiredX, desiredY));
      programStep++;
      break;

    case 10:
      break;

  }

  /*Command base*/
  if (thetaOneTarget < 100.0 && thetaOneTarget > -50.0)
  {
    baseMotorsCommand(thetaOneTarget, //target
                      4.0, .32, 0, //up
                      2.1, .29, .35, //down
                      0);//kv
  }
  else
  {
    baseMotorsCommand(thetaOneTarget,
                      0, 0, 0,
                      0, 0, 0,
                      0);
  }
  /*Command elbow joint*/
  if (thetaTwoTarget < 109.0 && thetaTwoTarget > -98.0)
  {
    elbowJoint.setTheta(thetaTwoTarget);
    elbowJoint.setGains(1.6, .2, 0.26,  //up
                        1.75, .2, 0.32, //down
                        0); //kv
  }
  else
  {
    elbowJoint.setGains(0, 0, 0,  //up
                        0, 0, 0, //down
                        0); //kv
  }
  if (jointPos > 100.0 && jointPos < 108.00)
  {
    elbowJoint.setTheta(thetaTwoTarget);
    elbowJoint.setGains(.12, .07, 0.1,  //up
                        1.75, .2, 0.32, //down
                        0); //kv
  }
  if (jointPos < -60.0)
  {
    elbowJoint.setTheta(thetaTwoTarget);
    elbowJoint.setGains(.2, .1, 0.16,  //up
                        .05, .02, 0.02, //down
                        0); //kv
  }

  lastThetaOneTarget = thetaOneTarget;
  lastThetaTwoTarget = thetaTwoTarget;

  // moveRail(); for rail
  /*Graph processes*/
  Serial.print(thetaOneTarget);
  Serial.print(" ");
  Serial.print(basePos);
  Serial.print(" ");
  Serial.print(thetaTwoTarget);
  Serial.print(" ");
  Serial.print(jointPos);
  Serial.print(" ");
  Serial.print(desiredX);
  Serial.print(" ");
  Serial.print(currentX);
  Serial.print(" ");
  Serial.print(desiredY);
  Serial.print(" ");
  Serial.print(currentY);
  Serial.print(" ");
  Serial.print(elbowJoint.getP());
  Serial.print(" ");
  Serial.print(elbowJoint.getMotorState());
  Serial.print(" ");
  Serial.println(programStep); 
}

/*Takes in analog input and returns real world units*/
float analogScale(int analogInput, int minAnalog, int maxAnalog, int minimumUnits, int maximumUnits)
{
  float unitDifference = maximumUnits - minimumUnits;
  float analogDifference = maxAnalog - minAnalog;
  float currDifference = analogInput - minAnalog;
  float quotient = currDifference / analogDifference;
  float product = unitDifference * quotient;
  return (minimumUnits + product);
}

/*Calculates the cartesian coordinates of the arm's end effector using forward kinematics*/
float calculateX()
{
  double basePosRad = basePos * DEGREE_RAD_CONV;
  double firstX = abs(BASE_LENGTH * cos(basePosRad));
  double jointPosRad = jointPos * DEGREE_RAD_CONV;
  double secondX = abs((JOINT_LENGTH *  cos((basePosRad + jointPosRad))));
  return (float)(firstX + secondX);
}
float calculateY()
{
  double basePosRad = basePos * DEGREE_RAD_CONV;
  double firstY = (BASE_LENGTH * sin(basePosRad)) + Y_OFFSET;
  double jointPosRad = jointPos * DEGREE_RAD_CONV;
  double secondY = (JOINT_LENGTH *  sin((basePosRad + jointPosRad)));
  Serial.println(secondY);
  return (float)(firstY + secondY);
}

/*Performs Inverse Kinematics to take cartesian coordinate and turn it into a desired theta*/
int calculateThetaOne(float x, float y)
{
  y -= Y_OFFSET;
  double jointPosRad = jointPos * DEGREE_RAD_CONV;
  double output = (atan2(y, x)) - (atan2((JOINT_LENGTH * sin(jointPosRad)), (BASE_LENGTH + (JOINT_LENGTH * cos(jointPosRad)))));
  //Serial.println(output * RAD_DEGREE_CONV);
  return (int)(output * RAD_DEGREE_CONV);
}
int calculateThetaTwo(float x, float y)
{
  y -= Y_OFFSET;
  double D = (square(x) + square(y) - square(BASE_LENGTH) - square(JOINT_LENGTH)) / (2 * BASE_LENGTH * JOINT_LENGTH); //intermediate step of final calculation
  double output = atan2((sqrt(1 - square(D))), D);
  //Serial.println(output * RAD_DEGREE_CONV);
  return (int)(output * RAD_DEGREE_CONV);
}

/*Groups together two base motors via software*/
void baseMotorsCommand(int theta,
                       float uinKp, float uinKi, float uinKd,
                       float dinKp, float dinKi, float dinKd,
                       float inKv)
{
  leftBase.setTheta(theta);
  leftBase.setGains(uinKp, uinKi, uinKd,
                    dinKp, dinKi, dinKd,
                    inKv);
  rightBase.setTheta(theta);
  rightBase.setGains(uinKp, uinKi, uinKd,
                     dinKp, dinKi, dinKd,
                     inKv);
}

/*A timer that doesn't freeze the program. Values entered in milliseconds*/
bool wait(int value)
{
  return runningTime >= value;
}

/*Helper function to turn serial commands to input*/
float coordToTarget(int first, int second, int third)
{
  float value = 0.0;
  value += (10.0 * (float)(coordinates[first] - '0'));
  value += (float)(coordinates[second] - '0');
  value += (.1 * (float)(coordinates[third] - '0'));
  return value;
}

void moveRail()
{
  int railError = desiredZ - currentZ;
  int railIncrement = railError * CM_REV_CONV;
  if (railError >= 0) rail -> step(railIncrement, FORWARD, MICROSTEP);
  if (railError < 0) rail -> step(abs(railIncrement), BACKWARD, MICROSTEP);
  railSteps += railIncrement;
}


