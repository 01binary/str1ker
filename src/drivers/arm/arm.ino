/*
    arm.ino
    ROS Serial Arm Driver Node
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <str1ker/VelocityCommand.h>
#include <str1ker/VelocityFeedback.h>
#include <str1ker/PositionCommand.h>
#include <str1ker/PositionFeedback.h>
#include "freertos/FreeRTOS.h"
#include <Preferences.h>
#include "motor.h"
#include "encoder..h"
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// Velocity command topic (see VelocityCommand.msg)
const char VELOCITY_COMMAND[] = "velocity";

// Position command topic (see PositionCommand.msg)
const char POSITION_COMMAND[] = "position";

// State feedback topic (see StateFeedback.msg)
const char STATE_FEEDBACK[] = "state";

// Startup delay (prevents published too soon errors)
const int STARTUP_DELAY = 3000;

// Base motor left PWM pin
const int BASE_LPWM = 1;

// Base motor right PWM pin
const int BASE_RPWM = 2;

// Base motor current sense pin
const int BASE_IS = 3;

// Base quadrature encoder A pin
const int BASE_QUADRATURE_A = 4;

// Base quadrature encoder B pin
const int BASE_QUADRATURE_B = 5;

// Base absolute encoder chip select pin
const int BASE_ENCODER_CS = 6;

// Base absolute encoder clock pin
const int BASE_ENCODER_CLK = 7;

// Base absolute encoder MISO pin
const int BASE_ENCODER_MISO = 8;

// Base absolute encoder MOSI pin
const int BASE_ENCODER_MOSI = 9;

// Shoulder motor left PWM pin
const int SHOULDER_LPWM = 10;

// Shoulder motor right PWM pin
const int SHOULDER_RPWM = 11;

// Shoulder motor current sense pin
const int SHOULDER_IS = 12;

// Shoulder motor potentiometer pin
const int SHOULDER_POT = 14;

// Elbow motor left PWM pin
const int ELBOW_LPWM = 13;

// Elbow motor right PWM pin
const int ELBOW_RPWM = 16;

// Elbow motor current sense pin
const int ELBOW_IS = 11;

// Elbow motor potentiometer pin
const int ELBOW_POT = 15;

// Gripper solenoid pin
const int GRIPPER_PIN = 17;

// Prevent published too soon errors
const int STARTUP_DELAY = 3000;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// ROS node
ros::NodeHandle node;

// State feedback publisher
ros::Publisher statePub(
  STATE_FEEDBACK, &stateFeedbackMsg);

// Velocity command subscriber
ros::Subscriber<str1ker::VelocityCommand> velocitySub(
  VELOCITY_COMMAND, velocityCommand);

// Position command subscriber
ros::Subscriber<str1ker::PositionCommand> positionSub(
  POSITION_COMMAND, positionCommand);

// Gripper command subscriber
ros::Subscriber<str1ker::GripperCommand> gripperSub(
  GRIPPER_COMMAND, gripperCommand);

// Current time
ros::Time time;

// Time step
double dt;

// Base state
ControlMode baseMode = VELOCITY;
Motor baseMotor;
AS5045Encoder baseAbsoluteEncoder;
QuadratureEncoder baseQuadratureEncoder;
PID baseController;
int baseQuadratureDelta;
double basePosition;
double baseVelocity;
bool baseStalled;

// Shoulder state
ControlMode shoulderMode = VELOCITY;
Motor shoulderMotor;
Potentiometer shoulderEncoder;
PID shoulderController;
double shoulderPosition;
double shoulderVelocity;
bool shoulderStalled;

// Elbow state
ControlMode elbowMode = VELOCITY;
Motor elbowMotor;
Potentiometer elbowEncoder;
PID elbowController;
double elbowPosition;
double elbowVelocity;
bool elbowStalled;

// Gripper state
Solenoid gripper;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeRosInterface();
void initializeHardware();
void initializeControllers();
void motorControl();
void velocityCommand(const str1ker::VelocityCommand& msg);
void positionCommand(const str1ker::PositionCommand& msg);
void gripperCommand(const str1ker::GripperCommand& msg);
void stateFeedback();

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void init()
{
  readSettings();

  initializeRosInterface();
  initializeHardware();
  initializeControllers();
}

void loop()
{
  if (millis() > STARTUP_DELAY)
  {
    stateFeedback();
    node.spinOnce();
  }
}

ros::Time getTime()
{
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t ns = (ms - sec * 1000) * 1000000;

  return ros::Time(sec, ns);
}

void motorControl()
{
  while (true)
  {
    ros::Time now = getTime();
    dt = (now - time).toSec();
    time = now;

    // Base motor
    basePosition = baseAbsoluteEncoder.read();
    baseQuadratureDelta = baseQuadratureEncoder.read();
    baseStalled = baseMotor.read();

    if (baseMode == POSITION)
    {
      baseVelocity = baseController.update(basePosition);
    }

    baseMotor.write(baseVelocity);

    // Shoulder motor
    shoulderPosition = shoulderEncoder.read();
    shoulderStalled = shoulderMotor.read();

    if (shoulderMode == POSITION)
    {
      shoulderVelocity = shoulderController.update(shoulderPosition);
    }

    shoulderMotor.write(shoulderVelocity);

    // Elbow motor
    elbowPosition = elbowEncoder.read();
    elbowStalled = elbowMotor.read();

    if (elbowMode == POSITION)
    {
      elbowVelocity = elbowController.update(elbowPosition);
    }
  }
}

void initializeRosInterface()
{
  delay(STARTUP_DELAY);

  node.initNode();
  node.advertise(statePub);
  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(gripperSub);
  node.negotiateTopics();
}

void initializeHardware()
{
  // Base
  baseMotor.initialize(
    BASE_LPWM,
    BASE_RPWM,
    BASE_IS,
    basePwmMin,
    basePwmMax,
    basePwmInvert
  );

  baseAbsoluteEncoder.initialize(
    BASE_ENCODER_CS,
    BASE_ENCODER_CLK,
    BASE_ENCODER_MISO,
    BASE_ENCODER_MOSI,
    baseEncoderMin,
    baseEncoderMax,
    baseEncoderInvert
  );

  baseQuadratureEncoder.initialize(
    BASE_QUADRATURE_A,
    BASE_QUADRATURE_B,
    baseQuadratureInvert
  );

  // Shoulder
  shoulderMotor.initialize(
    SHOULDER_LPWM,
    SHOULDER_RPWM,
    SHOULDER_IS,
    shoulderPwmMin,
    shoulderPwmMax,
    shoulderPwmInvert
  );

  shoulderEncoder.initialize(SHOULDER_POT);

  // Elbow
  elbowMotor.initialize(
    ELBOW_LPWM,
    ELBOW_RPWM,
    ELBOW_IS,
    elbowPwmMin,
    elbowPwmMax,
    elbowPwmInvert
  );

  elbowEncoder.initialize(ELBOW_POT);

  // Gripper
  gripper.initialize(GRIPPER_PIN);
}

void initializeControllers()
{
  baseController.initialize(
    baseKp,
    baseKi,
    baseKd,
    baseIMin,
    baseIMax
  );

  shoulderController.initialize(
    shoulderKp,
    shoulderKi,
    shoulderKd,
    shoulderIMin,
    shoulderIMax
  );

  elbowController.initialize(
    elbowKp,
    elbowKi,
    elbowKd,
    elbowIMin,
    elbowIMax
  );

  xTaskCreate(
    motorControl,
    "MotorControl",
    2048,
    NULL,
    configMAX_PRIORITIES - 1,
    NULL
  );
}

void velocityCommand(const str1ker::VelocityCommand& msg)
{

}

void positionCommand(const str1ker::PositionCommand& msg)
{

}

void gripperCommand(const str1ker::GripperCommand& msg)
{
  gripper.write(msg.holdTime);
}

void stateFeedback()
{
  str1ker::StateFeedback stateFeedbackMsg;

  stateFeedbackMsg.basePosition = basePosition;
  stateFeedbackMsg.baseVelocity = baseVelocity;
  stateFeedbackMsg.baseStalled = baseStalled;

  stateFeedbackMsg.shoulderPosition = shoulderPosition;
  stateFeedbackMsg.shoulderVelocity = shoulderVelocity;
  stateFeedbackMsg.shoulderStalled = shoulderStalled;

  stateFeedbackMsg.elbowPosition = elbowPosition;
  stateFeedbackMsg.elbowVelocity = elbowVelocity;
  stateFeedbackMsg.elbowStalled = elbowStalled;
}
