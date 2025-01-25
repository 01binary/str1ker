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

//
// ROS Topics
//

// Velocity command topic (see VelocityCommand.msg)
const char VELOCITY_COMMAND[] = "velocity";

// Position command topic (see PositionCommand.msg)
const char POSITION_COMMAND[] = "position";

// State feedback topic (see StateFeedback.msg)
const char STATE_FEEDBACK[] = "state_feedback";

// Startup delay (prevents published too soon errors)
const int STARTUP_DELAY = 3000;

//
// Enumerations
//

enum ControlMode
{
  VELOCITY,
  POSITION
};

//
// Settings
//

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

//
// ROS Interface
//

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

//
// State
//

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

//
// Settings
//

// ROS Node name
String name = "arm";

// Update rate
int rate = 50;

double baseKp = 1.0;
double baseKi = 0.1;
double baseKd = 0.1;
double baseIMin = -1.0;
double baseIMax = 1.0;
double baseEncoderMin = 0.0;
double baseEncoderMax = 0.0;
double basePwmMin = 0.0;
double basePwmMax = 0.0;
double baseStallThreshold = 0.0;
bool baseEncoderInvert = false;
bool baseQuadratureInvert = false;
bool basePwmInvert = false;

double shoulderKp = 1.0;
double shoulderKi = 0.1;
double shoulderKd = 0.1;
double shoulderIMin = -1.0;
double shoulderIMax = 1.0;
double shoulderEncoderMin = 0.0;
double shoulderEncoderMax = 0.0;
double shoulderPwmMin = 0.0;
double shoulderPwmMax = 0.0;
double shoulderStallThreshold = 0.0;
bool shoulderEncoderInvert = false;
bool shoulderPwmInvert = false;

double elbowKp = 1.0;
double elbowKi = 0.1;
double elbowKd = 0.1;
double elbowIMin = -1.0;
double elbowIMax = 1.0;
double elbowEncoderMin = 0.0;
double elbowEncoderMax = 0.0;
double elbowPwmMin = 0.0;
double elbowPwmMax = 0.0;
double elbowStallThreshold = 0.0;
bool elbowEncoderInvert = false;
bool elbowPwmInvert = false;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeRosInterface();
void initializeHardware();
void initializeControllers();
void motorControl();
void readSettings();
void writeSettings();
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

void initializeRosInterface()
{
  delay(STARTUP_DELAY);

  node.initNode();

  node.advertise(statePub);

  // TODO configuration
  //node.advertiseService(configServer);

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

void readSettings()
{
  preferences.begin("settings", false);
  name = preferences.getString("name", name);
  rate = preferences.getInt("rate", rate);

  baseKp = preferences.getDouble("baseKp", baseKp);
  baseKi = preferences.getDouble("baseKi", baseKi);
  baseKd = preferences.getDouble("baseKd", baseKd);
  baseIMin = preferences.getDouble("baseIMin", baseIMin);
  baseIMax = preferences.getDouble("baseIMax", baseIMax);
  baseEncoderMin = preferences.getDouble("baseEncoderMin", baseEncoderMin);
  baseEncoderMax = preferences.getDouble("baseEncoderMax", baseEncoderMax);
  basePwmMin = preferences.getDouble("basePwmMin", basePwmMin);
  basePwmMax = preferences.getDouble("basePwmMax", basePwmMax);
  baseStallThreshold = preferences.getDouble("baseStallThreshold", baseStallThreshold);
  baseEncoderInvert = preferences.getBool("baseEncoderInvert", baseEncoderInvert);
  baseQuadratureInvert = preferences.getBool("baseQuadratureInvert", baseQuadratureInvert);
  basePwmInvert = preferences.getBool("basePwmInvert", basePwmInvert);

  shoulderKp = preferences.getDouble("shoulderKp", shoulderKp);
  shoulderKi = preferences.getDouble("shoulderKi", shoulderKi);
  shoulderKd = preferences.getDouble("shoulderKd", shoulderKd);
  shoulderIMin = preferences.getDouble("shoulderIMin", shoulderIMin);
  shoulderIMax = preferences.getDouble("shoulderIMax", shoulderIMax);
  shoulderEncoderMin = preferences.getDouble("shoulderEncoderMin", shoulderEncoderMin);
  shoulderEncoderMax = preferences.getDouble("shoulderEncoderMax", shoulderEncoderMax);
  shoulderPwmMin = preferences.getDouble("shoulderPwmMin", shoulderPwmMin);
  shoulderPwmMax = preferences.getDouble("shoulderPwmMax", shoulderPwmMax);
  shoulderStallThreshold = preferences.getDouble("shoulderStallThreshold", shoulderStallThreshold);
  shoulderEncoderInvert = preferences.getBool("shoulderEncoderInvert", shoulderEncoderInvert);
  shoulderPwmInvert = preferences.getBool("shoulderPwmInvert", shoulderPwmInvert);

  elbowKp = preferences.getDouble("elbowKp", elbowKp);
  elbowKi = preferences.getDouble("elbowKi", elbowKi);
  elbowKd = preferences.getDouble("elbowKd", elbowKd);
  elbowIMin = preferences.getDouble("elbowIMin", elbowIMin);
  elbowIMax = preferences.getDouble("elbowIMax", elbowIMax);
  elbowEncoderMin = preferences.getDouble("elbowEncoderMin", elbowEncoderMin);
  elbowEncoderMax = preferences.getDouble("elbowEncoderMax", elbowEncoderMax);
  elbowPwmMin = preferences.getDouble("elbowPwmMin", elbowPwmMin);
  elbowPwmMax = preferences.getDouble("elbowPwmMax", elbowPwmMax);
  elbowStallThreshold = preferences.getDouble("elbowStallThreshold", elbowStallThreshold);
  elbowEncoderInvert = preferences.getBool("elbowEncoderInvert", elbowEncoderInvert);
  elbowPwmInvert = preferences.getBool("elbowPwmInvert", elbowPwmInvert);

  preferences.end();
}

void writeSettings()
{
  preferences.begin("settings", false);

  preferences.putString("name", name);
  preferences.putInt("rate", rate);

  preferences.putDouble("baseKp", baseKp);
  preferences.putDouble("baseKi", baseKi);
  preferences.putDouble("baseKd", baseKd);
  preferences.putDouble("baseIMin", baseIMin);
  preferences.putDouble("baseIMax", baseIMax);
  preferences.putDouble("baseEncoderMin", baseEncoderMin);
  preferences.putDouble("baseEncoderMax", baseEncoderMax);
  preferences.putDouble("basePwmMin", basePwmMin);
  preferences.putDouble("basePwmMax", basePwmMax);
  preferences.putDouble("baseStallThreshold", baseStallThreshold);
  preferences.putBool("baseEncoderInvert", baseEncoderInvert);
  preferences.putBool("baseQuadratureInvert", baseQuadratureInvert);
  preferences.putBool("basePwmInvert", basePwmInvert);

  preferences.putDouble("shoulderKp", shoulderKp);
  preferences.putDouble("shoulderKi", shoulderKi);
  preferences.putDouble("shoulderKd", shoulderKd);
  preferences.putDouble("shoulderIMin", shoulderIMin);
  preferences.putDouble("shoulderIMax", shoulderIMax);
  preferences.putDouble("shoulderEncoderMin", shoulderEncoderMin);
  preferences.putDouble("shoulderEncoderMax", shoulderEncoderMax);
  preferences.putDouble("shoulderPwmMin", shoulderPwmMin);
  preferences.putDouble("shoulderPwmMax", shoulderPwmMax);
  preferences.putDouble("shoulderStallThreshold", shoulderStallThreshold);
  preferences.putBool("shoulderEncoderInvert", shoulderEncoderInvert);
  preferences.putBool("shoulderPwmInvert", shoulderPwmInvert);

  preferences.putDouble("elbowKp", elbowKp);
  preferences.putDouble("elbowKi", elbowKi);
  preferences.putDouble("elbowKd", elbowKd);
  preferences.putDouble("elbowIMin", elbowIMin);
  preferences.putDouble("elbowIMax", elbowIMax);
  preferences.putDouble("elbowEncoderMin", elbowEncoderMin);
  preferences.putDouble("elbowEncoderMax", elbowEncoderMax);
  preferences.putDouble("elbowPwmMin", elbowPwmMin);
  preferences.putDouble("elbowPwmMax", elbowPwmMax);
  preferences.putDouble("elbowStallThreshold", elbowStallThreshold);
  preferences.putBool("elbowEncoderInvert", elbowEncoderInvert);
  preferences.putBool("elbowPwmInvert", elbowPwmInvert);

  preferences.end();
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
