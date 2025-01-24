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

// Velocity feedback topic (see VelocityFeedback.msg)
const char VELOCITY_FEEDBACK[] = "velocity_feedback";

// Position command topic (see PositionCommand.msg)
const char POSITION_COMMAND[] = "position";

// Position feedback topic (see PositionFeedback.msg)
const char POSITION_FEEDBACK[] = "position_feedback";

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

const int BASE_LPWM = 1;            // Base motor left PWM pin
const int BASE_RPWM = 2;            // Base motor right PWM pin
const int BASE_IS = 3;              // Base motor current sense pin

const int BASE_QUADRATURE_A = 4;    // Base quadrature encoder A pin
const int BASE_QUADRATURE_B = 5;    // Base quadrature encoder B pin

const int BASE_ENCODER_CS = 6;      // Base absolute encoder chip select pin
const int BASE_ENCODER_CLK = 7;     // Base absolute encoder clock pin
const int BASE_ENCODER_MISO = 8;    // Base absolute encoder MISO pin
const int BASE_ENCODER_MOSI = 9;    // Base absolute encoder MOSI pin

const int SHOULDER_LPWM = 10;       // Shoulder motor left PWM pin
const int SHOULDER_RPWM = 11;       // Shoulder motor right PWM pin
const int SHOULDER_IS = 12;         // Shoulder motor current sense pin
const int SHOULDER_POT = 14;        // Shoulder motor potentiometer pin

const int ELBOW_LPWM = 13;          // Elbow motor left PWM pin
const int ELBOW_RPWM = 16;          // Elbow motor right PWM pin
const int ELBOW_IS = 11;            // Elbow motor current sense pin
const int ELBOW_POT = 15;           // Elbow motor potentiometer pin

const int GRIPPER_PIN = 17;         // Gripper solenoid pin

const int STARTUP_DELAY = 3000;     // Prevent published too soon errors

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

//
// ROS Interface
//

// ROS node
ros::NodeHandle node;

// Velocity feedback publisher
str1ker::VelocityFeedback velocityFeedbackMsg;
ros::Publisher velocityPub(
  VELOCITY_FEEDBACK, &velocityFeedbackMsg);

// Position feedback publisher
str1ker::PositionFeedback positionFeedbackMsg;
ros::Publisher positionPub(
  POSITION_FEEDBACK, &positionFeedbackMsg);

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

int rate = 50;
ros::Time time;
float dt;

Motor baseMotor;
AS5045Encoder baseAbsoluteEncoder;
QuadratureEncoder baseQuadratureEncoder;
PID baseController;

Motor shoulderMotor;
Potentiometer shoulderEncoder;
PID shoulderController;

Motor elbowMotor;
Potentiometer elbowEncoder;
PID elbowController;

Solenoid gripper;

//
// Settings
//

ControlMode baseMode = VELOCITY;
double baseKp = 1.0;
double baseKi = 0.1;
double baseKd = 0.1;
double baseIMin = -1.0;
double baseIMax = 1.0;
double baseEncoderMin = 0.0;
double baseEncoderMax = 0.0;
double basePwmMin = 0.0;
double basePwmMax = 0.0;
bool baseEncoderInvert = false;
bool baseQuadratureInvert = false;
bool basePwmInvert = false;

ControlMode shoulderMode = VELOCITY;
double shoulderKp = 1.0;
double shoulderKi = 0.1;
double shoulderKd = 0.1;
double shoulderIMin = -1.0;
double shoulderIMax = 1.0;
double shoulderEncoderMin = 0.0;
double shoulderEncoderMax = 0.0;
double shoulderPwmMin = 0.0;
double shoulderPwmMax = 0.0;
bool shoulderEncoderInvert = false;
bool shoulderPwmInvert = false;

ControlMode elbowMode = VELOCITY;
double elbowKp = 1.0;
double elbowKi = 0.1;
double elbowKd = 0.1;
double elbowIMin = -1.0;
double elbowIMax = 1.0;
double elbowEncoderMin = 0.0;
double elbowEncoderMax = 0.0;
double elbowPwmMin = 0.0;
double elbowPwmMax = 0.0;
bool elbowEncoderInvert = false;
bool elbowPwmInvert = false;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeRosInterface();
void initializeHardware();
void initializeControllers();
void readSettings();
void writeSettings();
void velocityCommand(const str1ker::VelocityCommand& msg);
void positionCommand(const str1ker::PositionCommand& msg);
void velocityFeedback();
void positionFeedback();

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
  node.spinOnce();
}

ros::Time getTime()
{
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t ns = (ms - sec * 1000) * 1000000;

  return ros::Time(sec, ns);
}

void live()
{
  // Track time
  ros::Time now = getTime();
  dt = (now - time).toSec();
  elapsed = (now - start).toSec();

  if (dt >= TIMESTEP) time = now;
}

void initializeRosInterface()
{
  delay(STARTUP_DELAY);

  node.initNode();

  node.advertise(velocityPub);
  node.advertise(positionPub);

  node.advertiseService(configServer);

  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(stepSub);
  node.subscribe(configSub);
  node.subscribe(stopSub);

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
}

void readSettings()
{
  preferences.begin("settings", false);
  // name = preferences.getString("name", name);
  preferences.end();
}

void writeSettings()
{
  preferences.begin("settings", false);
  // preferences.putString("name", name.c_str());
  preferences.end();
}

void velocityCommand(const str1ker::VelocityCommand& msg)
{

}

void positionCommand(const str1ker::PositionCommand& msg)
{

}

void velocityFeedback()
{

}

void positionFeedback()
{

}
