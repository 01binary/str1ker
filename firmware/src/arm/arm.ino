
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.ino
 Arm Motor Control Board Firmware
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>                        // See ../../../readme.md
#include <ros/time.h>                   // ROS time tracking
#include <str1ker/VelocityCommand.h>    // Velocity message
#include <str1ker/PositionCommand.h>    // Position message
#include <str1ker/GripperCommand.h>     // Gripper message
#include <str1ker/StateFeedback.h>      // State message
#include <str1ker/RawStateFeedback.h>   // Raw state message
#include <firmware.h>                   // See ../readme.md

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int RATE_HZ = 50;                 // Default real time update rate
const int STARTUP_DELAY = 3000;         // Prevent "published too soon" errors

const int BASE_EN = 0;                  // Base motor driver Enabled pin
const int BASE_RPWM = 1;                // Base motor driver right PWM pin
const int BASE_LPWM = 3;                // Base motor driver left PWM pin
const int BASE_SENSE = A2;              // Base motor driver current sense pin
const int BASE_A = 2;                   // Base quadrature encoder A pin
const int BASE_B = 5;                   // Base quadrature encoder B pin
const int BASE_CS = 10;                 // Base absolute encoder chip select pin
const int BASE_SCK = 13;                // Base absolute encoder clock pin
const int BASE_MISO = 12;               // Base absolute encoder data pin
const int BASE_STATUS = 9;              // Base absolute encoder status pin

const int SHOULDER_EN = 6;              // Shoulder motor driver Enabled pin
const int SHOULDER_RPWM = 7;            // Shoulder motor driver right PWM pin
const int SHOULDER_LPWM = 8;            // Shoulder motor driver left PWM pin
const int SHOULDER_SENSE = 4;           // Shoulder motor driver current sense pin
const int SHOULDER_POTENTIOMETER = A0;  // Shoulder potentiometer pin

const int ELBOW_EN = 21;                // Elbow motor driver Enabled pin
const int ELBOW_RPWM = 22;              // Elbow motor right PWM pin
const int ELBOW_LPWM = 23;              // Elbow motor left PWM pin
const int ELBOW_SENSE = 20;             // Elbow motor current sense pin
const int ELBOW_POTENTIOMETER = A1;     // Elbow motor potentiometer pin

const int GRIPPER_PIN = 17;             // Gripper solenoid pin

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef ros::Subscriber<str1ker::VelocityCommand> VelocitySubscriber;
typedef ros::Subscriber<str1ker::PositionCommand> PositionSubscriber;
typedef ros::Subscriber<str1ker::GripperCommand> GripperSubscriber;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

ros::NodeHandle& initializeRos();
void initializeRosInterface();
void initializeJoints();
void loadSettings();
void updateJoints(float timeStep);
void velocityCommand(const str1ker::VelocityCommand& msg);
void positionCommand(const str1ker::PositionCommand& msg);
void gripperCommand(const str1ker::GripperCommand& msg);
void stateFeedback();
void rawFeedback();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::NodeHandle node;
str1ker::StateFeedback stateFeedbackMsg;
str1ker::RawStateFeedback rawFeedbackMsg;
ros::Publisher statePub("arm/state", &stateFeedbackMsg);
ros::Publisher rawPub("arm/raw", &rawFeedbackMsg);
VelocitySubscriber velocitySub("arm/velocity", velocityCommand);
PositionSubscriber positionSub("arm/position", positionCommand);
GripperSubscriber gripperSub("arm/gripper", gripperCommand);
Actuator<AbsoluteEncoder, Motor> baseJoint("base");
Actuator<Potentiometer, Motor> shoulderJoint("shoulder");
Actuator<Potentiometer, Motor> elbowJoint("elbow");
Solenoid gripper;
VoltageCurrentSensor supplySensor;
bool debug;

/*----------------------------------------------------------*\
| Entry points
\*----------------------------------------------------------*/

void setup()
{
  initializeRos();
  initializeJoints();
  supplySensor.initialize();
  loadSettings();
  initializeRosInterface();

  delay(STARTUP_DELAY);
}

void loop()
{
  static uint32_t last = 0;

  uint32_t now = millis();
  const uint32_t period = 1000 / RATE_HZ;

  if (last == 0)
  {
    last = now;
    node.spinOnce();
    return;
  }

  if (now < STARTUP_DELAY)
  {
    last = now;
    node.spinOnce();
    return;
  }

  uint32_t elapsed = now - last;

  if (elapsed < period)
  {
    node.spinOnce();
    return;
  }

  float timeStep = elapsed * 0.001f;

  if (timeStep <= 0.0f)
  {
    node.spinOnce();
    return;
  }

  updateJoints(timeStep);

  node.spinOnce();
  last = now;
}

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

ros::NodeHandle& initializeRos()
{
  node.initNode();

  while (!node.connected())
  {
    node.spinOnce();
    delay(10);
  }

  return node;
}

void initializeRosInterface()
{
  node.advertise(statePub);

  if (debug)
  {
    node.advertise(rawPub);
  }

  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(gripperSub);
  node.negotiateTopics();
}

void initializeJoints()
{
  baseJoint.motor.initialize(BASE_EN, BASE_LPWM, BASE_RPWM, BASE_SENSE);
  baseJoint.encoder.initialize(BASE_STATUS, BASE_CS);

  shoulderJoint.motor.initialize(SHOULDER_EN, SHOULDER_LPWM, SHOULDER_RPWM, SHOULDER_SENSE);
  shoulderJoint.encoder.initialize(SHOULDER_POTENTIOMETER);

  elbowJoint.motor.initialize(ELBOW_EN, ELBOW_LPWM, ELBOW_RPWM, ELBOW_SENSE);
  elbowJoint.encoder.initialize(ELBOW_POTENTIOMETER);

  gripper.initialize(GRIPPER_PIN);
  gripper.update();
}

void loadSettings()
{
  loadBoolParam(node, NULL, "debug", debug);

  baseJoint.loadSettings(node);
  shoulderJoint.loadSettings(node);
  elbowJoint.loadSettings(node);
}

void updateJoints(float timeStep)
{
  baseJoint.update(timeStep);
  shoulderJoint.update(timeStep);
  elbowJoint.update(timeStep);
  gripper.update();

  stateFeedback();

  if (debug)
  {
    rawFeedback();
  }
}

void velocityCommand(const str1ker::VelocityCommand& msg)
{
  baseJoint.writeVelocity(msg.base);
  shoulderJoint.writeVelocity(msg.shoulder);
  elbowJoint.writeVelocity(msg.elbow);
}

void positionCommand(const str1ker::PositionCommand& msg)
{
  baseJoint.writePosition(msg.base);
  shoulderJoint.writePosition(msg.shoulder);
  elbowJoint.writePosition(msg.elbow);
}

void gripperCommand(const str1ker::GripperCommand& msg)
{
  gripper.write(msg.holdTime);
}

void stateFeedback()
{
  stateFeedbackMsg.basePosition = baseJoint.getPosition();
  stateFeedbackMsg.baseVelocity = baseJoint.getVelocity();
  stateFeedbackMsg.baseCurrent = baseJoint.getCurrent();
  stateFeedbackMsg.baseStalled = baseJoint.isStalled();

  stateFeedbackMsg.shoulderPosition = shoulderJoint.getPosition();
  stateFeedbackMsg.shoulderVelocity = shoulderJoint.getVelocity();
  stateFeedbackMsg.shoulderCurrent = shoulderJoint.getCurrent();
  stateFeedbackMsg.shoulderStalled = shoulderJoint.isStalled();

  stateFeedbackMsg.elbowPosition = elbowJoint.getPosition();
  stateFeedbackMsg.elbowVelocity = elbowJoint.getVelocity();
  stateFeedbackMsg.elbowCurrent = elbowJoint.getCurrent();
  stateFeedbackMsg.elbowStalled = elbowJoint.isStalled();

  stateFeedbackMsg.voltage = supplySensor.readVoltage();
  stateFeedbackMsg.current = supplySensor.readCurrent();

  statePub.publish(&stateFeedbackMsg);
}

void rawFeedback()
{
  rawFeedbackMsg.base = baseJoint.encoder.raw();
  rawFeedbackMsg.shoulder = shoulderJoint.encoder.raw();
  rawFeedbackMsg.elbow = elbowJoint.encoder.raw();

  rawFeedbackMsg.baseI = baseJoint.motor.readRaw();
  rawFeedbackMsg.shoulderI = shoulderJoint.motor.readRaw();
  rawFeedbackMsg.elbowI = elbowJoint.motor.readRaw();

  rawPub.publish(&rawFeedbackMsg);
}
