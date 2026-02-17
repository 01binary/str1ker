
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.ino
 Arm Driver Board
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <str1ker/VelocityCommand.h>
#include <str1ker/PositionCommand.h>
#include <str1ker/GripperCommand.h>
#include <str1ker/StateFeedback.h>
#include <str1ker/RawJointSensors.h>
#include "actuator.h"
#include "motor.h"
#include "encoder.h"
#include "solenoid.h"
#include "params.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int RATE_HZ = 50;                       // Default real time update rate
const int STARTUP_DELAY = 3000;               // Prevent "published too soon" errors
const float MILLISECONDS_TO_SECONDS = 0.001f; // Milliseconds to seconds conversion factor

const int BASE_LPWM = 11;                     // Base motor left PWM pin
const int BASE_RPWM = 3;                      // Base motor right PWM pin
const int BASE_SENSE = A2;                    // Base motor current sense pin
const int BASE_A = 18;                        // Base quadrature encoder A pin
const int BASE_B = 19;                        // Base quadrature encoder B pin
const int BASE_CS = 53;                       // Base absolute encoder chip select pin

const int SHOULDER_LPWM = 6;                  // Shoulder motor left PWM pin
const int SHOULDER_RPWM = 5;                  // Shoulder motor right PWM pin
const int SHOULDER_SENSE = A3;                // Shoulder motor current sense pin
const int SHOULDER_POTENTIOMETER = A0;        // Shoulder motor potentiometer pin

const int ELBOW_LPWM = 10;                    // Elbow motor left PWM pin
const int ELBOW_RPWM = 9;                     // Elbow motor right PWM pin
const int ELBOW_SENSE = A4;                   // Elbow motor current sense pin
const int ELBOW_POTENTIOMETER = A1;           // Elbow motor potentiometer pin

const int GRIPPER_PIN = 13;                   // Gripper solenoid pin
const char ARM_NAMESPACE[] = "arm";           // Node namespace
const char VELOCITY_TOPIC[] = "arm/velocity"; // Velocity command topic
const char POSITION_TOPIC[] = "arm/position"; // Position command topic
const char GRIPPER_TOPIC[] = "arm/gripper";   // Gripper command topic
const char STATE_TOPIC[] = "arm/state";       // State topic
const char RAW_TOPIC[] = "arm/raw";           // Raw sensor topic

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef ros::Subscriber<str1ker::VelocityCommand> VelocitySubscriber;
typedef ros::Subscriber<str1ker::PositionCommand> PositionSubscriber;
typedef ros::Subscriber<str1ker::GripperCommand> GripperSubscriber;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

ros::NodeHandle& initializeRos();
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
str1ker::RawJointSensors rawFeedbackMsg;
ros::Publisher statePub(STATE_TOPIC, &stateFeedbackMsg);
ros::Publisher rawPub(RAW_TOPIC, &rawFeedbackMsg);
VelocitySubscriber velocitySub(VELOCITY_TOPIC, velocityCommand);
PositionSubscriber positionSub(POSITION_TOPIC, positionCommand);
GripperSubscriber gripperSub(GRIPPER_TOPIC, gripperCommand);

Actuator<AS5045Encoder, Motor> baseJoint(node, "arm", "base");
Actuator<Potentiometer, Motor> shoulderJoint(node, "arm", "shoulder");
Actuator<Potentiometer, Motor> elbowJoint(node, "arm", "elbow");
Solenoid gripper;

bool debug;

/*----------------------------------------------------------*\
| Entry points
\*----------------------------------------------------------*/

void setup()
{
  initializeRos();
  initializeJoints();
  loadSettings();
}

void loop()
{
  static uint32_t last = 0;

  uint32_t now = millis();
  const uint32_t period = 1000 / RATE_HZ;

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

  float timeStep = elapsed * MILLISECONDS_TO_SECONDS;

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

  node.advertise(statePub);
  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(gripperSub);
  node.negotiateTopics();

  delay(STARTUP_DELAY);

  return node;
}

void initializeJoints()
{
  baseJoint.motor.initialize(BASE_LPWM, BASE_RPWM, BASE_SENSE);
  baseJoint.encoder.initialize(BASE_CS, BASE_A, BASE_B);

  shoulderJoint.motor.initialize(SHOULDER_LPWM, SHOULDER_RPWM, SHOULDER_SENSE);
  shoulderJoint.encoder.initialize(SHOULDER_POTENTIOMETER);

  elbowJoint.motor.initialize(ELBOW_LPWM, ELBOW_RPWM, ELBOW_SENSE);
  elbowJoint.encoder.initialize(ELBOW_POTENTIOMETER);

  gripper.initialize(GRIPPER_PIN);
  gripper.update();
}

void loadSettings()
{
  loadParam(node, ARM_NAMESPACE, "debug", (int&)debug);

  if (debug)
  {
    node.loginfo("Debugging enabled");
  }

  baseJoint.loadSettings();
  shoulderJoint.loadSettings();
  elbowJoint.loadSettings();
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

  statePub.publish(&stateFeedbackMsg);
}

void rawFeedback()
{
  rawFeedbackMsg.base = baseJoint.encoder.raw();
  rawFeedbackMsg.shoulder = shoulderJoint.encoder.raw();
  rawFeedbackMsg.elbow = elbowJoint.encoder.raw();
}
