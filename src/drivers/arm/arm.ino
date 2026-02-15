
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
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include "interface.h"
#include "encoder.h"
#include "solenoid.h"
#include "joint.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int RATE_HZ = 50;                   // Default real time update rate
const int STARTUP_DELAY = 3000;           // Prevent "published too soon" errors
const float MS_TO_SECONDS = 0.001f;       // Milliseconds to seconds conversion factor

const int BASE_LPWM = 11;                 // Base motor left PWM pin
const int BASE_RPWM = 3;                  // Base motor right PWM pin
const int BASE_SENSE = A2;                // Base motor current sense pin
const int BASE_A = 18;                    // Base quadrature encoder A pin
const int BASE_B = 19;                    // Base quadrature encoder B pin
const int BASE_CS = 53;                   // Base absolute encoder chip select pin

const int SHOULDER_LPWM = 6;              // Shoulder motor left PWM pin
const int SHOULDER_RPWM = 5;              // Shoulder motor right PWM pin
const int SHOULDER_SENSE = A3;            // Shoulder motor current sense pin
const int SHOULDER_POTENTIOMETER = A0;    // Shoulder motor potentiometer pin

const int ELBOW_LPWM = 10;                // Elbow motor left PWM pin
const int ELBOW_RPWM = 9;                 // Elbow motor right PWM pin
const int ELBOW_SENSE = A4;               // Elbow motor current sense pin
const int ELBOW_POTENTIOMETER = A1;       // Elbow motor potentiometer pin

const int GRIPPER_PIN = 13;               // Gripper solenoid pin
const char ARM_NAMESPACE[] = "arm/";      // Node namespace
const char VELOCITY_TOPIC[] = "arm/velocity";
const char POSITION_TOPIC[] = "arm/position";
const char GRIPPER_TOPIC[] = "arm/gripper";
const char STATE_TOPIC[] = "arm/state";

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Solenoid gripper;                         // Gripper hardware
uint32_t lastTime = 0;

ros::NodeHandle node;
str1ker::StateFeedback stateFeedbackMsg;
ros::Publisher statePub(STATE_TOPIC, &stateFeedbackMsg);
VelocitySubscriber velocitySub(VELOCITY_TOPIC, velocityCommand);
PositionSubscriber positionSub(POSITION_TOPIC, positionCommand);
GripperSubscriber gripperSub(GRIPPER_TOPIC, gripperCommand);

typedef jointImpl<FusionEncoder> baseJoint;
typedef jointImpl<Potentiometer> armJoint;

void initBase(baseJoint::actuator& actuator)
{
  actuator.motor.initialize(BASE_LPWM, BASE_RPWM, BASE_SENSE);
  actuator.encoder.initialize(BASE_CS, BASE_A, BASE_B);
}

void initShoulder(armJoint::actuator& actuator)
{
  actuator.motor.initialize(SHOULDER_LPWM, SHOULDER_RPWM, SHOULDER_SENSE);
  actuator.encoder.initialize(SHOULDER_POTENTIOMETER);
}

void initElbow(armJoint::actuator& actuator)
{
  actuator.motor.initialize(ELBOW_LPWM, ELBOW_RPWM, ELBOW_SENSE);
  actuator.encoder.initialize(ELBOW_POTENTIOMETER);
}

baseJoint base(
  "base",
  &str1ker::VelocityCommand::base,
  &str1ker::PositionCommand::base,
  &str1ker::StateFeedback::basePosition,
  &str1ker::StateFeedback::baseVelocity,
  &str1ker::StateFeedback::baseStalled,
  initBase
);

armJoint shoulder(
  "shoulder",
  &str1ker::VelocityCommand::shoulder,
  &str1ker::PositionCommand::shoulder,
  &str1ker::StateFeedback::shoulderPosition,
  &str1ker::StateFeedback::shoulderVelocity,
  &str1ker::StateFeedback::shoulderStalled,
  initShoulder
);

armJoint elbow(
  "elbow",
  &str1ker::VelocityCommand::elbow,
  &str1ker::PositionCommand::elbow,
  &str1ker::StateFeedback::elbowPosition,
  &str1ker::StateFeedback::elbowVelocity,
  &str1ker::StateFeedback::elbowStalled,
  initElbow
);

joint* joints[] = {
  &base,
  &shoulder,
  &elbow
};

const unsigned int JOINT_COUNT = sizeof(joints) / sizeof(joints[0]);

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

void velocityCommand(const str1ker::VelocityCommand& msg)
{
  for (unsigned int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i]->writeVelocity(msg);
  }
}

void positionCommand(const str1ker::PositionCommand& msg)
{
  for (unsigned int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i]->writePosition(msg);
  }
}

void gripperCommand(const str1ker::GripperCommand& msg)
{
  gripper.write(msg.holdTime);
}

void stateFeedback()
{
  for (unsigned int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i]->writeState(stateFeedbackMsg);
  }

  statePub.publish(&stateFeedbackMsg);
}

/*----------------------------------------------------------*\
| Entry points
\*----------------------------------------------------------*/

void setup()
{
  gripper.initialize(GRIPPER_PIN);

  for (unsigned int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i]->initialize();
  }

  initializeRos();

  for (unsigned int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i]->load(node);
  }
}

void loop()
{
  uint32_t now = millis();
  const uint32_t periodMs = 1000 / RATE_HZ;
  gripper.update();

  if (now < STARTUP_DELAY)
  {
    lastTime = now;
    node.spinOnce();
    return;
  }

  uint32_t elapsedMs = now - lastTime;

  if (elapsedMs < periodMs)
  {
    node.spinOnce();
    return;
  }

  float timeStep = elapsedMs * MS_TO_SECONDS;

  if (timeStep <= 0.0f)
  {
    node.spinOnce();
    return;
  }

  for (unsigned int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i]->update(timeStep);
  }

  stateFeedback();

  node.spinOnce();
  lastTime = now;
}
