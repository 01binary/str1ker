/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motor.cpp

 PWM Motor Controller Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"
#include "motor.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char motor::TYPE[] = "motor";

/*----------------------------------------------------------*\
| motor implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(motor)

//
// Constructors
//

motor::motor(
  ros::NodeHandle node, const char* path)
  : controller(node, path)
{
}

motor::motor(
  ros::NodeHandle node,
  const char* path,
  string topic,
  int lpwm,
  int rpwm,
  int minPwm,
  int maxPwm,
  double minVelocity,
  double maxVelocity)
  : controller(node, path)
  , m_topic(topic)
  , m_lpwm(lpwm)
  , m_rpwm(rpwm)
  , m_minPwm(minPwm)
  , m_maxPwm(maxPwm)
  , m_minVelocity(minVelocity)
  , m_maxVelocity(maxVelocity)
{
}

//
// Configuration
//

bool motor::configure()
{
  controller::configure();

  if (!ros::param::get(getControllerPath("topic"), m_topic))
    ROS_WARN("%s did not specify output topic, using %s", getPath(), m_topic.c_str());

  if (!ros::param::get(getControllerPath("lpwm"), m_lpwm))
    ROS_WARN("%s did not specify lpwm channel, using %d", getPath(), m_lpwm);

  if (!ros::param::get(getControllerPath("rpwm"), m_rpwm))
    ROS_WARN("%s did not specify rpwm channel, using %d", getPath(), m_rpwm);

  if (!ros::param::get(getControllerPath("minPwm"), m_minPwm))
    ROS_WARN("%s did not specify minPwm value, using %d", getPath(), m_minPwm);

  if (!ros::param::get(getControllerPath("maxPwm"), m_maxPwm))
    ROS_WARN("%s did not specify maxPwm value, using %d", getPath(), m_maxPwm);

  if (!ros::param::get(getControllerPath("minVelocity"), m_minVelocity))
    ROS_WARN("%s did not specify minVelocity, using %g", getPath(), m_minVelocity);

  if (!ros::param::get(getControllerPath("maxVelocity"), m_maxVelocity))
    ROS_WARN("%s did not specify maxVelocity, using %g", getPath(), m_maxVelocity);

  return true;
}

//
// Initialization
//

bool motor::init()
{
  // Initialize PWM output publisher
  m_pwmPub = m_node.advertise<Pwm>(
    m_topic,
    QUEUE_SIZE
  );

  ROS_INFO("  initialized %s %s on %s: (LPWM %d RPWM %d) [%d, %d] -> [%g, %g]",
    getPath(),
    getType(),
    m_topic.c_str(),
    m_lpwm,
    m_rpwm,
    m_minPwm,
    m_maxPwm,
    m_minVelocity,
    m_maxVelocity);

  return true;
}

//
// Velocity command
//

void motor::command(double velocity)
{
  m_velocity = utilities::clampZero(abs(velocity), m_minVelocity, m_maxVelocity);

  uint16_t dutyCycle = (uint16_t)utilities::mapZero(
    m_velocity, m_minVelocity, m_maxVelocity, (double)m_minPwm, (double)m_maxPwm);

  m_lpwmCommand = (velocity >= 0 ? 0 : dutyCycle);
  m_rpwmCommand = (velocity >= 0 ? dutyCycle : 0);

  Pwm msg;
  msg.channels.resize(2);

  // LPWM
  msg.channels[0].channel = m_lpwm;
  msg.channels[0].mode = PwmChannel::MODE_ANALOG;
  msg.channels[0].value = m_lpwmCommand;

  // RPWM
  msg.channels[1].channel = m_rpwm;
  msg.channels[1].mode = PwmChannel::MODE_ANALOG;
  msg.channels[1].value = m_rpwmCommand;

  m_pwmPub.publish(msg);
}

//
// Dynamic creation
//

controller* motor::create(ros::NodeHandle node, const char* path)
{
    return new motor(node, path);
}
