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
  robot& robot, const char* path)
  : controller(robot, path)
{
}

motor::motor(
  robot& robot,
  const char* path,
  string topic,
  int lpwm,
  int rpwm,
  int minPwm,
  int maxPwm,
  double minVelocity,
  double maxVelocity)
  : controller(robot, path)
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

void motor::configure(ros::NodeHandle node)
{
  controller::configure(node);

  ros::param::get("outputTopic", m_topic);
  ros::param::get("lpwm", m_lpwm);
  ros::param::get("rpwm", m_rpwm);
  ros::param::get("minPwm", m_minPwm);
  ros::param::get("maxPwm", m_maxPwm);
  ros::param::get("minVelocity", m_minVelocity);
  ros::param::get("maxVelocity", m_maxVelocity);
}

//
// Initialization
//

bool motor::init(ros::NodeHandle node)
{
  // Initialize PWM output publisher
  m_pwmPub = node.advertise<Pwm>(
    m_topic,
    QUEUE_SIZE
  );

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

controller* motor::create(robot& robot, const char* path)
{
    return new motor(robot, path);
}
