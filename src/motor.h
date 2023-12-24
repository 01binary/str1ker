/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motor.h

 PWM Motor Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <str1ker/Pwm.h>
#include "controller.h"
#include "utilities.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| motor class
\*----------------------------------------------------------*/

class motor: public controller
{
public:
  // Controller type
  static const char TYPE[];

private:
  // Default min PWM pulse width
  const int PWM_MIN = 0;

  // Default max PWM pulse width
  const int PWM_MAX = 255;

  // Default min velocity
  const double VELOCITY_MIN = 0.0;

  // Default max velocity
  const double VELOCITY_MAX = 1.0;

  // Publish queue size
  const int QUEUE_SIZE = 8;

private:
  //
  // Configuration
  //

  // PWM output topic
  std::string m_topic = "pwm";

  // Left PWM channel
  int m_lpwm = 0;

  // Right PWM channel
  int m_rpwm = 1;

  // Min PWM pulse width
  int m_minPwm = PWM_MIN;

  // Max PWM pulse width
  int m_maxPwm = PWM_MAX;

  // Min velocity in physical units
  double m_minVelocity = VELOCITY_MIN;

  // Max velocity in physical units
  double m_maxVelocity = VELOCITY_MAX;

  //
  // Interface
  //

  // PWM publisher to motor driver
  ros::Publisher m_pwmPub;

  //
  // State
  //

  // Last LPWM pulse width
  uint16_t m_lpwmCommand = 0;

  // Last RPWM pulse width
  uint16_t m_rpwmCommand = 0;

  // Last velocity command
  double m_velocity = 0.0;

public:
  //
  // Constructors
  //

  motor(
    ros::NodeHandle node, std::string path);

  motor(
    ros::NodeHandle node,
    std::string path,
    std::string topic,
    int lpwm,
    int rpwm,
    int minPwm,
    int maxPwm,
    double minVelocity,
    double maxVelocity);

public:
  // Get current velocity
  inline double getVelocity()
  {
    return m_velocity;
  }

  // Get current LPWM pulse width
  inline int getLPWM()
  {
    return m_lpwmCommand;
  }

  // Get current RPWM pulse width
  inline int getRPWM()
  {
    return m_rpwmCommand;
  }

  // Load settings
  virtual bool configure();

  // Initialize
  virtual bool init();
  
  // Command velocity
  void command(double velocity);

public:
  // Create instance
  static controller* create(ros::NodeHandle node, std::string path);
};

} // namespace str1ker
