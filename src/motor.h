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
  std::string m_topic = "robot/pwm";

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
  // Controller type
  static const char TYPE[];

public:
  //
  // Constructor
  //

  motor(class robot& robot, const char* path);
  motor(
    class robot& robot,
    const char* path,
    std::string topic,
    int lpwm,
    int rpwm,
    int minPwm,
    int maxPwm,
    double minVelocity,
    double maxVelocity);

public:
  // Get display type
  virtual const char* getType();

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
  virtual void configure(ros::NodeHandle node);

  // Initialize
  virtual bool init(ros::NodeHandle node);
  
  // Command velocity
  void command(double velocity);

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
