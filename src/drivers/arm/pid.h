
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pid.h
 PID Controller with Integral Limit
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const float K_MIN = 0.0;
const float K_MAX = 1000.0;
const float DEFAULT_KP = 1.0;
const float DEFAULT_KI = 0.1;
const float DEFAULT_KD = 0.1;
const float DEFAULT_IMIN = -1.0;
const float DEFAULT_IMAX = 1.0;
const float DEFAULT_TOLERANCE = 0.05;

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class PID
{
public:
  //
  // Configuration
  //

  float Kp;         // Proportional gain
  float Ki;         // Integral gain
  float Kd;         // Derivative gain
  float iMin;       // Min integral
  float iMax;       // Max integral
  float tolerance;  // Tolerance reaching the goal

  //
  // State
  //

  float goal;       // Goal position

  float pe;         // Proportional error
  float ie;         // Integral error
  float de;         // Derivative error

  float p;          // Proportional term
  float i;          // Integral term
  float d;          // Derivative term

  bool enabled;     // Whether the controller is enabled
  float elapsed;    // Elapsed time since start

public:
  PID():
    Kp(DEFAULT_KP),
    Ki(DEFAULT_KI),
    Kd(DEFAULT_KD),
    iMin(DEFAULT_IMIN),
    iMax(DEFAULT_IMAX),
    tolerance(DEFAULT_TOLERANCE),
    pe(0.0), ie(0.0), de(0.0),
    p(0.0), i(0.0), d(0.0),
    goal(0.0),
    enabled(false),
    elapsed(0.0)
  {
  }

public:
  void initialize(
    float proportionalGain = DEFAULT_KP,
    float integralGain = DEFAULT_KI,
    float derivativeGain = DEFAULT_KD,
    float integralMin = DEFAULT_IMIN,
    float integralMax = DEFAULT_IMAX,
    float positionTolerance = DEFAULT_TOLERANCE)
  {
    stop();
  
    Kp = proportionalGain;
    Ki = integralGain;
    Kd = derivativeGain;
    iMin = integralMin;
    iMax = integralMax;
    tolerance = positionTolerance;
  }

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    node.getParam((String("~") + group + "/Kp").c_str(), &Kp);
    node.getParam((String("~") + group + "/Ki").c_str(), &Ki);
    node.getParam((String("~") + group + "/Kd").c_str(), &Kd);
    node.getParam((String("~") + group + "/iMin").c_str(), &iMin);
    node.getParam((String("~") + group + "/iMax").c_str(), &iMax);
    node.getParam((String("~") + group + "/tolerance").c_str(), &tolerance);
  }

  void start(float goalPosition)
  {
    stop();
    goal = goalPosition;
    enabled = true;
  }

  void stop()
  {
    pe = 0.0;
    ie = 0.0;
    de = 0.0;
    p = 0.0;
    i = 0.0;
    d = 0.0;
    enabled = false;
    elapsed = 0.0;
  }

  float update(float position, float dt)
  {
    if (!enabled || dt <= 0.0f)
    {
      return 0;
    }

    // Calculate proportional error
    float error = goal - position;

    if (abs(error) <= tolerance)
    {
      stop();
      return 0;
    }

    // Calculate integral error
    ie += dt * error;

    // Limit integral error
    if (Ki && iMax != 0.0 && iMin != 0.0) {
      ie = constrain(ie, iMin / Ki, iMax / Ki);
    }

    // Calculate derivative error
    de = (error - pe) / dt;
    pe = error;

    // Calculate proportional contribution
    p = Kp * pe;

    // Calculate integral contribution
    i = Ki * ie;

    // Limit integral contribution
    if (Ki && iMax != 0.0 && iMin != 0) {
      i = constrain(i, iMin, iMax);
    }

    // Calculate derivative contribution
    d = Kd * de;

    // Accumulate elapsed time
    elapsed += dt;

    // Calculate command
    return p + i + d;
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[255] = {0};
    char kp_s[16], ki_s[16], kd_s[16], imin_s[16], imax_s[16], tol_s[16];

    dtostrf(Kp,        0, 4, kp_s);
    dtostrf(Ki,        0, 4, ki_s);
    dtostrf(Kd,        0, 4, kd_s);
    dtostrf(iMin,      0, 4, imin_s);
    dtostrf(iMax,      0, 4, imax_s);
    dtostrf(tolerance, 0, 4, tol_s);

    snprintf(buffer, sizeof(buffer),
            "%s: Kp=%s Ki=%s Kd=%s iMin=%s iMax=%s tolerance=%s",
            group, kp_s, ki_s, kd_s, imin_s, imax_s, tol_s);

    node.loginfo(buffer);
  }
};
