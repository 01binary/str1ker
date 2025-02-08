/*
    pid.h
    Simple PID algorithm with integral limit.
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class PID
{
public:
  static const double DEFAULT_KP = 1.0;
  static const double DEFAULT_KI = 0.1;
  static const double DEFAULT_KD = 0.1;
  static const double DEFAULT_IMIN = -1.0;
  static const double DEFAULT_IMAX = 1.0;
  static const double DEFAULT_TOLERANCE = 0.05;

public:
  //
  // Configuration
  //

  double Kp;         // Proportional gain
  double Ki;         // Integral gain
  double Kd;         // Derivative gain
  double iMin;       // Min integral
  double iMax;       // Max integral
  double tolerance;  // Tolerance reaching the goal

  //
  // State
  //

  double goal;       // Goal position

  double pe;         // Proportional error
  double ie;         // Integral error
  double de;         // Derivative error

  double p;          // Proportional term
  double i;          // Integral term
  double d;          // Derivative term

  bool enabled;      // Whether the controller is enabled
  double elapsed;    // Elapsed time since start

public:
  PID():
    Kp(DEFAULT_KP),
    Ki(DEFAULT_KI),
    Kd(DEFAULT_KD),
    iMin(DEFAULT_IMAX),
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
    double proportionalGain = DEFAULT_KP,
    double integralGain = DEFAULT_KI,
    double derivativeGain = DEFAULT_KD,
    double integralMin = DEFAULT_IMIN,
    double integralMax = DEFAULT_IMAX,
    double positionTolerance = DEFAULT_TOLERANCE)
  {
    stop();
  
    Kp = proportionalGain;
    Ki = integralGain;
    Kd = derivativeGain;
    iMin = integralMin;
    iMax = integralMax;
    tolerance = positionTolerance;
  }

  void readSettings()
  {
    Kp = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    Ki = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    Kd = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    iMin = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    iMax = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    tolerance = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
  }

  void writeSettings()
  {
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), Kp);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), Ki);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), Kd);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), iMin);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), iMax);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), tolerance);
  }

  void start(double goalPosition)
  {
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

  double update(double position, double dt)
  {
    if (!enabled)
    {
      return 0;
    }

    // Calculate proportional error
    double error = goal - position;

    if (abs(error) <= tolerance)
    {
      stop();
      return 0;
    }

    // Calculate integral error
    ie += dt * pe;

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
};
