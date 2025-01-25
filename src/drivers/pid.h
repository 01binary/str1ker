/*
    pid.h
    Simple PID algorithm with integral limit.
*/

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
  //
  // Configuration
  //

  double Kp;         // Proportional gain
  double Ki;         // Integral gain
  double Kd;         // Derivative gain
  double iMin;       // Min integral
  double iMax;       // Max integral

  //
  // State
  //

  double goal;       // Goal position
  double tolerance;  // Tolerance reaching the goal

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
    Kp(1), Ki(0.1), Kd(0.1),
    tolerance(0.05),
    pe(0.0), ie(0.0), de(0.0),
    iMin(0), iMax(1.0),
    p(0.0), i(0.0), d(0.0),
    goal(0.0),
    enabled(false),
    elapsed(0.0)
  {
  }

public:
  void initialize(
    double Kp,
    double Ki,
    double Kd,
    double iMin,
    double iMax)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->iMin = iMin;
    this->iMax = iMax;
    this->pe = 0.0;
    this->ie = 0.0;
    this->de = 0.0;
    this->p = 0.0;
    this->i = 0.0;
    this->d = 0.0;
  }

  void begin(double goal, double tolerance)
  {
    this->goal = goal;
    this->tolerance = tolerance;
    this->enabled = true;
  }

  void end()
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
      reset();
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
