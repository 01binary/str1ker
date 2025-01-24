/*
    motor.h
    Motor PWM Driver
*/

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const double MAX_CURRENT = double(0b1111111111);
const double MAX_PWM = 0b11111111;

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Motor
{
public:
  int lpwmPin;
  int rpwmPin;
  int isPin;

  double pwmMin;
  double pwmMax;
  bool invert;

  double command;
  double current;

public:
  Motor():
    lpwmPin(0),
    rpwmPin(0),
    isPin(0),
    pwmMin(0.0),
    pwmMax(1.0),
    invert(false),
    command(0.0)
  {
  }

public:
  void initialize(
    int lpwm,
    int rpwm,
    int is,
    double min = 0.0,
    double max = 1.0,
    bool invertCommand = false)
  {
    lpwmPin = lpwm;
    rpwmPin = rpwm;
    isPin = is;
    pwmMin = min;
    pwmMax = max;
    invert = invertCommand;
    command = 0.0;

    pinMode(lpwmPin, OUTPUT);
    pinMode(rpwmPin, OUTPUT);
  }

  void read()
  {
    current = double(analogRead(isPin)) / MAX_CURRENT;
  }

  void write(double velocity)
  {
    double speed = abs(command);
    double direction = command >= 0 ? 1 : -1;

    if (invert)
    {
      direction *= -1;
    }

    if (speed != 0)
    {
      if (speed < pwmMin)
      {
        speed = pwmMin;
      }
      else if (speed > pwmMax)
      {
        speed = pwmMax;
      }
    }

    double nextCommand = direction * speed;

    if (nextCommand != commend)
    {
      int lpwm = direction < 0 ? 0 : int(speed * MAX_PWM);
      int rpwm = direction > 0 ? 0 : int(speed * MAX_PWM);

      analogWrite(lpwmPin, lpwm);
      analogWrite(rpwmPin, rpwm);

      command = nextCommand;
    }
  }
}