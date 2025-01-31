
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motor.h
 Motor PWM driver
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Motor
{
public:
  const unsigned int CURRENT_MAX = 0b1111111111;
  const unsigned int PWM_MAX = 0b11111111;

public:
  int lpwmPin;
  int rpwmPin;
  int isPin;

  double pwmMin;
  double pwmMax;
  double stallThreshold;
  bool invert;

  double velocity;
  double current;

public:
  Motor():
    lpwmPin(0),
    rpwmPin(0),
    isPin(0),
    pwmMin(0.0),
    pwmMax(1.0),
    invert(false),
    stallThreshold(1.0),
    velocity(0.0),
    current(0.0)
  {
  }

public:
  void initialize(
    int lpwm,
    int rpwm,
    int is,
    double min = 0.0,
    double max = 1.0,
    bool invertCommand = false,
    double stallCurrentThreshold = 1.0)
  {
    lpwmPin = lpwm;
    rpwmPin = rpwm;
    isPin = is;
    pwmMin = min;
    pwmMax = max;
    invert = invertCommand;
    stallThreshold = stallCurrentThreshold;
    velocity = 0.0;

    pinMode(lpwmPin, OUTPUT);
    pinMode(rpwmPin, OUTPUT);
  }

  void readSettings()
  {
    pwmMin = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)), 0.0);
    pwmMax = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)), 1.0);
    stallThreshold = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)), 1.0);
    invert = EEPROM.readBool(EEPROM.getAddress(sizeof(bool)), false);
  }

  void writeSettings()
  {
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), pwmMin);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), pwmMax);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), stallThreshold);
    EEPROM.writeBool(EEPROM.getAddress(sizeof(bool)), invert);
  }

  void read()
  {
    current = double(analogRead(isPin)) / double(CURRENT_MAX);
  }

  void write(double command)
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
      int lpwm = direction < 0 ? 0 : int(speed * double(PWM_MAX));
      int rpwm = direction > 0 ? 0 : int(speed * double(PWM_MAX));

      analogWrite(lpwmPin, lpwm);
      analogWrite(rpwmPin, rpwm);

      command = nextCommand;
    }
  }
}