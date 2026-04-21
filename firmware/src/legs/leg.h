/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 leg.h
 Mobile robot leg controller
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <firmware.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const float ENCODER_PPR = 1000.0;             // AS5047 pulses per revolution
const float ENCODER_CPR = ENCODER_PPR * 4.0f; // AS5047 counts per revolution
const int MOTOR_CURRENT_SENSE_UNUSED = -1;    // Legs board motor current sense not wired

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Leg
{
public:
  const char* name;
  int actuatorPotPin;
  int actuatorEnablePin;
  int actuatorLeftChannel;
  int actuatorRightChannel;
  int wheelEnablePin;
  int wheelLeftChannel;
  int wheelRightChannel;
  uint8_t wheelPinA;
  uint8_t wheelPinB;
  uint8_t wheelIndexPin;
  Motor::PwmWriter pwmWriter;
  Potentiometer actuatorSensor;
  Motor actuatorMotor;
  Motor wheelMotor;
  QuadratureEncoder* quadratureEncoder;

  float actuatorCommand;
  float wheelCommand;
  float actuatorPosition;
  float wheelRevolutions;
  float wheelRadians;
  float wheelVelocity;

public:
  Leg():
    name(""),
    actuatorPotPin(0),
    actuatorEnablePin(0),
    actuatorLeftChannel(0),
    actuatorRightChannel(0),
    wheelEnablePin(0),
    wheelLeftChannel(0),
    wheelRightChannel(0),
    wheelPinA(0),
    wheelPinB(0),
    wheelIndexPin(0),
    pwmWriter(nullptr),
    actuatorMotor(),
    wheelMotor(),
    quadratureEncoder(nullptr),
    actuatorCommand(0.0f),
    wheelCommand(0.0f),
    actuatorPosition(0.0f),
    wheelRevolutions(0.0f),
    wheelRadians(0.0f),
    wheelVelocity(0.0f)
  {
  }

  ~Leg()
  {
    if (quadratureEncoder != nullptr)
    {
      delete quadratureEncoder;
      quadratureEncoder = nullptr;
    }
  }

public:
  void initialize(
    const char* legName,
    int actuatorPotentiometerPin,
    int actuatorEnPin,
    int actuatorLChannel,
    int actuatorRChannel,
    int wheelEnPin,
    int wheelLChannel,
    int wheelRChannel,
    uint8_t quadratureA,
    uint8_t quadratureB,
    uint8_t indexPin,
    Motor::PwmWriter onPwmWrite)
  {
    name = legName;
    actuatorPotPin = actuatorPotentiometerPin;
    actuatorEnablePin = actuatorEnPin;
    actuatorLeftChannel = actuatorLChannel;
    actuatorRightChannel = actuatorRChannel;
    wheelEnablePin = wheelEnPin;
    wheelLeftChannel = wheelLChannel;
    wheelRightChannel = wheelRChannel;
    wheelPinA = quadratureA;
    wheelPinB = quadratureB;
    wheelIndexPin = indexPin;
    pwmWriter = onPwmWrite;

    if (quadratureEncoder != nullptr)
    {
      delete quadratureEncoder;
      quadratureEncoder = nullptr;
    }

    quadratureEncoder = new QuadratureEncoder(wheelPinA, wheelPinB, wheelIndexPin, ENCODER_CPR);

    actuatorSensor.initialize(actuatorPotPin);

    actuatorMotor.initialize(
      actuatorEnablePin,
      actuatorLeftChannel,
      actuatorRightChannel,
      MOTOR_CURRENT_SENSE_UNUSED,
      0,
      Motor::PWM_MAX,
      false,
      1.0f,
      pwmWriter
    );

    wheelMotor.initialize(
      wheelEnablePin,
      wheelLeftChannel,
      wheelRightChannel,
      MOTOR_CURRENT_SENSE_UNUSED,
      0,
      Motor::PWM_MAX,
      false,
      1.0f,
      pwmWriter
    );

    actuatorSensor.read();

    if (quadratureEncoder != nullptr)
    {
      quadratureEncoder->initialize();
    }

    stop();
  }

  void enable()
  {
    actuatorMotor.enable();
    wheelMotor.enable();
  }

  void disable()
  {
    actuatorMotor.disable();
    wheelMotor.disable();
  }

  void stop()
  {
    actuatorCommand = 0.0f;
    wheelCommand = 0.0f;
    actuatorMotor.write(0.0f);
    wheelMotor.write(0.0f);
  }

  void update(float timeStep)
  {
    actuatorPosition = actuatorSensor.read();
    if (quadratureEncoder != nullptr)
    {
      quadratureEncoder->update(timeStep);

      wheelRevolutions = quadratureEncoder->revolutions;
      wheelRadians = wheelRevolutions * TWO_PI;
      wheelVelocity = quadratureEncoder->velocity;
    }
    else
    {
      wheelRevolutions = 0.0f;
      wheelRadians = 0.0f;
      wheelVelocity = 0.0f;
    }

    actuatorMotor.write(actuatorCommand);
    wheelMotor.write(wheelCommand);
  }
};
