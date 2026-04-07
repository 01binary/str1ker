/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 leg.h
 Leg programming interface
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const float TWO_PI_F = 6.28318530718f;        // One full revolution in radians
const float ENCODER_PPR = 1000.0;             // AS5047 pulses per revolution
const float ENCODER_CPR = ENCODER_PPR * 4.0f; // AS5047 counts per revolution

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

struct Leg
{
  const char* name;
  int actuatorPotPin;
  int actuatorEnablePin;
  uint8_t actuatorLeftChannel;
  uint8_t actuatorRightChannel;
  int wheelEnablePin;
  uint8_t wheelLeftChannel;
  uint8_t wheelRightChannel;
  Potentiometer actuatorSensor;
  Pca9685Motor actuatorMotor;
  Pca9685Motor wheelMotor;
  WheelEncoder wheelEncoder;

  float actuatorCommand;
  float wheelCommand;
  float actuatorPosition;
  float wheelRevolutions;
  float wheelRadians;
  float wheelVelocity;

  Leg(
    const char* legName,
    Adafruit_PWMServoDriver& pwmDriver,
    int actuatorPotPin,
    int actuatorEnablePin,
    uint8_t actuatorLeftChannel,
    uint8_t actuatorRightChannel,
    int wheelEnablePin,
    uint8_t wheelLeftChannel,
    uint8_t wheelRightChannel,
    uint8_t wheelPinA,
    uint8_t wheelPinB,
    uint8_t wheelIndexPin):
    name(legName),
    actuatorPotPin(actuatorPotPin),
    actuatorEnablePin(actuatorEnablePin),
    actuatorLeftChannel(actuatorLeftChannel),
    actuatorRightChannel(actuatorRightChannel),
    wheelEnablePin(wheelEnablePin),
    wheelLeftChannel(wheelLeftChannel),
    wheelRightChannel(wheelRightChannel),
    actuatorMotor(pwmDriver),
    wheelMotor(pwmDriver),
    wheelEncoder(wheelPinA, wheelPinB, wheelIndexPin, ENCODER_CPR),
    actuatorCommand(0.0f),
    wheelCommand(0.0f),
    actuatorPosition(0.0f),
    wheelRevolutions(0.0f),
    wheelRadians(0.0f),
    wheelVelocity(0.0f)
  {
  }

  void initialize()
  {
    actuatorSensor.initialize(actuatorPotPin);
    actuatorMotor.initialize(actuatorEnablePin, actuatorLeftChannel, actuatorRightChannel);
    wheelMotor.initialize(wheelEnablePin, wheelLeftChannel, wheelRightChannel);
    actuatorSensor.read();
    wheelEncoder.initialize();
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
    actuatorMotor.stop();
    wheelMotor.stop();
  }

  void update(float timeStep)
  {
    actuatorPosition = actuatorSensor.read();
    wheelEncoder.update(timeStep);

    wheelRevolutions = wheelEncoder.revolutions;
    wheelRadians = wheelRevolutions * TWO_PI_F;
    wheelVelocity = wheelEncoder.velocity;

    actuatorMotor.write(actuatorCommand);
    wheelMotor.write(wheelCommand);
  }
};