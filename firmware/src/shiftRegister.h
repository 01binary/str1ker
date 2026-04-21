/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 shiftRegister.h
 74HC595 output helper for chained shift registers
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Arduino.h>
#include <ShiftRegister74HC595.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class ShiftRegister
{
public:
  static const uint8_t MAX_SUPPORTED_REGISTERS = 4;
  static const uint8_t OUTPUTS_PER_REGISTER = 8;
  static const uint8_t MAX_SUPPORTED_OUTPUTS =
    MAX_SUPPORTED_REGISTERS * OUTPUTS_PER_REGISTER;

  ShiftRegister():
    output(nullptr),
    initialized(false),
    registerCount(0),
    outputCount(0)
  {
  }

  ~ShiftRegister()
  {
    if (output != nullptr)
    {
      delete output;
    }
  }

public:
  void initialize(
    uint8_t dataPin,
    uint8_t clockPin,
    uint8_t latchPin,
    uint8_t registers,
    uint8_t outputsCount,
    int defaultValue = LOW)
  {
    if (output != nullptr)
    {
      delete output;
      output = nullptr;
    }

    initialized = false;
    registerCount = registers;
    outputCount = outputsCount;

    output = new ShiftRegister74HC595<MAX_SUPPORTED_REGISTERS>(dataPin, clockPin, latchPin);
    if (output == nullptr)
    {
      return;
    }

    for (uint8_t i = 0; i < outputCount; i++)
    {
      output->set(i, defaultValue);
    }

    initialized = true;
  }

  void write(uint8_t outputPin, bool value)
  {
    if (initialized && outputPin < outputCount)
    {
      output->set(outputPin, value ? HIGH : LOW);
    }
  }

private:
  ShiftRegister74HC595<MAX_SUPPORTED_REGISTERS>* output;
  bool initialized;
  uint8_t registerCount;
  uint8_t outputCount;
};
