/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 wheel_encoder.h
 Quadrature wheel encoder with index pulse tracking
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Arduino.h>
#include <Encoder.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class QuadratureEncoder
{
public:
  uint8_t pinA;
  uint8_t pinB;
  uint8_t pinIndex;
  float countsPerRevolution;
  Encoder quadrature;
  volatile uint32_t indexCount;
  int32_t counts;
  int32_t previousCounts;
  int32_t deltaCounts;
  bool indexState;
  float revolutions;
  float velocity;

public:
  QuadratureEncoder(
    uint8_t quadratureA,
    uint8_t quadratureB,
    uint8_t index,
    float countsPerTurn):
    pinA(quadratureA),
    pinB(quadratureB),
    pinIndex(index),
    countsPerRevolution(countsPerTurn),
    quadrature(quadratureA, quadratureB),
    indexCount(0),
    counts(0),
    previousCounts(0),
    deltaCounts(0),
    indexState(false),
    revolutions(0.0f),
    velocity(0.0f)
  {
  }

public:
  void initialize()
  {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinIndex, INPUT);

    quadrature.write(0);

    indexCount = 0;
    counts = 0;
    previousCounts = 0;
    deltaCounts = 0;
    indexState = digitalRead(pinIndex);
    revolutions = 0.0f;
    velocity = 0.0f;
  }

  void handleIndexRise()
  {
    indexCount++;
    indexState = true;
  }

  void update(float timeStep)
  {
    counts = quadrature.read();
    deltaCounts = counts - previousCounts;
    previousCounts = counts;
    indexState = digitalRead(pinIndex);

    if (countsPerRevolution <= 0.0f)
    {
      revolutions = 0.0f;
      velocity = 0.0f;
      return;
    }

    revolutions = float(counts) / countsPerRevolution;

    if (timeStep > 0.0f)
    {
      velocity = float(deltaCounts) / countsPerRevolution / timeStep;
    }
    else
    {
      velocity = 0.0f;
    }
  }

  void reset()
  {
    quadrature.write(0);
    indexCount = 0;
    counts = 0;
    previousCounts = 0;
    deltaCounts = 0;
    revolutions = 0.0f;
    velocity = 0.0f;
  }
};
