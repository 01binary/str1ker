/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 meter.h
 LED level meter with optional callback-based pin writer
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Arduino.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Meter
{
public:
  typedef void (*StatusWriter)(int statusId, bool enabled, void* context);

public:
  uint8_t levelCount;
  int* levelIds;
  float level;
  StatusWriter statusWriter;
  void* statusWriterContext;

public:
  Meter():
    levelCount(0),
    levelIds(nullptr),
    level(0.0f),
    statusWriter(nullptr),
    statusWriterContext(nullptr)
  {
  }

  ~Meter()
  {
    if (levelIds != nullptr)
    {
      delete[] levelIds;
      levelIds = nullptr;
    }
  }

public:
  void initialize(
    uint8_t levels,
    const int* ids,
    StatusWriter onStatusWrite = nullptr,
    void* onStatusWriteContext = nullptr)
  {
    if (levelIds != nullptr)
    {
      delete[] levelIds;
      levelIds = nullptr;
    }

    levelCount = levels;
    statusWriter = onStatusWrite;
    statusWriterContext = onStatusWriteContext;
    level = 0.0f;
    levelIds = new int[levelCount];

    for (uint8_t i = 0; i < levelCount; i++)
    {
      levelIds[i] = ids[i];

      if (statusWriter == nullptr)
      {
        pinMode(levelIds[i], OUTPUT);
      }

      writeLevel(levelIds[i], false);
    }
  }

  void write(float normalizedLevel)
  {
    if (!levelCount || !levelIds)
    {
      return;
    }

    level = constrain(normalizedLevel, 0.0f, 1.0f);

    int activeLevels = int(level * float(levelCount));

    if (level >= 1.0f)
    {
      activeLevels = levelCount;
    }

    for (uint8_t i = 0; i < levelCount; i++)
    {
      writeLevel(levelIds[i], int(i) < activeLevels);
    }
  }

  float read() const
  {
    return level;
  }

private:
  void writeLevel(int id, bool enabled)
  {
    if (statusWriter != nullptr)
    {
      statusWriter(id, enabled, statusWriterContext);
      return;
    }

    digitalWrite(id, enabled ? HIGH : LOW);
  }
};
