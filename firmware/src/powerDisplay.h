/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 powerDisplay.h
 SSD1306 Voltage / Current / Power Display
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class PowerDisplay
{
public:
  static constexpr uint8_t WIDTH = 128;
  static constexpr uint8_t HEIGHT = 64;
  static constexpr int8_t RESET_PIN = -1;
  static constexpr uint8_t I2C_ADDRESS = 0x3C;
  static constexpr unsigned long MODE_INTERVAL_MS = 5000;
  static constexpr unsigned long SAMPLE_INTERVAL_MS = 250;
  static constexpr unsigned long HISTORY_MS = 10000;
  static constexpr uint8_t SAMPLE_COUNT = HISTORY_MS / SAMPLE_INTERVAL_MS;

public:
  Adafruit_SSD1306 display;
  bool initialized;
  bool showWatts;
  unsigned long lastModeChangeMs;
  unsigned long lastSampleMs;
  float voltage;
  float current;
  float watts;
  float samples[SAMPLE_COUNT];
  uint8_t sampleIndex;
  uint8_t samplesStored;
  float accumulatedWatts;
  uint16_t accumulatedSamples;

public:
  PowerDisplay():
    display(WIDTH, HEIGHT, &Wire, RESET_PIN),
    initialized(false),
    showWatts(false),
    lastModeChangeMs(0),
    lastSampleMs(0),
    voltage(0.0f),
    current(0.0f),
    watts(0.0f),
    sampleIndex(0),
    samplesStored(0),
    accumulatedWatts(0.0f),
    accumulatedSamples(0)
  {
    for (uint8_t i = 0; i < SAMPLE_COUNT; i++)
    {
      samples[i] = 0.0f;
    }
  }

  bool initialize(uint8_t address = I2C_ADDRESS)
  {
    initialized = display.begin(SSD1306_SWITCHCAPVCC, address);

    if (!initialized)
    {
      return false;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.display();

    lastModeChangeMs = millis();
    lastSampleMs = 0;
    return true;
  }

  void update(float newVoltage, float newCurrent)
  {
    if (!initialized)
    {
      return;
    }

    unsigned long now = millis();
    voltage = newVoltage;
    current = newCurrent;
    watts = voltage * current;
    accumulatedWatts += watts;
    accumulatedSamples++;

    if (now - lastModeChangeMs >= MODE_INTERVAL_MS)
    {
      showWatts = !showWatts;
      lastModeChangeMs = now;
    }

    if (lastSampleMs == 0 || now - lastSampleMs >= SAMPLE_INTERVAL_MS)
    {
      writeSample(accumulatedWatts / float(accumulatedSamples));
      accumulatedWatts = 0.0f;
      accumulatedSamples = 0;
      lastSampleMs = now;
    }

    draw();
  }

private:
  void writeSample(float value)
  {
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;

    if (samplesStored < SAMPLE_COUNT)
    {
      samplesStored++;
    }
  }

  void draw()
  {
    display.clearDisplay();

    if (showWatts)
    {
      drawWatts();
    }
    else
    {
      drawVoltageCurrent();
    }

    display.display();
  }

  void drawVoltageCurrent()
  {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("BUS");

    drawValue(0, 14, voltage, "V", 2);
    drawValue(0, 40, current, "A", 2);
  }

  void drawWatts()
  {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("POWER");

    drawValue(0, 12, watts, "W", 1);
    drawGraph(0, 38, WIDTH, 26);
  }

  void drawValue(int16_t x, int16_t y, float value, const char* unit, uint8_t decimals)
  {
    display.setTextSize(2);
    display.setCursor(x, y);
    display.print(value, decimals);
    display.setTextSize(1);
    display.print(" ");
    display.print(unit);
  }

  void drawGraph(int16_t x, int16_t y, int16_t width, int16_t height)
  {
    display.drawRect(x, y, width, height, SSD1306_WHITE);

    if (samplesStored == 0)
    {
      return;
    }

    float maxWatts = 1.0f;

    for (uint8_t i = 0; i < samplesStored; i++)
    {
      maxWatts = max(maxWatts, samples[i]);
    }

    int16_t graphLeft = x + 1;
    int16_t graphTop = y + 1;
    int16_t graphWidth = width - 2;
    int16_t graphHeight = height - 2;
    int16_t previousX = graphLeft;
    int16_t previousY = graphTop + graphHeight;

    for (uint8_t i = 0; i < samplesStored; i++)
    {
      uint8_t sampleOffset = samplesStored - 1 - i;
      uint8_t index = (sampleIndex + SAMPLE_COUNT - 1 - sampleOffset) % SAMPLE_COUNT;
      float normalized = constrain(samples[index] / maxWatts, 0.0f, 1.0f);
      int16_t newestOffset = samplesStored - 1 - i;
      int16_t px = graphLeft + graphWidth - 1 - map(newestOffset, 0, SAMPLE_COUNT - 1, 0, graphWidth - 1);
      int16_t py = graphTop + graphHeight - int16_t(normalized * float(graphHeight));

      if (i > 0)
      {
        display.drawLine(previousX, previousY, px, py, SSD1306_WHITE);
      }

      previousX = px;
      previousY = py;
    }
  }
};
