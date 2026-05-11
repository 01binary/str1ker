/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 audio.ino
 Audio Face Board Firmware
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int SERIAL_BAUD = 115200;       // USB serial console rate
const int STARTUP_DELAY = 250;        // Small boot delay before first redraw
const int OLED_CS_PIN = 0;            // SSD1309 chip select
const int OLED_RESET_PIN = 1;         // SSD1309 reset
const int OLED_SPI_CLOCK_HZ = 1000000;
const int OLED_FLIP_MODE = 1;         // Try the alternate COM scan direction
const int LED_PINS[] = {2, 3, 4, 5, 6};
const int LED_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
const int AUDIO_PIN = A0;             // Audio input after onboard bias divider
const int OLED_DC_PIN = 12;           // SSD1309 data/command

const int ADC_BITS = 12;              // Teensy ADC resolution
const int ADC_MAX = (1 << ADC_BITS) - 1;
const int ADC_BIAS = ADC_MAX / 2;
const int SIGNAL_THRESHOLD = -180;    // Rough gate for "no signal"
const int DISPLAY_WIDTH = 128;
const int DISPLAY_HEIGHT = 64;
const int DISPLAY_MIDLINE = DISPLAY_HEIGHT / 2;
const int ANTIALIAS_FACTOR = 2;       // Horizontal oversampling for smoother lines
const int SAMPLE_COUNT = DISPLAY_WIDTH * ANTIALIAS_FACTOR;
const int RANGE_AVERAGE_COUNT = 32;   // Moving average window for min/max tracking
const float RANGE_RELAX_FACTOR = 0.1f;
const int DRAW_DELAY_MS = 25;         // Prevents redraw flicker
const int LED_LEVEL_SMOOTHING = 3;    // Heavier smoothing => calmer level meter
const int LED_LEVEL_FLOOR = 24;       // Ignore low-level noise for bar graph
const int LED_LEVEL_CEILING = 180;    // Lower ceiling makes the meter more lively

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeSerial();
void initializeAdc();
void initializePins();
void initializeDisplay();
void captureSample();
void updateTrackedRange(int value, int* current, short* history, int* index, bool trackMinimum);
void tuneRange();
void drawFrame();
void drawWaveform();
void drawIdleFrame();
void updateLevelMeter();
int mapSample(int value);
int computeFrameLevel();
void writeLedChannel(int index, bool enabled);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

U8G2_SSD1309_128X64_NONAME0_1_4W_HW_SPI display(U8G2_R0, OLED_CS_PIN, OLED_DC_PIN, OLED_RESET_PIN);
short audioSamples[SAMPLE_COUNT] = {0};
short minHistory[RANGE_AVERAGE_COUNT] = {0};
short maxHistory[RANGE_AVERAGE_COUNT] = {0};
int sampleIndex = 0;
int minHistoryIndex = 0;
int maxHistoryIndex = 0;
int trackedMin = ADC_MAX;
int trackedMax = -ADC_MAX;
int smoothedLevel = 0;

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeSerial();
  initializeAdc();
  initializePins();
  initializeDisplay();

  delay(STARTUP_DELAY);
}

void loop()
{
  captureSample();

  if (sampleIndex < SAMPLE_COUNT)
  {
    return;
  }

  sampleIndex = 0;
  tuneRange();
  drawFrame();
}

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeSerial()
{
  Serial.begin(SERIAL_BAUD);

  while (!Serial && millis() < 2000)
  {
    delay(10);
  }
}

void initializeAdc()
{
  analogReadResolution(ADC_BITS);
  analogReadAveraging(8);
  pinMode(AUDIO_PIN, INPUT);
}

void initializePins()
{
  for (int led = 0; led < LED_COUNT; ++led)
  {
    pinMode(LED_PINS[led], OUTPUT);
    writeLedChannel(led, false);
  }
}

void initializeDisplay()
{
  display.setBusClock(OLED_SPI_CLOCK_HZ);
  display.begin();
  display.setContrast(255);
  display.setFlipMode(OLED_FLIP_MODE);

  drawIdleFrame();

  Serial.println("audio display ready");
}

void captureSample()
{
  int value = analogRead(AUDIO_PIN) - ADC_BIAS;

  audioSamples[sampleIndex++] = value;
  updateTrackedRange(value, &trackedMin, minHistory, &minHistoryIndex, true);
  updateTrackedRange(value, &trackedMax, maxHistory, &maxHistoryIndex, false);
}

void updateTrackedRange(int value, int* current, short* history, int* index, bool trackMinimum)
{
  if ((trackMinimum && value < *current) || (!trackMinimum && value > *current))
  {
    *current = value;
  }

  history[*index] = *current;
  *index += 1;

  if (*index < RANGE_AVERAGE_COUNT)
  {
    return;
  }

  int sum = 0;

  for (int n = 0; n < RANGE_AVERAGE_COUNT; ++n)
  {
    sum += history[n];
  }

  *current = sum / RANGE_AVERAGE_COUNT;
  *index = 0;
}

void tuneRange()
{
  int localMin = ADC_MAX;
  int localMax = -ADC_MAX;

  for (int n = 0; n < SAMPLE_COUNT; ++n)
  {
    if (audioSamples[n] < localMin)
    {
      localMin = audioSamples[n];
    }

    if (audioSamples[n] > localMax)
    {
      localMax = audioSamples[n];
    }
  }

  if (localMin > trackedMin)
  {
    trackedMin += int(ceilf((localMin - trackedMin) * RANGE_RELAX_FACTOR));
  }

  if (localMax < trackedMax)
  {
    trackedMax -= int(ceilf((trackedMax - localMax) * RANGE_RELAX_FACTOR));
  }

  if (trackedMax <= trackedMin)
  {
    trackedMin = localMin;
    trackedMax = localMax + 1;
  }
}

void drawFrame()
{
  if (trackedMin >= SIGNAL_THRESHOLD)
  {
    drawIdleFrame();

    for (int led = 0; led < LED_COUNT; ++led)
    {
      writeLedChannel(led, false);
    }

    return;
  }

  if (DRAW_DELAY_MS > 0)
  {
    delay(DRAW_DELAY_MS);
  }

  updateLevelMeter();
  drawWaveform();
}

void drawWaveform()
{
  display.firstPage();

  do
  {
    int lastX = 0;
    int lastY = mapSample(audioSamples[0]);
    int smoothY = lastY;

    for (int x = 0; x < DISPLAY_WIDTH; ++x)
    {
      int sum = 0;

      for (int oversample = 0; oversample < ANTIALIAS_FACTOR; ++oversample)
      {
        int index = (x * ANTIALIAS_FACTOR) + oversample;
        sum += audioSamples[index];
      }

      int y = (mapSample(sum / ANTIALIAS_FACTOR) + lastY + smoothY) / 3;
      display.drawLine(lastX, lastY, x, y);

      smoothY = lastY;
      lastY = y;
      lastX = x;
    }
  }
  while (display.nextPage());
}

void drawIdleFrame()
{
  display.firstPage();

  do
  {
    display.drawFrame(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    display.drawHLine(0, DISPLAY_HEIGHT / 4, DISPLAY_WIDTH);
    display.drawHLine(0, DISPLAY_MIDLINE, DISPLAY_WIDTH);
    display.drawHLine(0, (DISPLAY_HEIGHT * 3) / 4, DISPLAY_WIDTH);
    display.drawVLine(DISPLAY_WIDTH / 2, 0, DISPLAY_HEIGHT);
  }
  while (display.nextPage());
}

void updateLevelMeter()
{
  int frameLevel = computeFrameLevel();
  smoothedLevel = ((smoothedLevel * LED_LEVEL_SMOOTHING) + frameLevel) / (LED_LEVEL_SMOOTHING + 1);
  int litChannels = map(
    constrain(smoothedLevel, LED_LEVEL_FLOOR, LED_LEVEL_CEILING),
    LED_LEVEL_FLOOR,
    LED_LEVEL_CEILING,
    0,
    LED_COUNT
  );
  litChannels = constrain(litChannels, 0, LED_COUNT);

  for (int led = 0; led < LED_COUNT; ++led)
  {
    writeLedChannel(led, led < litChannels);
  }
}

int mapSample(int value)
{
  if (trackedMax <= trackedMin)
  {
    return DISPLAY_MIDLINE;
  }

  return constrain(
    map(value, trackedMin, trackedMax, DISPLAY_HEIGHT - 1, 0),
    0,
    DISPLAY_HEIGHT - 1
  );
}

int computeFrameLevel()
{
  long sum = 0;

  for (int n = 0; n < SAMPLE_COUNT; ++n)
  {
    sum += abs(audioSamples[n]);
  }

  return int(sum / SAMPLE_COUNT);
}

void writeLedChannel(int index, bool enabled)
{
  digitalWrite(LED_PINS[index], enabled ? LOW : HIGH);
}
