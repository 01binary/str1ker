#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>

// Place splash.h into Adafruit_SSD1305 folder on your machine to replace Adafruit logo
#include "splash.h"

/*
  SSD1305 pins (left to right)

  Screen    Arduino                 ItsyBitsy 3u4   Definition
 
  [1] GND   GND                     GND             Ground
  [2] 3V3   3V3                     3V3             Power
  
  [4] DC    [8] Digital (shifted)   [8] Digital     OLED_DC
  [7] SCLK  [13] SCLK (shifted)     SCLK            Hardware Clock Pin
  [8] DIN   [11] MOSI (shifted)     MOSI            Hardware MOSI Pin
  [15] CS   [10] CS (shifted)       [6] CS          OLED_CS
  [16] RST  [9] Digital (shifted)   [9] Digital     Reset Pin

  For 5V boards like Arduino Uno, everything goes through 5V to 3.3V level shifter
 
*/

/*
  When audio source is Left or Right audio channel from a bluetooth board such as Sunrom M28,
  the Analog Ground of the bluetooth board has to be connected to Arduino/Itsy Bitsy ground.
  In general, controller must always have common ground with audio signal or you get garbage.
*/

const int OLED_DC = 8;
const int OLED_CS = 10;
const int OLED_RESET = 9;
const int AUDIO = A2;

const int MIN = 0;
const int MAX = 1024;
const int BIAS = 512;
// Signal level for "no signal"
//const int THRESHOLD = -50;  // Uncomment if audio source is computer audio jack
const int THRESHOLD = -60;  // Uncomment if audio source is bluetooth
const int WIDTH = 128;
// Anti-alias smoothing multiplier
const int ALIAS = 2;
const int SAMPLES = WIDTH * ALIAS;
const int HEIGHT = 32;
const int HALF = HEIGHT / 2;
// Sample average count
const int AVG_COUNT = 32;
const double AVG_MUL = 0.1;
const int DRAW_DELAY = 50;

// Using hardware SPI
Adafruit_SSD1305 display(WIDTH, HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS, 7000000UL);

short aud[SAMPLES] = {0};
int sample = 0;

int minCur = MAX;
short minAvg[AVG_COUNT] = {0};
int minSample = 0;

int maxCur = MIN;
short maxAvg[AVG_COUNT] = {0};
int maxSample = 0;

void setup() {
  pinMode(AUDIO, INPUT);

  initDisplay();
}

void loop() {
  int value = analogRead(AUDIO) - BIAS;
 
  aud[sample++] = value;

  averageRange(value, &minCur, minAvg, &minSample, true);
  averageRange(value, &maxCur, maxAvg, &maxSample, false);
 
  if (sample >= SAMPLES)
  {
    sample = 0;

    tuneRange();
 
    draw();
  }
}

int averageRange(int value, int* current, short* samples, int* sample, bool minOrMax)
{
  if ((minOrMax && value < *current) || (!minOrMax && value > *current))
    *current = value;

  samples[*sample] = *current;
  *sample = *sample + 1;

  if (*sample >= AVG_COUNT)
  {
    int sum = 0;
   
    for (int n = 0; n < AVG_COUNT; ++n)
    {
      sum += samples[n];
    }

    *current = sum / AVG_COUNT;
    *sample = 0;
  }
}

void tuneRange()
{
  int localMin = MAX;
  int localMax = MIN;
 
  for (int n = 0; n < SAMPLES; ++n)
  {
    if (aud[n] < localMin)
      localMin = aud[n];
    else if (aud[n] > localMax)
      localMax = aud[n];
  }

  if (localMin > minCur)
  {
    minCur += ceil((localMin - minCur) * AVG_MUL);
  }

  if (localMax < maxCur)
  {
    maxCur -= ceil((maxCur - localMax) * AVG_MUL);
  }
}

void draw()
{
  if (minCur < THRESHOLD) {
    // Prevents from drawing too fast, which can look ugly and flickery
    if (DRAW_DELAY) delay(DRAW_DELAY);

    display.clearDisplay();
  
    int lastx = 0;
    int lasty = mapSample(aud[0]);
    int smoothy = lasty;
   
    for (int x = 0; x < WIDTH; ++x)
    {
      int sum = 0;
     
      for (int oversample = 0; oversample < ALIAS; ++oversample)
      {
        sum += aud[x];
      }
  
      int y = (mapSample(sum / ALIAS) + lasty + smoothy) / 3;
     
      display.drawLine(lastx, lasty, x, y, WHITE);
  
      smoothy = lasty;
      lasty = y;
      lastx = x;
    }
  
    display.display();
  } else {
    // When "no signal", present the logo
    drawSplash();
  }
}

inline int mapSample(int value)
{
  return max(map(value, minCur, maxCur, HEIGHT - 1, 0), 0);
}

void initDisplay()
{
  Serial.begin(9600);

  if (!display.begin(0x3C) ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }

  display.display();
  delay(1000);
  display.clearDisplay();

  Serial.println("Initialized");
}

void drawSplash()
{
  display.clearDisplay();
    
  display.drawBitmap(
    (WIDTH - splash2_width) / 2,
    (HEIGHT - splash2_height) / 2,
    splash2_data,
    splash2_width,
    splash2_height,
    1);
    
  display.display();
}