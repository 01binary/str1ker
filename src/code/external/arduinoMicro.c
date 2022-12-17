// Arduino Micro analog channels
const int CHANNELS = 12;

// Signature for each data frame
const unsigned char SIGNATURE[] = { 'a', 'd', 'c', 'd' };

// Pins on Arduino Micro that correspond to each channel
const int PINS[] =
{
  A0,   // Analog 0
  A1,   // Analog 1
  A2,   // Analog 2
  
  A3,   // Analog 3
  A4,   // Analog 4
  A5,   // Analog 5


  A6,   // Digital 4
  A7,   // Digital 6
  A8,   // Digital 8


  A9,   // Digital 9
  A10,  // Digital 10
  A11,  // Digital 12
};


// Data transfer structure to send values of all channels
struct SAMPLE
{
  unsigned char signature[4];
  short readings[CHANNELS];
};

struct SAMPLE sample;

void setup()
{
  Serial.begin(9600);

  for (int channel = 0; channel < CHANNELS; channel++)
    pinMode(PINS[channel], INPUT);
  
  memcpy(&sample.signature, SIGNATURE, sizeof(SIGNATURE));
}

void loop()
{
  for (int channel = 0; channel < CHANNELS; channel++)
  {
    sample.readings[channel] = analogRead(PINS[channel]);
  }

  Serial.write((unsigned char*)&sample, sizeof(sample));

  // This can be as fast as the Arduino
  delay(500);
}
