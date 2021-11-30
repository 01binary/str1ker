#define ARMS    4
#define SENSORS 3

const unsigned char SIGNATURE[] = { 'p', 'o', 't', 's' };

// Pins on Arduino Micro (which has 12 usable ADC pins)
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

struct ARM
{
  short sensors[SENSORS];
};

struct POTS
{
  unsigned char sig[4];
  struct ARM arms[ARMS];
};

struct POTS values;

void setup()
{
  Serial.begin(9600);
 
  for (int arm = 0; arm < ARMS; arm++)
  {
    for (int sensor = 0; sensor < SENSORS; sensor++)
    {
      pinMode(PINS[arm * SENSORS + sensor], INPUT);
    }
  }
  
  memcpy(&values.sig, SIGNATURE, sizeof(SIGNATURE));
}

void loop()
{
  for (int arm = 0; arm < ARMS; arm++)
  {
    for (int sensor = 0; sensor < SENSORS; sensor++)
    {
      values.arms[arm].sensors[sensor] = analogRead(PINS[arm * SENSORS + sensor]);
    }
  }

  Serial.write((unsigned char*)&values, sizeof(POTS));

  delay(500);
}