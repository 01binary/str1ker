/*
    solenoid.h
    Solenoid Driver
*/

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Solenoid
{
public:
  int triggerPin;

public:
  Solenoid(): triggerPin(0)
  {
  }

public:
  void initialize(int trigger)
  {
    triggerPin = trigger;
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
  }

  void write(double holdTime)
  {
    digitalWrite(triggerPin, HIGH);
    delay(int(holdTime * 1000));
    digitalWrite(triggerPin, LOW);
  }
}
