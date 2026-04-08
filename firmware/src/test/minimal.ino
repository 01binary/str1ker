void setup()
{
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
}

void loop()
{
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(20, HIGH);
  digitalWrite(21, HIGH);
  delay(1000);
}
