


void playUSBModeChangeTone()
{
  // Signal: USB reader mode activated or normal mode resumed
  // First tone sequence: 2 kHz (high pitched)
  for( uint16_t n=0; n<200; n++)
  {
    delayMicroseconds(250);
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(250);
    digitalWrite(BUZZER, LOW);
  }
  // Second tone sequence: ~2.8 kHz (higher pitch)
  for( uint16_t n=0; n<200; n++)
  {
    delayMicroseconds(180);
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(180);
    digitalWrite(BUZZER, LOW);
  }
}


void playPowerOffTone()
{
  for( uint16_t n=0; n<200; n++)
  {
    delayMicroseconds(250);
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(250);
    digitalWrite(BUZZER, LOW);
  }
}

void playErrorTone()
{
  for( uint16_t n=0; n<200; n++)
  {
    delayMicroseconds(250);
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(250);
    digitalWrite(BUZZER, LOW);
  }
}

void playPowerOnTone()
{
  for( uint16_t n=0; n<200; n++)
  {
    delayMicroseconds(180);
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(180);
    digitalWrite(BUZZER, LOW);
  }
}

void playClickChangeTone()
{
   for( uint16_t n=0; n<200; n++)
    {
      delayMicroseconds(150);
      pinMode(BUZZER, OUTPUT);
      digitalWrite(BUZZER, HIGH);
      delayMicroseconds(150);
      pinMode(BUZZER, OUTPUT);
      digitalWrite(BUZZER, LOW);
    }
}

void blinkLeds(bool useLed1, bool useLed2,bool useLed3,uint16_t delayMs, uint8_t count)
{
  uint8_t led1=digitalRead(LED1);
  uint8_t led2=digitalRead(LED2);
  uint8_t led3=digitalRead(LED3); 

  for( uint8_t n=0; n<count; n++)
  {
    delay(delayMs);
    if(useLed1)
      digitalWrite(LED1, HIGH);
    if(useLed2)
      digitalWrite(LED2, HIGH);
    if(useLed3)
      digitalWrite(LED3, HIGH);
    delay(delayMs);
    if(useLed1)
      digitalWrite(LED1, LOW);
    if(useLed2)
      digitalWrite(LED2, LOW);
    if(useLed3)
      digitalWrite(LED3, LOW);
  }

  digitalWrite(LED1, led1);
  digitalWrite(LED2, led2);
  digitalWrite(LED3, led3);

}

