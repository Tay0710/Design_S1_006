// /*
// Reading MB1030 using interrupts
// */

#include <Arduino.h>

#define rxPin 16

// US L
#define pwPin1 15
volatile unsigned long pulseStart1 = 0;
volatile float distanceCm1 = 0;
volatile bool US_ready1 = false;

// US U
#define pwPin2 18
volatile unsigned long pulseStart2 = 0;
volatile float distanceCm2 = 0;
volatile bool US2_ready = false;

// US R
#define pwPin3 5
volatile unsigned long pulseStart3 = 0;
volatile float distanceCm3 = 0;
volatile bool US3_ready = false;

void US1_ISR()
{
  // Called when pwPin changes
  if (digitalRead(pwPin1) == HIGH)
  {
    // Rising edge
    pulseStart1 = micros();
    US_ready1 = false;
  }
  else
  {
    // Falling edge
    unsigned long pulseWidth = micros() - pulseStart1;
    distanceCm1 = pulseWidth / 57.87;
    US_ready1 = true;
  }
}

// ISR for sensor 2
void IRAM_ATTR US2_ISR()
{
  if (digitalRead(pwPin2) == HIGH)
  {
    pulseStart2 = micros();
    US2_ready = false;
  }
  else
  {
    unsigned long pulseWidth = micros() - pulseStart2;
    distanceCm2 = pulseWidth / 57.87;
    US2_ready = true;
  }
}

// ISR for sensor 2
void IRAM_ATTR US3_ISR()
{
  if (digitalRead(pwPin3) == HIGH)
  {
    pulseStart3 = micros();
    US2_ready = false;
  }
  else
  {
    unsigned long pulseWidth = micros() - pulseStart3;
    distanceCm3 = pulseWidth / 57.87;
    US3_ready = true;
  }
}

void setup()
{
  Serial.begin(115200);

  delay(2000);

  Serial.println("Hello World");

  pinMode(rxPin, OUTPUT);
  pinMode(pwPin1, INPUT_PULLUP);
  digitalWrite(rxPin, LOW);
  attachInterrupt(digitalPinToInterrupt(pwPin1), US1_ISR, CHANGE);

  pinMode(pwPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPin2), US2_ISR, CHANGE);

  pinMode(pwPin3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPin3), US3_ISR, CHANGE);

  Serial.println("MB1030 pulse width distance measurement");

  digitalWrite(rxPin, HIGH);
  delayMicroseconds(25);
  digitalWrite(rxPin, LOW);
  delayMicroseconds(25);

  pinMode(rxPin, INPUT);
}

void loop()
{
  if (US_ready1)
  {
    Serial.print("US L Distance: ");
    Serial.print(distanceCm1, 2);
    Serial.println(" cm");
    US_ready1 = false;
  }

  if (US2_ready)
  {
    Serial.print("US U Dist: ");
    Serial.print(distanceCm2, 2);
    Serial.println(" cm");
    US2_ready = false;
  }

  if (US3_ready)
  {
    Serial.print("US R Dist: ");
    Serial.print(distanceCm3, 2);
    Serial.println(" cm");
    US3_ready = false;
    // digitalWrite(rxPin, HIGH);
    // delayMicroseconds(25);
    // digitalWrite(rxPin, LOW);
  }
}