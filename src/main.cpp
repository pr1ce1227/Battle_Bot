#include <Arduino.h>

const int pwmInputPin = 3;  // Input pin for PWM signal from RC receiver
const int motorPin = 5;     // Output pin for DC motor
const int powerPin = 6;     // Power control pin for DC motor

void setup() {
  pinMode(pwmInputPin, INPUT);     // Set the PWM input pin as an input
  pinMode(motorPin, OUTPUT);       // Set the motor pin as an output
  pinMode(powerPin, OUTPUT);       // Set the power control pin as an output
  digitalWrite(powerPin, HIGH);    // Turn on power to the motor

  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop() {
  int pwmValue = pulseIn(pwmInputPin, HIGH);  // Read the PWM signal
  Serial.print("PWM: ");
  Serial.println(pwmValue);

  int motorSpeed = map(pwmValue, 980, 2000, 0, 255); // Map the PWM value to motor speed
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);

  analogWrite(motorPin, motorSpeed); // Set the motor speed
}
