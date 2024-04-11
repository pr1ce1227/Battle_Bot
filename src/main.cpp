#include <Arduino.h>

const int fwd_rev_pwm_in = 3;  // Input pin for PWM signal from RC receiver
const int lft_rgt_pwm_in = 4;  // Input pin for PWM signal from RC receiver
const int left_wheel_fwd_pin = 2;     // Output pin for DC motor
const int left_wheel_rev_pin = 3;     // Output pin for DC motor
const int right_wheel_fwd_pin = 4;     // Output pin for DC motor
const int right_wheel_rev_pin = 5;     // Output pin for DC motor
const int right_wheel_pwm_out = 6;     // Output pin for DC motor
const int left_wheel_pwm_out = 7;     // Output pin for DC motor
const int powerPin = 9;     // Power control pin for DC motor

void setup() {
  pinMode(fwd_rev_pwm_in, INPUT);     // Set the PWM input pin as an input
  pinMode(lft_rgt_pwm_in, INPUT);     // Set the PWM input pin as an input

  pinMode(left_wheel_fwd_pin, OUTPUT);       // Set the motor pin as an output

  pinMode(powerPin, OUTPUT);       // Set the power control pin as an output
  digitalWrite(powerPin, HIGH);    // Turn on power to the motor

  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop() {
  // Grab the input pwm signals 
  int fwd_rev_pwmValue = pulseIn(fwd_rev_pwm_in, HIGH);  // Read the PWM signal
  Serial.print("fwd_rev_PWM: ");
  Serial.println(fwd_rev_pwmValue);

  int lft_rgt_pwmValue = pulseIn(lft_rgt_pwm_in, HIGH);  // Read the PWM signal
  Serial.print("lft_rgt_PWM: ");
  Serial.println(lft_rgt_pwmValue);

  int motorSpeed = map(fwd_rev_pwmValue, 980, 2000, 0, 255); // Map the PWM value to motor speed
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);

  // Calculate forward or reverse
  if (motorSpeed < 255/2) {
    // Reverse
    digitalWrite(left_wheel_fwd_pin, LOW); 
    digitalWrite(right_wheel_fwd_pin, LOW); 
    digitalWrite(left_wheel_rev_pin, HIGH); 
    digitalWrite(right_wheel_rev_pin, HIGH); 
  } else {
    // Forward
    digitalWrite(left_wheel_rev_pin, LOW); // Set the motor direction
    digitalWrite(right_wheel_rev_pin, LOW); // Set the motor direction
    digitalWrite(left_wheel_fwd_pin, HIGH); // Set the motor direction
    digitalWrite(right_wheel_fwd_pin, HIGH); // Set the motor direction
  }

  // Calculate left and right pwm 
  int turn_motorSpeed = map(lft_rgt_pwmValue, 980, 2000, 0, 255); // Map the PWM value to motor speed
  Serial.print("Turn Motor Speed: ");
  Serial.println(turn_motorSpeed);

  // Calculate left and right motor speed
  if (turn_motorSpeed < 255/2) {
    // Turn left
    analogWrite(left_wheel_fwd_pin, motorSpeed); // Set the motor speed
    analogWrite(right_wheel_fwd_pin, motorSpeed - turn_motorSpeed); // Set the motor speed
  } else {
    // Turn right
    analogWrite(left_wheel_fwd_pin, motorSpeed - turn_motorSpeed); // Set the motor speed
    analogWrite(right_wheel_fwd_pin, motorSpeed); // Set the motor speed
  }

}
