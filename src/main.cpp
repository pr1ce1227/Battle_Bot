#include <Arduino.h>
// throttle channel 3 on pin 10 
const int fwd_rev_pwm_in = 9;  // Input pin for PWM signal from RC receiver
const int lft_rgt_pwm_in = 6;  // Input pin for PWM signal from RC receiver
const int left_wheel_fwd_pin = 7;     // Output pin for DC motor
const int left_wheel_rev_pin = 8;     // Output pin for DC motor
const int right_wheel_fwd_pin = 4;     // Output pin for DC motor
const int right_wheel_rev_pin = 2;     // Output pin for DC motor
const int right_wheel_pwm_out = 3;     // Output pin for DC motor
const int left_wheel_pwm_out = 5;     // Output pin for DC motor

void setup() {
  pinMode(fwd_rev_pwm_in, INPUT);     // Set the PWM input pin as an input
  pinMode(lft_rgt_pwm_in, INPUT);     // Set the PWM input pin as an input

  pinMode(left_wheel_fwd_pin, OUTPUT);       // Set the motor pin as an output
  pinMode(left_wheel_rev_pin, OUTPUT);       // Set the motor pin as an output

  pinMode(right_wheel_fwd_pin, OUTPUT);       // Set the motor pin as an output
  pinMode(right_wheel_rev_pin, OUTPUT);       // Set the motor pin as an output

  pinMode(right_wheel_pwm_out, OUTPUT);       // Set the motor pin as an output
  pinMode(left_wheel_pwm_out, OUTPUT);       // Set the motor pin as an output




  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop() {

  // Grab the input pwm signals 
  int fwd_rev_pwmValue = pulseIn(fwd_rev_pwm_in, HIGH);  // Read the PWM signal
  // Serial.print("fwd_rev_PWM: ");
  // Serial.println(fwd_rev_pwmValue);

  int lft_rgt_pwmValue = pulseIn(lft_rgt_pwm_in, HIGH);  // Read the PWM signal
  // Serial.print("lft_rgt_PWM: ");
  // Serial.println(lft_rgt_pwmValue);

  int motorSpeed = map(fwd_rev_pwmValue, 990, 1992, 0, 255); // Map the PWM value to motor speed
  // Serial.print("Motor Speed: ");
  // Serial.println(motorSpeed);

  if (motorSpeed < 125) {
    motorSpeed = 125 - motorSpeed; 
  }
  else if (motorSpeed > 130) {
    motorSpeed = motorSpeed - 130; 
  }
  else{
    motorSpeed = 0; 
  }

  motorSpeed = motorSpeed * 2; 
  int left_motorSpeed = motorSpeed;
  int right_motorSpeed = motorSpeed;

  // Calculate left and right pwm 
  int turn_motorSpeed = map(lft_rgt_pwmValue, 980, 2000, 0, 255); // Map the PWM value to motor speed

  if (turn_motorSpeed < 125) {
    right_motorSpeed = motorSpeed * (float)((float)turn_motorSpeed  / 125); 
  }
  else if (turn_motorSpeed > 130) {
    left_motorSpeed = motorSpeed * (float)(130  / (float)turn_motorSpeed); 
  }
  else{
    right_motorSpeed = motorSpeed;
    left_motorSpeed = motorSpeed; 
  }

  Serial.print("Left Motor Speed: ");
  Serial.println(left_motorSpeed);
  // Serial.print("Right Motor Speed: ");
  // Serial.println(right_motorSpeed);



  // Serial.print("Turn Motor Speed: ");
  // Serial.println(turn_motorSpeed);


  // Calculate left and right motor speed



  if (motorSpeed < 125) {
    // // Turn left
    analogWrite(left_wheel_pwm_out, left_motorSpeed); // Set the motor speed
    analogWrite(right_wheel_pwm_out, right_motorSpeed ); // Set the motor speed
  } 
  else if (motorSpeed > 130) {
    // // Turn right
    analogWrite(left_wheel_pwm_out, left_motorSpeed ); // Set the motor speed
    analogWrite(right_wheel_pwm_out, right_motorSpeed); // Set the motor speed
  }
  else{
    motorSpeed = 0; 
    analogWrite(left_wheel_pwm_out, motorSpeed); // Set the motor speed
    analogWrite(right_wheel_pwm_out, motorSpeed); // Set the motor speed
  }

}
