/*
  Object Avoidance Robot with Speed Control
  
  This code controls a robot to move forward and avoid obstacles.
  It uses an ultrasonic sensor to detect distance and a servo motor to "look" left and right.
  Motor speed is controlled using the ENA and ENB pins of the L298N motor driver.
*/

#include <Servo.h>      // Standard library for servo motor control
#include <NewPing.h>    // Library for the ultrasonic sensor

// Define L298N Motor Driver Pins
// Motor A (Left) 
const int ENA = 11; // Speed control (must be a PWM pin)
const int LeftMotorForward = 4;  // IN1
const int LeftMotorBackward = 5; // IN2

// Motor B (Right) 
const int ENB = 3; // Speed control (must be a PWM pin)
const int RightMotorForward = 6;  // IN3
const int RightMotorBackward = 7; // IN4

// Define Ultrasonic Sensor Pins
#define trig_pin A2
#define echo_pin A1

// Configuration
#define maximum_distance 200 // Maximum distance (in cm) to ping for
const int OBSTACLE_DISTANCE = 45; // Distance to trigger avoidance logic
int motorSpeed = 100; // Motor speed (0-255). Start with a lower value and increase.

// Initialize libraries
NewPing sonar(trig_pin, echo_pin, maximum_distance);
Servo servo_motor;

void setup() {
  // Set all motor control pins as Outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  // Attach the servo to its pin and center it
  servo_motor.attach(10);
  servo_motor.write(90); // Centered position
  delay(1000); // Wait for servo to center
}

void loop() {
  int distance = readPing();
  delay(50); // Small delay between pings

  if (distance <= OBSTACLE_DISTANCE) {
    // Obstacle detected!
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);

    // Look for the clearest path
    int distanceRight = lookRight();
    delay(300);
    int distanceLeft = lookLeft();
    delay(300);

    // Decide which way to turn
    if (distanceRight > distanceLeft) {
      turnRight();
    } else {
      turnLeft();
    }
  } else {
    // Path is clear, move forward
    moveForward();
  }
}

// Sensor Functions

int lookRight() {
  servo_motor.write(30); // Point servo to the right
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90); // Return to center
  return distance;
}

int lookLeft() {
  servo_motor.write(150); // Point servo to the left
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90); // Return to center
  return distance;
}

int readPing() {
  delay(70); // Wait 70ms between pings
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250; // If no ping, assume it's far away
  }
  return cm;
}

// Motor Control Functions

void moveStop() {
  // Set speed to 0 to stop the motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  // Also set direction pins to LOW as a good practice
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward() {
  // Set direction for both motors to forward
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  
  // Set the speed
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveBackward() {
  // Set direction for both motors to backward
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);

  // Set the speed
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  // Left motor forward, Right motor backward
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  
  // Set the speed
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  
  delay(350); // Turn duration - adjust as needed
  
  // After turning, continue forward
  moveForward();
}

void turnLeft() {
  // Left motor backward, Right motor forward
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);

  // Set the speed
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  
  delay(350); // Turn duration - adjust as needed
  
  // After turning, continue forward
  moveForward();
}
