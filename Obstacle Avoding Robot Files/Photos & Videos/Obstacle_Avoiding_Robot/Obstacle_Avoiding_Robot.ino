#include <AFMotor.h> // Motor Driver Library
#include <NewPing.h> // Ultrasonic Sensor Library
#include <Servo.h> // Servo Motor Library

// Define pins for ultrasonic sensor
#define TRIG_PIN A0
#define ECHO_PIN A5

// Define maximum distance for ultrasonic sensor
#define MAX_DISTANCE 300  // Maximum distance in cm

// Define motor speed constants
#define MAX_SPEED 200        // Maximum speed for the motors (0-255)
#define MAX_SPEED_OFFSET 20  // Offset applied to the calculated speed

#define INITIAL_SERVO_POSITION 115       // Initial servo position (center)
#define LEFT_LOOKING_SERVO_POSITION 170  // Servo position for looking left
#define RIGHT_LOOKING_SERVO_POSITION 50  // Servo position for looking right


// Create motor objects
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Create a NewPing object for ultrasonic sensor
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);


int speedSet = 0;     // Speed setpoint
int distanceF = 0;  // Forward distance
int distanceR = 0;    // Right distance
int distanceL = 0;    // Left distance

Servo servo;  // Servo initialized


void setup() {

  servo.attach(10);
  servo.write(INITIAL_SERVO_POSITION);
  delay(200);

  /*
  Take multiple distance readings using a for loop
  Reason: To assess the initial environment and potentially smooth out 
  sensor noise by averaging multiple readings.
  */

  for (int i = 0; i < 4; i = i++) {
    distanceF = readPing();
    delay(100);
  }
}

void loop() {
  // Check if the forward distance is too close
  if (distanceF <= 15) {
    // Stop the robot and take a short pause
    stop();
    delay(100);

    // Measure the distance to the right and left
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    // Choose a direction based on the measured distances
    if (distanceR > distanceL) {
      // Turn right if the right distance is greater
      goRight();
      delay(500);
      stop();
    } else {
      // Turn left if the left distance is greater
      goLeft();
      delay(500);
      stop();
    }
  } else {
    // If the forward distance is safe, move forward
    goForward();
  }

  // Update the forward distance for the next iteration
  distanceF = readPing();
}

int lookRight() {
  // Move the servo to the "look right" position
  servo.write(RIGHT_LOOKING_SERVO_POSITION);
  delay(100);

  // Read the distance with the servo pointing right
  int newDistance = readPing();
  return newDistance;
}

int lookLeft() {
  // Move the servo to the "look left" position
  servo.write(LEFT_LOOKING_SERVO_POSITION);
  delay(100);

  // Read the distance with the servo pointing left
  int newDistance = readPing();
  return newDistance;
}


int readPing() {
  // Delay for sensor stabilization
  delay(70);

  // Measure distance in centimeters
  int distanceInCentimeter = sonar.ping_cm();

  // Return the measured distance
  return distanceInCentimeter;
}


// Function for forward movement
void goForward() {
  // Set direction for all motors to forward
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  // Slowly bring the speed up to avoid loading down the batteries too quickly
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += MAX_SPEED_OFFSET) {
    // Set speed for all motors
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

// Function for backward movement
void goBackward() {
  // Set direction for all motors to backward
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  // Slowly bring the speed up to avoid loading down the batteries too quickly
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += MAX_SPEED_OFFSET) {
    // Set speed for all motors
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

// Function for right movement
void goRight() {
  int speedForBackward = 100;
  // Set direction for all motors to backward
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  // Set lower speed for right motors
  motor3.setSpeed(speedForBackward);
  motor4.setSpeed(speedForBackward);
}


// Function for left movement
void goLeft() {
  int speedForBackward = 100;
  // Set direction for all motors to backward
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  // Set lower speed for left motors
  motor1.setSpeed(speedForBackward);
  motor2.setSpeed(speedForBackward);
}

// Function for stop movement
void stop() {
  // Set all motors to the "RELEASE" state, which stops them.
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
