#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Right motor
int enableRightMotor = 22; 
int rightMotorPin1 = 16;
int rightMotorPin2 = 17;

// Left motor
int enableLeftMotor = 23;
int leftMotorPin1 = 18;
int leftMotorPin2 = 19;

// DC motor
int motorPin1 = 12;
int motorPin2 = 14;

#define MAX_MOTOR_SPEED 150  // Define the maximum motor speed

// Variables to store the state of the buttons
bool forwardPressed = false;
bool reversePressed = false;
bool rightPressed = false;
bool leftPressed = false;
bool upPressed = false;
bool downPressed = false;

const int PWMFreq = 1000; // Increased PWM frequency to 20 KHz
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 0;
const int leftMotorPWMSpeedChannel = 1;

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor
  SerialBT.begin("Controller"); // Device name
  setUpPinModes();
  Serial.println("Bluetooth Device is Ready to Pair");
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print("Received: ");
    Serial.println(c); // Print the received character

    // Update button states based on the received character
    if (c == 'F') {
      forwardPressed = true;
    } else if (c == 'f') {
      forwardPressed = false;
    } else if (c == 'B') {
      reversePressed = true;
    } else if (c == 'b') {
      reversePressed = false;
    } else if (c == 'R') {
      rightPressed = true;
    } else if (c == 'r') {
      rightPressed = false;
    } else if (c == 'L') {
      leftPressed = true;
    } else if (c == 'l') {
      leftPressed = false;
    } else if (c == 'U') {
      upPressed = true;
    } else if (c == 'u') {
      upPressed = false;
    } else if (c == 'D') {
      downPressed = true;
    } else if (c == 'd') {
      downPressed = false;
    }
  }

  // Control motors based on the button states
  if (forwardPressed) {
    forward();
  } else if (reversePressed) {
    reverse();
  } else if (rightPressed) {
    turnRight();
  } else if (leftPressed) {
    turnLeft();
  } else if (upPressed) {
    up();
  } else if (downPressed) {
    down();
  } else {
    stop();
  }
}

void setUpPinModes() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  
  // Set up PWM for speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel); 

  // Initialize motors to stop
  stop();
}

void forward() {
  Serial.println("Moving Forward");
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
  ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
}

void reverse() {
  Serial.println("Moving Backward");
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
  ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
}

void turnRight() {
  Serial.println("Turning Right");
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
  ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
}

void turnLeft() {
  Serial.println("Turning Left");
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
  ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
}

void stop() {
  Serial.println("Stopping");
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  ledcWrite(rightMotorPWMSpeedChannel, 0);
  ledcWrite(leftMotorPWMSpeedChannel, 0);
}

void up() {
  Serial.println("Moving Up");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}

void down() {
  Serial.println("Moving Down");
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);
}
