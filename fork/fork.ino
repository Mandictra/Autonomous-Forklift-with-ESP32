#include <Ps3Controller.h>
#include <SoftwareSerial.h>

// HC-05 Bluetooth module on pins 0 (RX) and 1 (TX)
SoftwareSerial BTSerial(0, 1); // RX, TX

// Right motor
int enableRightMotor = 3;
int rightMotorPin1 = 4;
int rightMotorPin2 = 6;
// Left motor
int enableLeftMotor = 5;
int leftMotorPin1 = 7;
int leftMotorPin2 = 8;
// DC motor
int Dcmotorpin1 = 9;
int Dcmotorpin2 = 10;

#define MAX_MOTOR_SPEED 200 // Its value can range from 0-255. 255 is maximum speed.

void notify() {
  // Control forward and backward motion with the left analog stick
  int leftStickY = Ps3.data.analog.stick.ly;
  if (leftStickY > 0) { // Move car Forward
    rotateMotor(leftStickY, leftStickY);
  } else if (leftStickY < 0) { // Move car Backward
    rotateMotor(leftStickY, leftStickY);
  } else { // Stop the car
    rotateMotor(0, 0);
  }

  // Control left and right motion with the right analog stick
  int rightStickX = Ps3.data.analog.stick.rx;
  if (rightStickX > 0) { // Turn car Right
    rotateMotor(rightStickX, -rightStickX);
  } else if (rightStickX < 0) { // Turn car Left
    rotateMotor(-rightStickX, rightStickX);
  } else { // Stop the car
    rotateMotor(0, 0);
  }
    delay(10);
}
 

void onConnect() {
  Serial.println("Connected!.");
}

void onDisconnect() {
  Serial.println("Disconnected!.");
}

void rotateDCMotor(bool direction) {
  if (direction) {
    digitalWrite(Dcmotorpin1, HIGH);
    digitalWrite(Dcmotorpin2, LOW);
  } else {
    digitalWrite(Dcmotorpin1, LOW);
    digitalWrite(Dcmotorpin2, HIGH);
  }
}

void stopDCMotor() {
  digitalWrite(Dcmotorpin1, LOW);
  digitalWrite(Dcmotorpin2, LOW);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Right Motor
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    analogWrite(enableRightMotor, -rightMotorSpeed);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(enableRightMotor, rightMotorSpeed);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(enableRightMotor, 0);
  }

  // Left Motor
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
    analogWrite(enableLeftMotor, -leftMotorSpeed);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    analogWrite(enableLeftMotor, leftMotorSpeed);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    analogWrite(enableLeftMotor, 0);
  }
}

void setUpPinModes() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(Dcmotorpin1, OUTPUT);
  pinMode(Dcmotorpin2, OUTPUT);
}

void setup() {
  setUpPinModes();
  Serial.begin(115200);
  BTSerial.begin(9600); // Start software serial communication with the HC-05
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin();
  Serial.println("Ready.");
  BTSerial.println("Enter AT commands:");
}

void loop() {
  // Check if data is available from the HC-05
  if (BTSerial.available()) {
    char data = BTSerial.read();
    Serial.write(data);
  }

  // Check if data is available from the serial monitor
  if (Serial.available()) {
    char data = Serial.read();
    BTSerial.write(data);
  }
}
