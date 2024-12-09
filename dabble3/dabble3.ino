#include <AFMotor.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>


//RXD 9 TXD 10= 4,3
// Define the motors using the Adafruit Motor Shield library
AF_DCMotor motor1(1);  // Motor 1
AF_DCMotor motor2(2);  // Motor 2
AF_DCMotor motor3(3);  // Motor 3
AF_DCMotor motor4(4);  // Motor 4

void setup() {
  Serial.begin(9600);  // Start the serial monitor to see the test progress
  
  // Test all motors at the beginning to ensure they are all initialized correctly
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  Dabble.begin(9600, 9, 10);
}

void loop() {
  Dabble.processInput();
  
  int x = GamePad.getXaxisData();
  int y = GamePad.getYaxisData();
  
  Serial.print("X-axis: ");
  Serial.print(x);
  Serial.print(" | Y-axis: ");
  Serial.println(y);

  // Adjust threshold values to ensure detection
  if (y > 5) {
    Serial.println("Moving forward");
    forward();
  } else if (y < -5) {
    Serial.println("Moving backward");
    backward();
  } else if (x < -5) {
    Serial.println("Turning left");
    turnleft();
  } else if (x > 5) {
    Serial.println("Turning right");
    turnright();
  } else {
    Serial.println("Stopping motors");
    stopMotors();
  }
}

void turnright() {
  motor3.run(FORWARD);
  motor1.run(BACKWARD);
}

void turnleft() {
  motor4.run(BACKWARD);
  motor2.run(FORWARD);
}

void forward() {
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void backward() {
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
