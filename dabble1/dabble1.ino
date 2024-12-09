#include <AFMotor.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>


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
  if (GamePad.isUpPressed())
  {
    forward();
  }

  if (GamePad.isDownPressed())
  {
    backward();
  }

  if (GamePad.isLeftPressed())
  {
   turnleft();
  }

  if (GamePad.isRightPressed())
  {
    turnright();
  }
  
}

void turnright(){
  //motor4.run(FORWARD);   // Run the motor forward
  motor3.run(FORWARD);            // Wait for 2 seconds
  motor1.run(BACKWARD);  // Run the motor backward
 
  
    
}
void turnleft(){
 
  motor4.run(BACKWARD); 
  motor2.run(FORWARD); 

  
    
}

void forward(){
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
}

void backward(){
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  

}
