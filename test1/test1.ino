#include <AFMotor.h>


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

  
}

void loop() {
 turnright();
 delay(200);
 forward();
 delay(200);
 backward();
 delay(200);
 turnright();
 delay(200);
  
}

void turnright(){
  //motor4.run(FORWARD);   // Run the motor forward
  motor3.run(FORWARD);            // Wait for 2 seconds
  motor1.run(BACKWARD);  // Run the motor backward
 
  //motor3.run(BACKWARD); 

  delay(2000); // Run the motor backward
  motor1.run(RELEASE);
  motor4.run(RELEASE);
  motor3.run(RELEASE);
  motor2.run(RELEASE);
    
}
void turnleft(){
 
  motor4.run(BACKWARD); 
  motor2.run(FORWARD); 

  delay(2000); 
  motor1.run(RELEASE);
  motor4.run(RELEASE);
  motor3.run(RELEASE);
  motor2.run(RELEASE);
    
}

void forward(){
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(2000); 
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}

void backward(){
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(2000); 
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}
