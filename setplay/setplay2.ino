#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Stepper.h>
#include <DabbleESP32.h>
#include <vector>

#define IN1 12
#define IN2 14
#define IN3 27
#define IN4 26
const int stepsPerRev = 2048;
int stepperSteps = 0;
int currentStepperPosition = 0;
int initialStepperPosition = 0;

Stepper myStepper(stepsPerRev, IN1, IN3, IN2, IN4);

#define speed1 15 // rpm for the stepper motor

// Motor control pins
const int enableRightMotor = 22;
const int rightMotorPin1 = 16;
const int rightMotorPin2 = 17;
const int enableLeftMotor = 23;
const int leftMotorPin1 = 18;
const int leftMotorPin2 = 19;

#define MAX_MOTOR_SPEED 150 // Define the maximum motor speed

struct RecordedStep {
    int rightMotorSpeed;
    int leftMotorSpeed;
    int stepperSteps;
    int duration;
};

std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;
bool isPlaying = false; // Flag to prevent input during playback
bool stopAll = false;
unsigned long previousTimeInMilli = millis();

const int PWMFreq = 1000; // PWM frequency
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 0;
const int leftMotorPWMSpeedChannel = 1;

int initialRightMotorSpeed = 0;
int initialLeftMotorSpeed = 0;

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor at 115200 baud
    Dabble.begin("FORKLIFT"); // Initialize Dabble with Bluetooth name
    setUpPinModes();
    Serial.println("Bluetooth Device is Ready to Pair");
    myStepper.setSpeed(speed1);
}

void loop() {
      if (stopAll) {
        rotateMotor(0, 0); // Stop motors immediately
        return;
    }

    Dabble.processInput();
    
    
    // Process commands only if not playing back
    if (!isPlaying) {
        if (GamePad.isSelectPressed()) {
            if (recordSteps) {
                recordSteps = false;
                Serial.println("Recording stopped");
            } else {
                recordSteps = true;
                recordedSteps.clear();
                previousTimeInMilli = millis();
                initialStepperPosition = currentStepperPosition; // Record initial position
                initialRightMotorSpeed = ledcRead(rightMotorPWMSpeedChannel);
                initialLeftMotorSpeed = ledcRead(leftMotorPWMSpeedChannel);

                Serial.println("Recording started");
            }
            Serial.print("Record Steps: ");
            Serial.println(recordSteps);
        } else if (GamePad.isStartPressed()) {
            if (playRecordedSteps) {
                playRecordedSteps = false;
                isPlaying = false;
                Serial.println("Playback stopped");
            } else {
                
                playRecordedSteps = true;
                recordSteps = false;
                isPlaying = true;
                Serial.println("Playing recorded steps");
                returnToInitialPosition(); // Move to initial position before playback
            }
            Serial.print("Play Recorded Steps: ");
            Serial.println(playRecordedSteps);
        } else {
            controlMotors();
        }
    }

    if (recordSteps) {
        recordCarStep();
    }

    if (playRecordedSteps) {
        playRecordedCarSteps();
        playRecordedSteps = false; // Ensure playback does not loop
        isPlaying = false;
    }
   // Check cross button state on Dabble Gamepad
    if (GamePad.isCrossPressed()) {
        delay(50); // Debounce delay
        if (GamePad.isCrossPressed()) {
            stopAll = !stopAll; // Toggle the stopAll flag
            if (stopAll) {
                stopAllMovements(); // Stop all movements immediately
            }
        }
        while (GamePad.isCrossPressed()); // Wait for button release
    }

    // If stopAll is active, skip all other logic
    if (stopAll) {
        return;
    }

    // Your regular playback or motor control logic here
    if (playRecordedSteps && !isPlaying) {
        playRecordedCarSteps();
    }

}

void setUpPinModes() {
    pinMode(enableRightMotor, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(enableLeftMotor, OUTPUT);
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);

    // Set up PWM for speed
    ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
    ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);
}

void controlMotors() {
     if (stopAll) {
      rotateMotor(0, 0); // Stop motors immediately
      return;
    }

    int rightMotorSpeed = 0;
    int leftMotorSpeed = 0;
    int Stepperspeed = 0;

    if (GamePad.isUpPressed()) {
        rightMotorSpeed = MAX_MOTOR_SPEED;
        leftMotorSpeed = MAX_MOTOR_SPEED;
        Serial.println("Moving Forward");
    } else if (GamePad.isDownPressed()) {
        rightMotorSpeed = -MAX_MOTOR_SPEED;
        leftMotorSpeed = -MAX_MOTOR_SPEED;
        Serial.println("Moving Backward");
    } else if (GamePad.isRightPressed()) {
        rightMotorSpeed = -MAX_MOTOR_SPEED;
        leftMotorSpeed = MAX_MOTOR_SPEED;
        Serial.println("Turning Right");
    } else if (GamePad.isLeftPressed()) {
        rightMotorSpeed = MAX_MOTOR_SPEED;
        leftMotorSpeed = -MAX_MOTOR_SPEED;
        Serial.println("Turning Left");
    } else if (GamePad.isTrianglePressed()) {
        stepperSteps = stepsPerRev / 20;
        myStepper.step(stepperSteps);
        currentStepperPosition += stepperSteps;
        Serial.println("Motor Up");
    } else if (GamePad.isSquarePressed()) {
        stepperSteps = -(stepsPerRev / 20);
        myStepper.step(stepperSteps);
        currentStepperPosition += stepperSteps;
        Serial.println("Motor Down");
    } else {
        Serial.println("Stopping");
        stepperSteps = 0;
    }

    // Add information about motor speed
    Serial.print("Right Motor Speed: ");
    Serial.print(rightMotorSpeed);
    Serial.print(", Left Motor Speed: ");
    Serial.println(leftMotorSpeed);
    Serial.print(", Stepper Motor Speed: ");
    Serial.println(stepperSteps);

    rotateMotor(rightMotorSpeed, leftMotorSpeed);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
    Serial.print("Rotating motors - Right Motor Speed: ");
    Serial.print(rightMotorSpeed);
    Serial.print(", Left Motor Speed: ");
    Serial.println(leftMotorSpeed);
     if (stopAll) {
        rotateMotor(0, 0); // Ensure motors stop immediately if stopAll is set
        return;
    }

    if (rightMotorSpeed < 0) {
        digitalWrite(rightMotorPin1, LOW);
        digitalWrite(rightMotorPin2, HIGH);
    } else if (rightMotorSpeed > 0) {
        digitalWrite(rightMotorPin1, HIGH);
        digitalWrite(rightMotorPin2, LOW);
    } else {
        digitalWrite(rightMotorPin1, LOW);
        digitalWrite(rightMotorPin2, LOW);
    }

    if (leftMotorSpeed < 0) {
        digitalWrite(leftMotorPin1, LOW);
        digitalWrite(leftMotorPin2, HIGH);
    } else if (leftMotorSpeed > 0) {
        digitalWrite(leftMotorPin1, HIGH);
        digitalWrite(leftMotorPin2, LOW);
    } else {
        digitalWrite(leftMotorPin1, LOW);
        digitalWrite(leftMotorPin2, LOW);
    }

    ledcWrite(0, abs(rightMotorSpeed));
    ledcWrite(1, abs(leftMotorSpeed));
}

void recordCarStep() {
        if (stopAll) {
        rotateMotor(0, 0); // Stop motors immediately
        return;
    }

    RecordedStep recordedStep;

    // Determine right motor direction and speed
    if (digitalRead(rightMotorPin1) == HIGH && digitalRead(rightMotorPin2) == LOW) {
        recordedStep.rightMotorSpeed = ledcRead(0); // Forward
    } else if (digitalRead(rightMotorPin1) == LOW && digitalRead(rightMotorPin2) == HIGH) {
        recordedStep.rightMotorSpeed = -ledcRead(0); // Backward
    } else {
        recordedStep.rightMotorSpeed = 0; // Stopped
    }

    // Determine left motor direction and speed
    if (digitalRead(leftMotorPin1) == HIGH && digitalRead(leftMotorPin2) == LOW) {
        recordedStep.leftMotorSpeed = ledcRead(1); // Forward
    } else if (digitalRead(leftMotorPin1) == LOW && digitalRead(leftMotorPin2) == HIGH) {
        recordedStep.leftMotorSpeed = -ledcRead(1); // Backward
    } else {
        recordedStep.leftMotorSpeed = 0; // Stopped
    }

    recordedStep.stepperSteps = currentStepperPosition;
    recordedStep.duration = millis() - previousTimeInMilli;
    previousTimeInMilli = millis();

    recordedSteps.push_back(recordedStep);

    Serial.print("Recorded Step - Right Motor Speed: ");
    Serial.print(recordedStep.rightMotorSpeed);
    Serial.print(", Left Motor Speed: ");
    Serial.println(recordedStep.leftMotorSpeed);
}

void playRecordedCarSteps() {
    if (stopAll) {
        rotateMotor(0, 0); // Stop motors immediately
        return;
    }


    if (recordedSteps.empty()) {
        Serial.println("No recorded steps to play.");
        return;
    }

    for (const auto& step : recordedSteps) {
        Serial.print("Playing step - Right Motor Speed: ");
        Serial.print(step.rightMotorSpeed);
        Serial.print(", Left Motor Speed: ");
        Serial.println(step.leftMotorSpeed);

        rotateMotor(step.rightMotorSpeed, step.leftMotorSpeed);
        if (stopAll) {
        stopAllMovements(); // Ensure everything stops immediately
        return;
        }

        delay(step.duration);
    }

    // Stop motors after playback
    rotateMotor(0, 0);
}

void stop() {
    rotateMotor(0, 0);
    Serial.println("Stopped");
}

void returnToInitialPosition() {
    int stepDifference = initialStepperPosition - currentStepperPosition;
    int steps = abs(stepDifference);
    int direction = (stepDifference > 0) ? 1 : -1;

    for (int i = 0; i < steps; ++i) {
        myStepper.step(direction);
        delayMicroseconds(500); // Adjust delay for smooth motion
    }
    currentStepperPosition = initialStepperPosition;
    Serial.println("Returned to initial position");
}

void stopAllMovements() {
    // Stop all motors immediately
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    ledcWrite(rightMotorPWMSpeedChannel, 0); // Set PWM to 0
    ledcWrite(leftMotorPWMSpeedChannel, 0);  // Set PWM to 0

    // Stop the stepper motor
    myStepper.step(0); // Stop the stepper motor

    // Reset all flags
    isPlaying = false;
    playRecordedSteps = false;
    recordSteps = false;
    stopAll = true; // Keep stopAll active until explicitly reset

    Serial.println("All movements stopped.");
}








