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

#define speed1 5 //rpm for the stepper motor

// Motor control pins
const int enableRightMotor = 22;
const int rightMotorPin1 = 16;
const int rightMotorPin2 = 17;
const int enableLeftMotor = 23;
const int leftMotorPin1 = 18;
const int leftMotorPin2 = 19;

#define MAX_MOTOR_SPEED 150  // Define the maximum motor speed

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
unsigned long previousTimeInMilli = millis();

const int PWMFreq = 20000; // PWM frequency
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 0;
const int leftMotorPWMSpeedChannel = 1;

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor at 115200 baud
    Dabble.begin("FORKLIFT"); // Initialize Dabble with Bluetooth name
    setUpPinModes();
    Serial.println("Bluetooth Device is Ready to Pair");
    myStepper.setSpeed(speed1);
}

void loop() {
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
        playRecordedSteps = false;  // Ensure playback does not loop
        isPlaying = false;
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
        rightMotorSpeed = MAX_MOTOR_SPEED;
        leftMotorSpeed = -MAX_MOTOR_SPEED;
        Serial.println("Turning Right");
    } else if (GamePad.isLeftPressed()) {
        rightMotorSpeed = -MAX_MOTOR_SPEED;
        leftMotorSpeed = MAX_MOTOR_SPEED;
        Serial.println("Turning Left");
    } else if(GamePad.isTrianglePressed()){
      stepperSteps = stepsPerRev / 20;
      myStepper.step(stepperSteps);
      currentStepperPosition += stepperSteps;
      Serial.println("Motor Up");
    } else if (GamePad.isSquarePressed()){
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

    ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
    ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

void recordCarStep() {
    unsigned long currentTime = millis();
    RecordedStep recordedStep;
    recordedStep.rightMotorSpeed = ledcRead(rightMotorPWMSpeedChannel);
    recordedStep.leftMotorSpeed = ledcRead(leftMotorPWMSpeedChannel);
    recordedStep.stepperSteps = currentStepperPosition;
    recordedStep.duration = currentTime - previousTimeInMilli;
    recordedSteps.push_back(recordedStep);
    previousTimeInMilli = currentTime;

    Serial.print("Recorded Step - Right Motor Speed: ");
    Serial.print(recordedStep.rightMotorSpeed);
    Serial.print(", Left Motor Speed: ");
    Serial.print(recordedStep.leftMotorSpeed);
    Serial.print(", Duration: ");
    Serial.print(", Stepper Motor Speed: ");
    Serial.println(stepperSteps);
    Serial.println(recordedStep.duration);
}

void playRecordedCarSteps() {
    if (recordedSteps.empty()) {
        playRecordedSteps = false;
        isPlaying = false;
        return;
    }

    Serial.println("Playing Recorded Steps");

    for (const auto &step : recordedSteps) {
        rotateMotor(step.rightMotorSpeed, step.leftMotorSpeed);
        int totalSteps = step.stepperSteps - currentStepperPosition;
        int steps = abs(totalSteps);
        int direction = (totalSteps > 0) ? 1 : -1;

        for (int i = 0; i < steps; ++i) {
            myStepper.step(direction);
            delayMicroseconds(500); // Adjust delay for smooth motion
        }
        currentStepperPosition = step.stepperSteps;

        Serial.print("Playing step - Right Motor Speed: ");
        Serial.print(step.rightMotorSpeed);
        Serial.print(", Left Motor Speed: ");
        Serial.print(step.leftMotorSpeed);
        Serial.print(", Duration: ");
        Serial.println(step.duration);
    }

    // Return to initial position
    stop();
    Serial.println("Finished Playing Recorded Steps");
    isPlaying = false; // Allow input processing again
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
