// Include the AccelStepper and MultiStepper Library
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define pin connections
const int dirPin1 = 2;
const int stepPin1 = 3;
const int dirPin2 = 4;
const int stepPin2 = 5;
const int dirPin3 = 6;
const int stepPin3 = 7;

#define motorPin1  8     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motorPin2  9     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motorPin3  10    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define motorPin4  11    // IN4 on ULN2003 ==> Orange on 28BYJ-48

#define motorPin5 22
#define motorPin6 23
#define motorPin7 24
#define motorPin8 25

// Creates an instance of each motor
AccelStepper motor1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor2(AccelStepper::DRIVER, stepPin2, dirPin2);
// AccelStepper motor3(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper motor3(AccelStepper::HALF4WIRE, motorPin5, motorPin7, motorPin6, motorPin8);
AccelStepper motor4(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);

MultiStepper motors;

const int dof = 4; // CHANGE VALUE to number of motors
long thetas[dof]; // initializing theta array
bool receivedThetas[dof] = {false}; // Array to track received inputs

void setup() {
  Serial.begin(9600);

	// set the maximum speed, acceleration factor,
	// initial speed, and set up multistepper
	motor1.setMaxSpeed(200);
  motor2.setMaxSpeed(100);
	motor3.setMaxSpeed(200);
  motor4.setMaxSpeed(200);
  
  motors.addStepper(motor1);
  motors.addStepper(motor2);
  motors.addStepper(motor3);
  motors.addStepper(motor4);
}

float readFloatFromSerial(int motorIndex) {
  Serial.print("Enter theta ");
  Serial.print(motorIndex);
  Serial.print(": ");
  
  while (true) {
    if (Serial.available() > 0) {
      float receivedFloat = Serial.parseFloat();
      if (Serial.read() == '\n') {
        Serial.print("Received theta ");
        Serial.print(motorIndex);
        Serial.print(": ");
        Serial.println(receivedFloat);
        receivedThetas[motorIndex] = true; // Mark this theta as received
        return receivedFloat;
      }
    }
    // Prompt again if no input received
    if (Serial.available() == 0) {
      delay(100);  // Wait for a second before prompting again
      Serial.print("Enter theta ");
      Serial.print(motorIndex);
      Serial.println(": ");
    }
  }
}

void loop() {
  if (motors.run() == 0) {
    bool allInputsReceived = true;
    
    for (int i = 0; i < dof; i++) {
      if (!receivedThetas[i]) {
        thetas[i] = readFloatFromSerial(i);
        if (!receivedThetas[i]) {
          allInputsReceived = false;
          break;
        }
      }
    }
    
  
    if (allInputsReceived) {
      motors.moveTo(thetas);
      delay(100);
      // Reset receivedThetas for the next round of inputs
      for (int i = 0; i < dof; i++) {
        receivedThetas[i] = false;
      }
    }
  }
}