// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 2;
const int stepPin = 3;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

long theta = 0; // initializing theta


void setup() {
	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(1000);
	myStepper.setAcceleration(50);
	myStepper.setSpeed(200);
  Serial.begin(9600);
}

int readIntFromSerial() {
  while (true) {  // Keep checking until get a valid integer
    if (Serial.available() > 0) {
      int receivedInt = Serial.parseInt();
      // Check if has actually received a complete number
      if (Serial.read() == '\n') {
        Serial.print("Received integer: ");
        Serial.println(receivedInt);
        return receivedInt;  // Return received int
      }
    }
    // Small delay to prevent excessive usage
    delay(10);
  }
}

void runMotor() {
    while (myStepper.distanceToGo() != 0) {
    Serial.println(myStepper.distanceToGo());
    myStepper.run();
  }
}

void loop() {
  Serial.println("Waiting for next integer...");
  theta = readIntFromSerial();
  myStepper.moveTo(theta);

  runMotor();

  // if (theta > 0) {
  //   while (myStepper.distanceToGo() > 0) {
  //     myStepper.run();
  //   }
  // }
  // else if (theta < 0) {
  //     while (myStepper.distanceToGo() < 0) {
  //       myStepper.run();
  //   }
  // }
}