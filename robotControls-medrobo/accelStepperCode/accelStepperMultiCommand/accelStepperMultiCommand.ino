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

// Creates an instance of each motor
AccelStepper motor1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper motor3(AccelStepper::DRIVER, stepPin3, dirPin3);

MultiStepper motors;

const int dof = 2; // CHANGE VALUE to number of motors
long thetas[dof]; // initializing theta array

void setup() {
  Serial.begin(9600);

	// set the maximum speed, acceleration factor,
	// initial speed, and set up multistepper
	motor1.setMaxSpeed(1000);
	motor1.setAcceleration(50);
	motor1.setSpeed(200);
  motor2.setMaxSpeed(1000);
	motor2.setAcceleration(50);
	motor2.setSpeed(200);

  motors.addStepper(motor1);
  motors.addStepper(motor2);
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

void loop() {
  // thetas[0] = Serial.parseFloat();
  // Serial.print(thetas[0]);
  // delay(100);
  // Serial.print(", ");
  // thetas[1] = Serial.parseFloat();
  // Serial.println(thetas[1]);
  
  for (int i = 0; i < dof; i++) {
    Serial.print("Enter theta ");
    Serial.print(i + 1);
    Serial.print(": ");
    thetas[i] = readIntFromSerial();
  }

  motors.moveTo(thetas);
  motors.runSpeedToPosition();
  delay(1000);
}