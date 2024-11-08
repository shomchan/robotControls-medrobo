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

// Creates an instance of each motor
AccelStepper motor1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor2(AccelStepper::DRIVER, stepPin2, dirPin2);
// AccelStepper motor3(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper motor4(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);

MultiStepper motors;

const int dof = 3; // CHANGE VALUE to number of motors
long thetas[dof]; // initializing theta array

void setup() {
  Serial.begin(9600);

	// set the maximum speed, acceleration factor,
	// initial speed, and set up multistepper
	motor1.setMaxSpeed(500);
  motor2.setMaxSpeed(500);
	// motor3.setMaxSpeed(500);
  motor4.setMaxSpeed(500);
  
  motors.addStepper(motor1);
  motors.addStepper(motor2);
  // motors.addStepper(motor3);
  motors.addStepper(motor4);
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
  if (motors.run() == 0) {
    for (int i = 0; i < dof; i++) {
      Serial.print("Enter theta ");
      Serial.print(i);
      Serial.print(": ");
      thetas[i] = readIntFromSerial();
    }

    motors.moveTo(thetas);
    delay(1000);
  }
}