// Code adapted from MultiAxis.ino from StepperDriver library documentation
// by Shom Chandra for BMED 4739/6739 class at Georgia Tech, Fall 2024

#include <Arduino.h>
#include "A4988.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
// This arduino code is meant to take inputs from the serial port and move multiple motors.

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_1_RPM 10
// Target RPM for Y axis motor
#define MOTOR_2_RPM 10

// 1st motor
#define DIR_1 2
#define STEP_1 3

// 2nd motor
#define DIR_2 4
#define STEP_2 5

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepper1(MOTOR_STEPS, DIR_1, STEP_1);
BasicStepperDriver stepper2(MOTOR_STEPS, DIR_2, STEP_2);

// Pick one of the two controllers below
// each motor moves independently
// MultiDriver controller(stepper1, stepper2);
// OR
// synchronized move
SyncDriver controller(stepper1, stepper2);

void setup() {
    /*
     * Set target motors RPM.
     */
    stepper1.begin(MOTOR_1_RPM, MICROSTEPS);
    stepper2.begin(MOTOR_1_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepper1.setEnableActiveState(LOW);
    // stepper2.setEnableActiveState(LOW);
}

void loop() {
  float theta1 = Serial.parseFloat();
  delay(100);
  float theta2 = Serial.parseFloat();
  // For now rotating just two motors, can be expanded upon later
controller.rotate(theta1, theta2);  // note to self: try converting float to long (library may require it)
  // Serial.println(theta2);
  delay(100);
}