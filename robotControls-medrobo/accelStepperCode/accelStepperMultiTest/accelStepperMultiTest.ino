#include <AccelStepper.h>
#include <MultiStepper.h>

int xPositions[] = {-10, 20, -50, 0, 50, 0};
int yPositions[] = {-50, 30, -10, 0, 10, 0};
int kPositions[] = {40, -10, 20, 0, -30, 0};
int zPositions[] = {400, -100, 200, 0, -300, 0};


#define motorPin1  8     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
#define motorPin2  9     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
#define motorPin3  10    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
#define motorPin4  11    // IN4 on ULN2003 ==> Orange on 28BYJ-48

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(AccelStepper::DRIVER, 5, 4);
AccelStepper stepper2(AccelStepper::DRIVER, 3, 2);
AccelStepper stepperk(AccelStepper::DRIVER, 7, 6);
AccelStepper stepper3(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);

long positions[4]; // Array of desired stepper positions
// long positions[2];
int numSteps = 0;

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

void setup()
{
   Serial.begin(9600);

   // Configure each stepper
   stepper1.setMaxSpeed(500);
   stepper2.setMaxSpeed(500);
   stepper3.setMaxSpeed(500);
   stepperk.setMaxSpeed(500);

   // Then give them to MultiStepper to manage
   steppers.addStepper(stepper1);
   steppers.addStepper(stepper2);
   steppers.addStepper(stepperk);
   steppers.addStepper(stepper3);
   numSteps = sizeof(xPositions) / sizeof(xPositions[0]);
}

void loop()
{
   static int index = 0;
   if (steppers.run() == 0)
   {
      Serial.print("run  ");
      Serial.println(index);
      positions[0] = xPositions[index];
      positions[1] = yPositions[index];
      positions[2] = kPositions[index];
      positions[3] = zPositions[index];
      steppers.moveTo(positions);
      index++;
      if(index > numSteps)
      {
         index = 0;
      }
   }
}
