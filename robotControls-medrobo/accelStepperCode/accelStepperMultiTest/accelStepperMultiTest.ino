#include <AccelStepper.h>
#include <MultiStepper.h>

int xPositions[] = {-100, 200, -500, 0, 500, 0};
int yPositions[] = {-500, 300, -100, 0, 100, 0};
// int zPositions[] = {400, -100, 200, 0, -300, 0};

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(AccelStepper::DRIVER, 5, 4);
AccelStepper stepper2(AccelStepper::DRIVER, 3, 2);
// AccelStepper stepper3(AccelStepper::DRIVER, 7, 6);

long positions[2]; // Array of desired stepper positions
// long positions[3];
int numSteps = 0;

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

void setup()
{
   Serial.begin(9600);

   // Configure each stepper
   stepper1.setMaxSpeed(500);
   stepper2.setMaxSpeed(500);
  //  stepper2.setMaxSpeed(500);

   // Then give them to MultiStepper to manage
   steppers.addStepper(stepper1);
   steppers.addStepper(stepper2);
  //  steppers.addStepper(stepper3);
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
      // positions[2] = zPositions[index];
      steppers.moveTo(positions);
      index++;
      if(index > numSteps)
      {
         index = 0;
      }
   }
}
