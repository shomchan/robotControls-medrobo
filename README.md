AccelStepperCode folder:
-
Folder contains code based on arduino accelstepper library for testing of individual motors, motor movement in tandem, as well as code for reading serial commands as angles.

accelStepperMultiCommand - send multiple angles to arduino for multiple motors

accelStepperSingleCommand - send 1 angle to arduino for motor angle

accelStepperSingleTest - test function of 1 motor

accelStepperMultiTest - test function of multiple motors in tandem

MATLAB Code for sending commands:
-
createControls.m - function that creates trajectory and calls all necessary helper functions to create controls, visualize, and send angles for robot movement along trajectory. 

inputCommands_multimotor.m - MATLAB function for manual input of angles to Arduino (useful for testing); utilizes sendCommands_multimotor.m helper function

sendCommands_multimotor.m - MATLAB helper function that sends serial messages to Arduino sequentially

Unused/Legacy Code:
-
StepperDriverCode_OLD folder contains old code based on laurb9/StepperDriver library for arduino (unused).

arduinoStepperControl - Arduino sketch for simultaneous control of 2 motors (can be expanded to 4)

updated_StepperDriver_library - updated .cpp and .h files for StepperDriver library to be used with up to 4 motors instead of 3
