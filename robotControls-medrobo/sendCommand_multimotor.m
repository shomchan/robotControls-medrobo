%% Send Commands (Multimotor) Helper Function
% inputs cell array of messages & arduinoSerial object
% outputs transformation matrix 

% params setup: cell array contains angles in degrees in order of motor

%% function definition
function [] = sendCommand_multimotor(msg,ard)
    for i=1:size(msg,2) % iterates through each cell in msg
        writeline(ard,sprintf('%f',msg{i})); % prints as text to arduino
        pause(0.2) % pauses 200ms, change as necessary
    end
end