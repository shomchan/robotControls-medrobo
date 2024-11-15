%% Send Commands (Multimotor) Helper Function
% inputs cell array of messages & arduinoSerial object
% outputs to arduino

% params setup: cell array contains angles in degrees in order of motor

%% function definition
function [] = sendCommands_multimotor(msg,ard)
    for i=1:size(msg,2) % iterates through each cell in msg
        writeline(ard,sprintf('%f',msg{i})); % prints as text to arduino
        pause(0.05) % pauses 50ms, change as necessary
    end
end