%% MATLAB code outputs angle command via serial to Arduino device

close all
clear
clc

% % takes user input for COM number that arduino is connected to
inputCOM=inputdlg("Please input COM number for serial connection:");
COM=strcat("COM",inputCOM{1});
% %

br=9600; % baud rate, change as necessary

arduinoSerial=serialport(COM,br); % creating arduinoSerial object

active=true;

while active
    mode=input("Input 0 to exit, 1 for angle control:");
    switch mode
        case 0
            active=false;
            clear arduinoSerial
        case 1
            theta1=input("Please input angle for motor 1 in degrees:");
            theta2=input("Please input angle for motor 2 in degrees:");
            msgs={theta1 theta2};
            sendCommand_multimotor(msgs,arduinoSerial);
        otherwise
            disp('Invalid Input!')
    end
end
