%% MATLAB code outputs angle command via serial to Arduino device
delete(instrfindall);%delete any residual serial connection
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
            theta3=input("Please input angle for motor 3 in degrees:");
            theta4=input("Please input angle for motor 4 in degrees:");

            msgs={theta1 theta2 theta3 theta4};
            sendCommands_multimotor(msgs,arduinoSerial);
        otherwise
            disp('Invalid Input!')
    end
end

