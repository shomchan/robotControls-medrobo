delete(instrfindall); % delete any residual serial connection
clear
clc

%% create circle

N = 72; % number of targets
center = [0.26; 0.0; 0.1];
R = [0 0 1;
     1 0 0;
     0 1 0]; % orientation of the circle
r = 0.04; % radius
Tt = zeros(4,4,N);
for i = 1:N
    alpha = i/N*2*pi;
    ca = cos(alpha); sa = sin(alpha);
    Tt(1,4,i) = R(1,1)*r*ca + R(1,2)*r*sa + center(1);
    Tt(2,4,i) = R(2,1)*r*ca + R(2,2)*r*sa + center(2);
    Tt(3,4,i) = R(3,1)*r*ca + R(3,2)*r*sa + center(3);
    Tt(1:3,1:3,i) = R*roty(-90);
end

%% define parameters for resolved rates

% target position and orientation homogeneous matrix ([R,d;0,1])
% xc_target=  [1 0 0  0.2
%              0 0 -1  0.05
%              0 1 0  0.3
%              0 0 0  1   ];

tol_p=0.01; % threshold error of position (m)
tol_xi=pi/2; % threshold error of twist (rad)

v=0.1; % speed to increment (m/s)
w=0.01; % speed to increment (rad/s)
dt=0.1; % time to multiply qdot to increment

q_start=[0; 0; 0; 0]; % initializing joint angles
max_iter=4000; % max allowed iterations before giving up

%% run resolved rates for each target position

q_storage=zeros(4,N+1);
q_storage(:,1)=q_start;
data=zeros(3,N); % storage matrix for later analysis
% 1st row # of iterations to reach solution, 
% 2nd row errors in position,
% 3rd row errors in angle
for n=1:N
    xc_target=Tt(:,:,n);
    [q_new, iterations, posErr, angErr]=resolvedRates(q_start,xc_target,tol_p,tol_xi,v,w,dt,max_iter);
    q_storage(:,n+1)=q_new;
    q_start=q_new;
    data(1,n)=iterations;
    data(2,n)=posErr;
    data(3,n)=angErr;
end
mask=(data(1,:)==max_iter+1);
percent_timedout=round(100*sum(mask)/N,2);
fprintf("Percent timed out: %f\n",percent_timedout);
fprintf("Average position error: %f m\n",mean(data(2,:)));
fprintf("Average angle error: %f rad\n",mean(data(3,:)));

%% send commands to arduino

% % takes user input for COM number that arduino is connected to
inputCOM=inputdlg("Please input COM number for serial connection:");
COM=strcat("COM",inputCOM{1});
% %

br=9600; % baud rate, change as necessary

arduinoSerial=serialport(COM,br); % creating arduinoSerial object

for i=1:n+1
    thisq=q_storage(:,i);
    thisq=num2cell(thisq);
    sendCommands_multimotor(thisq,arduinoSerial);
    pause(0.8);
end

%% sub-functions for visualization
function h = drawRobot(frames)
    % Draw the robot
    h1 = drawArrows([0;0;0], eye(3), 15e-2, 2); % draw base frame
    h2 = drawArrows(frames(1:3,4,end), frames(1:3,1:3,end), 15e-2, 2); % draw end effector frame
    color = 'rgbmcyk';
    linewidth = [5 4 3 2 1];
    hl = line([0 frames(1,4,1)], [0 frames(2,4,1)], [0 frames(3,4,1)], ...
    'LineWidth', linewidth(1), 'Color', color(1));
    h = [h1; h2; hl];
    for i = 2:5
        hl = line([frames(1,4,i-1) frames(1,4,i)], [frames(2,4,i-1) frames(2,4,i)],[frames(3,4,i-1) frames(3,4,i)], ...
        'LineWidth', linewidth(i), 'Color', color(i));
        h = [h; hl];
    end
    xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
end

function arrows = drawArrows(p,R,length,width)
    % Draw Cartesian coordinates
    % length - length of the axis
    % width - line width of the axis
    R=length.*R; %introduce length to the orthogonal vectors
    hold on
    arrow_x = quiver3(p(1),p(2),p(3),R(1,1),R(2,1),R(3,1),'r','filled','LineWidth',width); % xaxis
    arrow_y = quiver3(p(1),p(2),p(3),R(1,2),R(2,2),R(3,2),'g','filled','LineWidth',width); % yaxis
    arrow_z =quiver3(p(1),p(2),p(3),R(1,3),R(2,3),R(3,3),'b','filled','LineWidth',width); % zaxis
    arrows = [arrow_x; arrow_y; arrow_z];
end