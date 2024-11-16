clear
clc

%% create circle

N = 72; % number of targets
center = [0.26; 0.0; 0.11];
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

tol_p=[0.005; 0.01]; % threshold error of position, brakepoint (m)
tol_xi=[pi/2; pi]; % threshold error of twist, brakepoint (rad)
tols=[tol_p tol_xi];

v=0.1; % speed to increment (m/s)
w=0.01; % speed to increment (rad/s)
maxs=[v w];
dt=0.1; % time to multiply qdot to increment

v2=0.09;
w2=0.001;
mins=[v2 w2];

q_start=[0; 0; 0; 0]; % initializing joint angles
max_iter=4000; % max allowed iterations before giving up

%% run resolved rates for each target position

visualize=true;
fignum=1;
q_storage=zeros(4,N+1);
q_storage(:,1)=q_start;
data=zeros(3,N); % storage matrix for later analysis
% 1st row # of iterations to reach solution, 
% 2nd row errors in position,
% 3rd row errors in angle

for n=1:N
    [q_new, iterations, posErr, angErr]=resolvedRates(q_start,Tt,n,tols,maxs,mins,dt,max_iter,visualize,fignum);
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

%% visualize

visualize = true;
if visualize
    % initialize the figure for animation
    figure(2);
    view(3)
    axis([-1,1,-1,1,-0.5,1.8]);
    axis equal
    grid on; hold on
    title("Visualizing Final Outputs of Resolved Rates")
    fig = gcf;
    axis_handle = fig.CurrentAxes;
    set(gcf,'CurrentAxes',axis_handle);
    set(gcf,'units','normalized','position',[0 0.05 0.5 0.75]);
end
for k=1:N+1
    thisq=q_storage(:,k);
    frames=forwardKinematics(thisq);
    if k==1
        n=k;
    else
        n=k-1;
    end
    if visualize
        cla(axis_handle);
        ht = drawArrows(Tt(1:3,4,n), Tt(1:3,1:3,n), 5e-2, 2); % draw target pose
        h = drawRobot(frames);
        scatter3(squeeze(Tt(1,4,:)),squeeze(Tt(2,4,:)),squeeze(Tt(3,4,:)));
        txt=sprintf("Position %d",k-1);
        text(0.0,0.0,0.25,txt);
        % real-time data display
%         str{1}=datestr(now);
%         str{2}=['Target Pose # ',num2str(k),];
%         str{3}=['Current: ',...
%         ' \theta_0=',num2str(thisq(1)),...
%         ' \theta_1=',num2str(thisq(2))];
%         str{4}=[' \theta_2=',num2str(thisq(3)),...
%         ' \theta_3=',num2str(thisq(4))];
        drawnow; % immediately draw the current configuration
    end
    pause(0.1);
end

%% send commands to arduino

% % takes user input for COM number that arduino is connected to
inputCOM=inputdlg("Please input COM number for serial connection:");
COM=strcat("COM",inputCOM{1});
% %

br=9600; % baud rate, change as necessary

delete(instrfindall); % delete any residual serial connection
arduinoSerial=serialport(COM,br); % creating arduinoSerial object

for i=1:n+1
    thisq=q_storage(:,i)*180/pi;
    fprintf("Joint values for position %0.1f:\n",i)
    disp(thisq);
    thisq=thisq';
    thisq=num2cell(thisq);
    sendCommands_multimotor(thisq,arduinoSerial);
    pause(1);
end