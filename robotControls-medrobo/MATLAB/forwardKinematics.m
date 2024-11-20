%% forwardKinematics Helper Function

% inputs 4x1 vector of joint angles (radians)
% outputs 4x4xN array of end positions (m, rad)
% ([R,p;0,1], third dimension stores each frame

%% function definition
function out = forwardKinematics(input)
    % specific to our robot:
    input=input+[0; pi/2; 0; -pi/2];
    dhParams=   [0 0.14 0 .125           % a in DH convention
                 pi/2 pi/2 -pi/2 -pi/2  % alpha in DH convention
                 0.12 0 .07 0           % d in DH convention
                 ];
    dhParams=dhParams';
    rows=size(input,1);
    
    % initialize storage matrices
    out=zeros(4,4,rows);
    A_=eye(4);
    % loop through rows
    for i=1:rows
        a=dhParams(i,1);
        al=dhParams(i,2);
        d=dhParams(i,3);
        th=input(i);
        A=      [cos(th) -sin(th)*cos(al) sin(th)*sin(al)  a*cos(th)
                sin(th) cos(th)*cos(al)  -cos(th)*sin(al) a*sin(th)
                0       sin(al)          cos(al)          d
                0       0                0                1         ];
        A_=A_*A;
        out(:,:,i)=A_;
    end
end