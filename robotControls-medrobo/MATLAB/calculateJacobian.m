%% calculateJacobian Helper Function
% inputs 4x4xN array of frames where N is DoF
% outputs 6xN matrix Jacobian where N is DoF

%% function definition
function j_out = calculateJacobian(input)
    % check for errors in input and display debug messages
    if size(input,1)~=4
        error("Invalid size of input array, must be 4x4xN")
    elseif size(input,2)~=4
        error("Invalid size of input array, must be 4x4xN")
    end
    
    dof=size(input,3);
    e=input(1:3,4,end);
    
    % initializing j matrices:
    j_out=zeros(6,dof);
    j_v=zeros(3,dof); % linear portion of jacobian
    j_w=zeros(3,dof); % angular portion of jacobian
    
    % o_0 is constant [0,0,0]' b/c base frame doesn't move
    o_=[0;0;0]; 
    % z_0 is constant [0,0,1]' b/c base frame doesn't move 
    z_=[0;0;1];

    % iterate through each column of j
    for k=1:dof
        j_w(:,k)=z_;
        diff=e-o_;
        j_v(:,k)=cross(z_,diff);
        % update z_ and o_
        z_=input(1:3,3,k);
        o_=input(1:3,4,k);
    end

    % output final jacobian
    j_out=[j_v; j_w];
end