%% resolvedRates helper function
% runs resolved rates algorithm to determine movement

%% inputs:
% input joint angles (4x1) (rad)
% input target position and pose homogeneous matrix (4x4xN) (R,p;0,1)
% input # of target n
% input tolerances for position and twist
% input speeds for changing position and twist
% input dt
% max iterations allowed
% visualization boolean

% % e.g.
% q_in=[0; 0; 0; 0]; % starting joint angles
% 
% % target position and orientation homogeneous matrix ([R,d;0,1])
% xc_target(:,:,j)=   [1 0 0  0.2
%                      0 0 -1  0.05
%                      0 1 0  0.3
%                      0 0 0  1   ];
% 
% tols= [tol_p tol_xi]
% tol_p=0.01; % threshold error of position (m)
% tol_xi=pi/8; % threshold error of twist (rad)
% 
% maxs= [v w]
% v=0.1; % speed to increment (m/s)
% w=0.001; % speed to increment (rad/s)
% dt=0.1; % time to multiply qdot to increment
%
% viz = true

%% outputs:
% ending joint angles

%% function definition
function [out, counter, p_err, ang_err] = resolvedRates(q_in,xc_target,n,tols,maxs,mins,dt,max_iter,viz,fignum)
    if viz
        % initialize the figure for animation
        figure(fignum);
        view(3)
        axis([-1,1,-1,1,-0.5,1.8]);
        axis equal
        grid on; hold on
        title("Visualizing Progress of Resolved Rates")
        fig = gcf;
        axis_handle = fig.CurrentAxes;
        set(gcf,'CurrentAxes',axis_handle);
        set(gcf,'units','normalized','position',[0 0.05 0.5 0.75]);
        % draw circle
        scatter3(squeeze(xc_target(1,4,:)),squeeze(xc_target(2,4,:)),squeeze(xc_target(3,4,:)));
    end
    tol_p=tols(1,1);
    tol_xi=tols(1,2);
    brake_p=tols(2,1);
    brake_xi=tols(2,2);
    q=q_in; % initialize q
    p_target=xc_target(1:3,4,n); % extracts translation
    target_orientation=xc_target(1:3,1:3,n); % extracts rotation
    
    singval_threshold = 1e-3; % threshold of triggering singularity robust pseudo-inverse 
    lambda = 1e-3; % singular value damping coefficient for singularity robustness
    
    frames=forwardKinematics(q);
    p=frames(1:3,4,end); % starting x, y, and z of end effector
    end_orientation=frames(1:3,1:3,end); % rotation matrix of end effector orientation
    
    % calculate deviation from target
    p_diff=p_target-p;
    p_err=norm(p_diff);
    xi_diff=target_orientation*end_orientation';
    xi_err=rotm2axang(xi_diff);
    ax_err=xi_err(1:3)';
    ang_err=xi_err(4);
    
    counter=1; % initialize counter
    
    while p_err>tol_p || ang_err>tol_xi % iterate until under thresholds
        if viz && (mod(counter,5) == 1)
            axis_handle = fig.CurrentAxes;
            cla(axis_handle)
            ht = drawArrows(xc_target(1:3,4,n), xc_target(1:3,1:3,n), 5e-2, 2); % draw target pose
            h = drawRobot(frames);
            scatter3(squeeze(xc_target(1,4,:)),squeeze(xc_target(2,4,:)),squeeze(xc_target(3,4,:)));
            txt{1}=sprintf("Target #%d",n);
            txt{2}=sprintf("Iteration #%d",counter);
            txt{3}=sprintf("Position Error: %0.2d m", p_err);
            txt{4}=sprintf("Angle Error: %0.2d rad", ang_err);
            text(0.0,0.0,0.25,txt); % display info
            jointVals{1}="Joint Angles:";
            jointVals{2}=sprintf("θ_0: %0.2f",q(1));
            jointVals{3}=sprintf("θ_1: %0.2f",q(2));
            jointVals{4}=sprintf("θ_2: %0.2f",q(3));
            jointVals{5}=sprintf("θ_3: %0.2f",q(4));
            text(0.2,-0.2,0.025,jointVals);
            drawnow
        end
        if counter>max_iter % break if past max_iter
            fprintf("Max iterations reached; giving up to save resources." + ...
                "\nCurrent position error: %d m" + ...
                "\nCurrent angle error: %d rad\n\n",p_err,ang_err);
            break
        end
        if p_err<brake_p % if under brake point, slow down
            v=mins(1);
        else
            v=maxs(1);
        end
        if ang_err<brake_xi % if under brake point, slow down
            w=mins(2);
        else
            w=maxs(2);
        end
        % find xcdot
        pdot=v*p_diff/norm(p_diff);
        xidot=w*ax_err; % already normalized
        xcdot=[pdot; xidot];
        
        % find pseudo inverse of jacobian and calculate qdot
        J=calculateJacobian(frames);
        
        % detect singularity
        min_singval_J = min(svd(J));
        if min_singval_J < singval_threshold % when singular
            pinvJ = J'/(J*J' + lambda*eye(size(J,1))); % singularity robust pseudo-inverse
        else
            pinvJ=pinv(J);
        end

        qdot=pinvJ*xcdot; % this is the jacobian equation x_dot=J*q_dot
        
        q=q+qdot*dt; % step q
        
        % recalculate forward kinematics after stepping q
        frames=forwardKinematics(q);
        p=frames(1:3,4,end); % starting x, y, and z of end effector
        end_orientation=frames(1:3,1:3,end); % rotation matrix of end effector orientation
        
        % calculate deviation from target
        p_diff=p_target-p;
        p_err=abs(norm(p_diff));
        xi_diff=target_orientation*end_orientation';
        xi_err=rotm2axang(xi_diff);
        ax_err=xi_err(1:3)';
        ang_err=abs(xi_err(4));

        counter=counter+1; % increment
        % debug statements
%         fprintf("This is iteration #%d.\n",counter)
%         disp("  p_err:     ang_err:")
%         disp([p_err ang_err])
%         disp("ax_err:")
%         disp(ax_err)
%         disp("q:")
%         disp(q)
    end
%     fprintf("Solution reached on iteration #%d.\n",counter)
    out=q;
end