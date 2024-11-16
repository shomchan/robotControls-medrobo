%% drawRobot helper function
% used to draw links for visualization of robot

% inputs frames of robot (4x4xN) where N is DoF
% outputs visualization

%% function definition
function h = drawRobot(frames)
    % Draw the robot
    h1 = drawArrows([0;0;0], eye(3), 15e-2, 2); % draw base frame
    h2 = drawArrows(frames(1:3,4,end), frames(1:3,1:3,end), 10e-2, 2); % draw end effector frame
    color = 'rgbmcyk';
    linewidth = [5 4 3 2 1];
    hl = line([0 frames(1,4,1)], [0 frames(2,4,1)], [0 frames(3,4,1)], ...
    'LineWidth', linewidth(1), 'Color', color(1));
    h = [h1; h2; hl];
    for i = 2:4
        hl = line([frames(1,4,i-1) frames(1,4,i)], [frames(2,4,i-1) frames(2,4,i)],[frames(3,4,i-1) frames(3,4,i)], ...
        'LineWidth', linewidth(i), 'Color', color(i));
        h = [h; hl];
    end
    xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
end
