%% drawArrows helper function
% used to draw arrows for visualization of frames

% inputs point at where axis is, rotation matrix, 
% length and width for visualization

% outputs xyz arrows

%% function definition
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