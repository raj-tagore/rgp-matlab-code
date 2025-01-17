%%
% Notes: Gripper straight, (gripper pointing down and camera image
% straight) is oritentation (pi pi 0). to turn 90 degrees left, the
% orientation would be (-pi/2 pi 0) and to turn 90 degrees right the
% orientation would be (pi/2 pi 0). 
% Right rotation is +ve

% input here is (x, y, z, rotz, roty, rotx, gripwidth)
% x is right, y is straight and z is up.
gripperPoseAndGripWidth = [
    -0.35 -0.007 0.23 -pi/2 -pi 0 0.228;    % can
    -0.53 -0.05 0.08 0 -pi 0 0.228;         % bottle
    -0.5 0.39 0.15 0 -pi 0 0.228;           % can
    0.66 0.025 0.25 0 -pi 0 0.228;          % can
    0.66 0.025 0.15 0 -pi 0 0.228;          % can
    0.46 -0.07 0.24 0 -pi 0 0.517           % bottle
    ];
bins = ["green" "blue" "green" "green" "green" "blue"];

for i=1:length(bins)
    pickDrop(gripperPoseAndGripWidth(i,:),bins(i));
end