clc; clear; close all;

% Specify the path to the JSON file
jsonFile = 'ScannedPoints-3.mrk.json';

% Read the JSON content
fid = fopen(jsonFile);
raw = fread(fid, inf);
str = char(raw');
fclose(fid);

% Decode JSON to MATLAB structure
data = jsondecode(str);

% Access control points
controlPoints = data.markups.controlPoints;

% Extract the positions of the points into an array
numPoints = length(controlPoints);
points = zeros(numPoints, 3);

for i = 1:numPoints
    points(i, :) = controlPoints(i).position;
end

downsample_factor = 2;  % Keep every 10th point
pointsdown = points(1:downsample_factor:end, :);

% Display the downsampled point cloud
figure;
scatter3(pointsdown(:,1), pointsdown(:,2), pointsdown(:,3), 'filled');
% title('Downsampled Point Cloud (Uniform)');
xlabel('X'); ylabel('Y'); zlabel('Z');

axis equal;

figure;
scatter3(points(:,1), points(:,2), points(:,3), 'filled');
title('Point Cloud (Uniform)');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;

% [pathras,pathlengthras] = raster(pointsdown);
% [pathopt,pathlengthopt] = twoopt(pointsdown);

[pathras,pathlengthras] = raster(pointsdown(1:end-2,:));
[pathopt,pathlengthopt] = twoopt(pointsdown(1:end-2,:));

%% Robot Visualization and Kinematics

robot = importrobot("./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf");
robot.DataFormat = 'row';
showdetails(robot);

config = homeConfiguration(robot);

% DH Table: [alpha, a, d, theta_offset]
DH = [
    0,     0,    131.56,  0;         % Joint 1
    pi/2,  0,    0,       -pi/2;     % Joint 2
    0,    -110.4, 0,       0;         % Joint 3
    0,    -96,   64.62,   -pi/2;     % Joint 4
    pi/2,  0,    73.18,    pi/2;     % Joint 5
    -pi/2,  0,    48.6,     0;        % Joint 6
    ];


% Joint Limits in Radians
jointLimits = deg2rad([
    -165, 165;  % J1
    -165, 165;  % J2
    -165, 165;  % J3
    -165, 165;  % J4
    -165, 165;  % J5
    -175, 175   % J6
    ]);

%%

% Find the index of the point with the lowest z-coordinate
[~, minIdx] = min(pointsdown(:, 3));

% Extract the lowest point's coordinates
lowestPoint = pointsdown(minIdx, :);

% Define the target position for the lowest point
targetPosition = [0, 0.2, 0];

% Calculate the translation vector to move the lowest point to the target position
translationVector = targetPosition - lowestPoint;

% Translate the entire point cloud to align the lowest point with the target position
transformedPointCloudData = pointsdown + translationVector;

% Define the scaling factor (e.g., 0.5 for half-size)
scalingFactor = 0.003;

% Scale the point cloud relative to the target position
% 1. Translate point cloud to origin (relative to target position)
relativePointCloudData = transformedPointCloudData - targetPosition;

% 2. Apply the scaling factor
pointsdownnew = relativePointCloudData * scalingFactor;

% 3. Translate back to the target position to maintain alignment
pointsdownnew = pointsdownnew + targetPosition;

figure;

% Plot the robot model in the specified axes
show(robot, config, PreservePlot=false);
% title('Robot with Transformed and Scaled Point Cloud');
hold on

% Plot the transformed and scaled point cloud in the same axes
scatter3( pointsdownnew(:, 1), pointsdownnew(:, 2), pointsdownnew(:, 3), 6,'filled');

% Label axes for clarity
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');



%% IK

initialGuess = homeConfiguration(robot);

% Interpolate Joint Configurations
framerate = 15;
r = rateControl(framerate);
tFinal = 30; % Total time for the trajectory
tWaypoints = linspace(0, tFinal, length(pointsdownnew)+1); % Time for waypoints
numFrames = tFinal * framerate; % Total frames for smooth animation
numWaypoints = length(pointsdownnew);
% Initialize joint configurations for waypoints
qWaypoints = repmat(initialGuess, numWaypoints+1, 1);

for i = 1:length(pointsdownnew)
    % Extract target point from path
    target = pointsdownnew(i,:);
    desiredDistance = 0.15;
    targetPosition = target + [0, 0, desiredDistance];
    targetPose = trvec2tform(targetPosition);

    % Define the GIK solver
    gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', {'position', 'joint','orientation'});

    % Define the target position for the end-effector
    positionConstraint = constraintPositionTarget('joint6_flange');  % Adjust end-effector link name as needed
    positionConstraint.TargetPosition = targetPosition;
    positionConstraint.PositionTolerance = 1e-3;  % Position tolerance in meters

    % Define joint bounds constraints
    jointBounds = constraintJointBounds(robot);
    for j = 1:length(jointLimits)
        jointBounds.Bounds(j, :) = jointLimits(j, :);
    end

    % Define an orientation constraint to make the end-effector Z-axis point down
    % Assuming the target orientation is for the end-effector Z-axis to align with global -Z
    orientationConstraint = constraintOrientationTarget('joint6_flange');  % Use actual end-effector link name
    targetRotation = axang2rotm([1 0 0 pi]);  % Rotation matrix to point Z-axis downward
    targetQuaternion = rotm2quat(targetRotation);  % Convert to quaternion
    orientationConstraint.TargetOrientation = targetQuaternion;
    orientationConstraint.OrientationTolerance = deg2rad(5);

    % Solve for the target position with joint bounds
    [qWaypoints(i+1,:), solutionInfo] = gik(initialGuess, positionConstraint, jointBounds,orientationConstraint);

    if strcmp(solutionInfo.Status, 'success')
        
        disp('GIK solution found and respects joint limits.');
        % initialGuess = qWaypoints(i+1,:);

    else
        disp('GIK solution not found or joint constraints are not feasible.');
    end

end

% Interpolate configurations using pchip for smooth trajectory
qInterp = pchip(tWaypoints, qWaypoints', linspace(0, tFinal, numFrames))';

%% Compute Gripper Position for Each Interpolated Configuration
gripperPosition = zeros(numFrames, 3);

% Convert interpolated joint configurations to configuration structures
for k = 1:numFrames
    % Compute the gripper position using the configuration structure
    gripperPosition(k, :) = tform2trvec(getTransform(robot, qInterp(k,:), 'joint6_flange')); % Update with the correct end-effector link name
end

%% Weighted Euclidian Distance
jointMaxRanges = abs(jointLimits(:, 2) - jointLimits(:, 1));
maxLinkLengths = [0.13156, 0.632, 0.6005, 0.2013, 0.1025, 0.094];
weights = maxLinkLengths.*jointMaxRanges'; 

% Initialize total cost
totalCost = 0;

% Loop through all interpolated configurations
for i = 1:(size(qInterp, 1) - 1)
    % Difference between consecutive configurations
    deltaQ = qInterp(i+1, :) - qInterp(i, :);
    
    % Weighted Euclidean distance (scalar for each transition)
    weightedDistance = sqrt(sum((weights .* deltaQ).^2));
    
    % Accumulate the total cost
    totalCost = totalCost + weightedDistance;
end

% Display the total cost
disp(['Total Weighted Euclidean Joint Distance: ', num2str(totalCost)]);

%% Maximum Joint Difference

maxJointSpeed = deg2rad(80);
jointVelocityLimits = repmat(maxJointSpeed, 1, size(qInterp, 2));

% Initialize maximum joint difference cost
maxJointDifference = zeros(size(qInterp, 1) - 1, 1); % Preallocate

% Loop through all interpolated configurations
for i = 1:(size(qInterp, 1) - 1)
    % Difference between consecutive configurations
    deltaQ = qInterp(i+1, :) - qInterp(i, :);
    
    % Normalize by joint velocity limits
    normalizedDeltaQ = abs(deltaQ ./ jointVelocityLimits);
    
    % Maximum joint difference for this transition
    maxJointDifference(i) = max(normalizedDeltaQ);
end

% Total cost: Sum or analyze individual transitions
totalMaxJointDifference = sum(maxJointDifference);

% Display results
disp(['Total Maximum Joint Difference Cost: ', num2str(totalMaxJointDifference)]);

%% Visualization and Animation
% Setup VideoWriter
videoFileName = 'robot_trajectory_animation_ras.mp4'; % Name of the video file
v = VideoWriter(videoFileName, 'MPEG-4'); % Create VideoWriter object
v.FrameRate = 60; % Set frame rate
v.Quality = 100;
open(v); % Open the video file for writing

figure;
set(gcf, 'Position', [100, 100, 2000, 1080]);
show(robot, qWaypoints(1, :), 'PreservePlot', false);
hold on;
scatter3(pointsdownnew(:, 1), pointsdownnew(:, 2), pointsdownnew(:, 3), 6, 'filled'); % Point cloud
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;

% Display the total cost value
totalCostStr = sprintf('Total Cost: 21.39 (Max Joint Difference)\nTotal Cost: 61.69 (Weighted Euclidean Distance)');
textX = max(pointsdownnew(:, 1)) + 0.1; % X position for text
textY = max(pointsdownnew(:, 2)) + 0.87; % Y position for text
textZ = max(pointsdownnew(:, 3)) + 0.1; % Z position for text
text(textX, textY, textZ, totalCostStr, 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
title("Raster Scan NO optimization")

% Plot the gripper trajectory path
% p = plot3(gripperPosition(:, 1), gripperPosition(:, 2), gripperPosition(:, 3), 'r', 'LineWidth', 2);

% Animate the robot along the interpolated trajectory
for k = 1:size(qInterp, 1)
    show(robot, qInterp(k, :), 'PreservePlot', false); % Update robot configuration
    p.XData(k) = gripperPosition(k, 1);
    p.YData(k) = gripperPosition(k, 2);
    p.ZData(k) = gripperPosition(k, 3);
    waitfor(r);
    % Capture the current figure as a frame for the video
    frame = getframe(gcf);
    writeVideo(v, frame); % Write the frame to the video
    pause(0.03); % Adjust to control animation speed
end
% Close the video file
close(v);
disp('Trajectory animation complete. Video saved.');
disp('Trajectory animation complete.');
%%
function T = dh_transform(alpha, a, d, theta)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),            cos(alpha),            d;
    0,           0,                     0,                     1];
end