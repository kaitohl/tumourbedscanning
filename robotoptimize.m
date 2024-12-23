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


%% Graph Initialization

% Initialize
numSolutionsPerTarget = 4; % 1 initial guess + 3 perturbed configurations
numTargets = length(pathopt); % Total targets in pathopt (180)

% Start and end targets are the same, so 178 unique IK layers
numIKLayers = numTargets - 2; % 178 IK layers
numNodesPerLayer = numSolutionsPerTarget;

% Store IK solutions
qinitguess = cell(numTargets+1, 1);
qsolutions = cell(numTargets, 1);

% Initialize graph variables
totalLayers = numTargets; % 180 layers including start and finish
totalNodes = totalLayers * numNodesPerLayer;

%Start Node
qinitguess{1} = homeConfiguration(robot); % First initial guess

%% IK
firstGuess = homeConfiguration(robot);

% Iterate over intermediate targets
for i = 1:numTargets % Exclude start (1) and finish (end)
    % Extract target point
    target = pointsdownnew(pathopt(i), :);
    desiredDistance = 0.15; % Desired offset
    targetPosition = target + [0, 0, desiredDistance];

    % Define GIK solver
    gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', {'position', 'joint', 'orientation'});

    % Define position constraint
    positionConstraint = constraintPositionTarget('joint6_flange'); % Adjust end-effector link
    positionConstraint.TargetPosition = targetPosition;
    positionConstraint.PositionTolerance = 1e-3;

    % Define joint bounds
    jointBounds = constraintJointBounds(robot);
    for j = 1:length(jointLimits)
        jointBounds.Bounds(j, :) = jointLimits(j, :);
    end

    % Define orientation constraint
    orientationConstraint = constraintOrientationTarget('joint6_flange');
    targetRotation = axang2rotm([1, 0, 0, pi]); % End-effector Z-axis down
    targetQuaternion = rotm2quat(targetRotation);
    orientationConstraint.TargetOrientation = targetQuaternion;
    orientationConstraint.OrientationTolerance = deg2rad(5);

    % Solve IK for each initial guess
    solength = size(qinitguess{i},1);
    for s = 1:solength
        [qSol, solutionInfo] = gik(qinitguess{i}(s,:), positionConstraint, jointBounds, orientationConstraint);
        if strcmp(solutionInfo.Status, 'success')
            qsolutions{i}(s, :) = qSol;
            if solength == 1
                qinitguess{2}(1, :) = qSol;
                for s = 2:numNodesPerLayer
                    qinitguess{2}(s, :) = qSol + 0.15 * randn(size(qSol));
                end
            else
                qinitguess{i+1}(s,:) = qSol;
            end
        else
            qsolutions(s, :) = NaN(size(qSol)); % Mark invalid solutions
        end
        fprintf("Finished Layer %d\n", i);
    end

end

%% Contruct Graph

% Joint velocity limits
maxJointSpeed = deg2rad(80); % Maximum joint speed in radians/sec
jointVelocityLimits = repmat(maxJointSpeed, 1, size(qsolutions{1}, 2));

% Initialize sparse cost matrix
edgeCosts = sparse(totalNodes, totalNodes); % Preallocate cost matrix

% % Node indexing function
% nodeIndex = @(node) ...
%     (node == 1) * 1 + ... % First layer has only one node (index = 1)
%     (node > 1) * 1;

% Populate edge costs between layers
for layer = 1:(numTargets - 1)
    qCurrentLayer = qsolutions{layer};     % Solutions for current layer
    qNextLayer = qsolutions{layer + 1};   % Solutions for next layer
    numNodesCurrentLayer = size(qCurrentLayer, 1); % Number of nodes in the current layer

    for i = 1:numNodesCurrentLayer
        if layer == 1
            idxCurrent = 1; % Only one node in the first layer
        else
            idxCurrent = (layer - 2) * numNodesPerLayer + i + 1; % Calculate idxCurrent
        end

        for j = 1:numNodesPerLayer
            idxNext = (layer - 1) * numNodesPerLayer + j + 1; % Calculate idxNext
            
            % Retrieve joint configurations
            if layer == 1
                qCurrent = qCurrentLayer; % Single node in the first layer
            else
                qCurrent = qCurrentLayer(i, :);
            end
            qNext = qNextLayer(j, :);

            % Calculate normalized maximum joint difference
            deltaQ = qNext - qCurrent; % Joint space difference
            normalizedDeltaQ = abs(deltaQ ./ jointVelocityLimits); % Normalize
            maxDiff = max(normalizedDeltaQ); % Maximum normalized difference

            % Add edge cost
            edgeCosts(idxCurrent, idxNext) = maxDiff;
        end
    end
end

% Create the graph
G = digraph(edgeCosts);

% Define start and end nodes
startNode = 1; % Only one node in the first layer
endNodes = (2 + (numTargets - 2) * numNodesPerLayer):(1 + (numTargets - 1) * numNodesPerLayer);

% Find shortest paths to all end nodes
paths = zeros(length(endNodes), numTargets); % Preallocate for paths
totalCosts = zeros(length(endNodes), 1); % Preallocate for costs

for i = 1:length(endNodes)
    [paths(i, :), totalCosts(i)] = shortestpath(G, startNode, endNodes(i));
end

% Find the optimal end node
[~, minIdx] = min(totalCosts);
optimalEndNode = endNodes(minIdx);
optimalPath = paths(minIdx, :);

% Extract optimal joint configurations
qOptimal = zeros(numTargets, size(qsolutions{2}, 2)); % Preallocate for all layers
qOptimal(1, :) = qsolutions{1}(1, :); % First configuration (single node in layer 1)

for k = 2:length(optimalPath) % Start from the second node in the path
    node = optimalPath(k);

    % Determine the layer (accounting for the single-node first layer)
    if node == 1
        layer = 1; % Special case for the first node
    else
        layer = floor((node - 2) / numNodesPerLayer) + 2; % Adjust for single-node first layer
    end

    % Determine the index within the layer
    idx = mod(node - 2, numNodesPerLayer) + 1;

    % Assign the corresponding joint configuration
    qOptimal(layer, :) = qsolutions{layer}(idx, :);
end

fprintf("Total Cost: %d/n",min(totalCosts))

%% Animate
% Extract joint configurations at waypoints
numWaypoints = length(pathopt); % Number of waypoints
qWaypoints = qOptimal; 

% Interpolate Waypoints using pchip (Piecewise Cubic Hermite Interpolation)
tFinal = 30; % Total time for the trajectory (in seconds)
framerate = 15; % Frames per second
tWaypoints = linspace(0, tFinal, numWaypoints); % Time for waypoints
numFrames = tFinal * framerate; % Total number of frames

% Interpolate joint configurations
qInterp = pchip(tWaypoints, qWaypoints', linspace(0, tFinal, numFrames))';

% Preallocate gripper position array
gripperPosition = zeros(numFrames, 3);

% Compute gripper position for each interpolated configuration
for k = 1:numFrames
    % Get the transform for the end-effector link
    T = getTransform(robot, qInterp(k, :), 'joint6_flange'); % Replace 'joint6_flange' with your end-effector link name
    gripperPosition(k, :) = tform2trvec(T); % Extract translation (position)
end

% Setup VideoWriter
videoFileName = 'robot_trajectory_animation_2optOPT.mp4'; % Name of the video file
v = VideoWriter(videoFileName, 'MPEG-4'); % Create VideoWriter object
v.FrameRate = 60; % Set frame rate
v.Quality = 100;
open(v); % Open the video file for writing

% Create the figure
figure;
set(gcf, 'Position', [100, 100, 2000, 1080]);

% Plot the robot's initial configuration
show(robot, qWaypoints(1, :), 'PreservePlot', false);
hold on;

% Plot the downsampled and transformed point cloud
scatter3(pointsdownnew(:, 1), pointsdownnew(:, 2), pointsdownnew(:, 3), 6, 'filled'); % Point cloud
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;

% Display the total cost value
totalCostStr = sprintf('Total Cost: %.2f (Max Joint Difference)\nTotal Cost: 39.03 (Weighted Euclidean Distance)', min(totalCosts));
textX = max(pointsdownnew(:, 1)) + 0.1; % X position for text
textY = max(pointsdownnew(:, 2)) + 0.87; % Y position for text
textZ = max(pointsdownnew(:, 3)) + 0.1; % Z position for text
text(textX, textY, textZ, totalCostStr, 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
title("2-opt Scan WITH optimization")

% Animate the robot along the interpolated trajectory
for k = 1:size(qInterp, 1)
    % Update robot configuration
    show(robot, qInterp(k, :), 'PreservePlot', false);
    
    % Update the gripper trajectory path
    if k > 1
        p.XData = gripperPosition(1:k, 1); % Update X trajectory
        p.YData = gripperPosition(1:k, 2); % Update Y trajectory
        p.ZData = gripperPosition(1:k, 3); % Update Z trajectory
    end

    % Capture the current figure as a frame for the video
    frame = getframe(gcf);
    writeVideo(v, frame); % Write the frame to the video

    % Pause to control animation speed (adjust as needed)
    pause(0.03); 
end

% Close the video file
close(v);

disp('Trajectory animation complete. Video saved.');
