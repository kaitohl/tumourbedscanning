
%% Data Loading
function [path, bestDistance] = twoopt(pointsdown)

% % Specify the path to the JSON file
% jsonFile = 'ScannedPoints.mrk.json';
% 
% % Read the JSON content
% fid = fopen(jsonFile); 
% raw = fread(fid, inf); 
% str = char(raw'); 
% fclose(fid);
% 
% % Decode JSON to MATLAB structure
% data = jsondecode(str);
% 
% % Access control points
% controlPoints = data.markups.controlPoints;
% 
% % Extract the positions of the points into an array
% numPoints = length(controlPoints);
% points = zeros(numPoints, 3); % Preallocate for performance
% 
% for i = 1:numPoints
%     points(i, :) = controlPoints(i).position;
% end
% 
% downsample_factor = 2;  % Keep every 10th point
% pointsdown = points(1:downsample_factor:end, :);
% 
% % Display the downsampled point cloud
% figure;
% scatter3(pointsdown(:,1), pointsdown(:,2), pointsdown(:,3), 'filled');
% title('Downsampled Point Cloud (Uniform)');
% xlabel('X'); ylabel('Y'); zlabel('Z');
% axis equal;

numPoints = length(pointsdown);

% Step 1: Compute the Distance Matrix
D = squareform(pdist(pointsdown));  % Euclidean distance matrix in 3D

%% Initial Random Path (Greedy Initialization)
path = [1:numPoints 1];
bestDistance = pathLengthIndices(path, D);

%% 2-opt Algorithm for TSP

improvement = true;  % Track if improvements are made
while improvement
    improvement = false;  % Reset improvement flag

    % Try swapping all pairs of edges
    for i = 2:(numPoints - 1)  % Skip the first and last point to maintain loop
        for j = (i + 1):numPoints
            % Create a new path by swapping edges (i, i+1) and (j, j+1)
            newPath = twoOptSwap(path, i, j);
            newDistance = pathLengthIndices(newPath, D);

            % If the new path is shorter, keep it
            if newDistance < bestDistance
                path = newPath;
                bestDistance = newDistance;
                improvement = true;  % Mark that improvement was made
            end
        end
    end
end

%% Display Results
fprintf('Optimized Path Length: %.2f units\n', bestDistance);

% Plot the optimized path
figure;
hold on;

% Plot all points in blue
scatter3(pointsdown(:, 1), pointsdown(:, 2), pointsdown(:, 3), 'filled', 'MarkerFaceColor', 'b');

% Plot the starting point in green
scatter3(pointsdown(path(1), 1), pointsdown(path(1), 2), pointsdown(path(1), 3), 100, 'g', 'filled');

% Plot the end point in red (if different from start)
scatter3(pointsdown(path(end), 1), pointsdown(path(end), 2), pointsdown(path(end), 3), 100, 'r', 'filled');

% Plot the path edges
for k = 1:numPoints
    plot3(pointsdown(path(k:k+1), 1), pointsdown(path(k:k+1), 2), pointsdown(path(k:k+1), 3), '-r', 'LineWidth', 2);
end

xlabel('X');
ylabel('Y');
zlabel('Z');
% title('Optimized TSP Path using 2-opt');
grid on;
axis equal;
hold off;


%% Helper Functions

% Function to compute the length of a path using indices and distance matrix D
function dist = pathLengthIndices(path, D)
    dist = 0;
    for t = 1:(length(path) - 1)
        dist = dist + D(path(t), path(t + 1));
    end
end


% Function to perform a 2-opt swap between points i and j
function newPath = twoOptSwap(path, i, j)
    % Reverse the section of the path between i and j
    newPath = [path(1:i-1), fliplr(path(i:j)), path(j+1:end)];
end


end