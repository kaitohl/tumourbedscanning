
function [order, pathlength] = raster(pointsdown)

% distances = sqrt(sum(diff(pointsdown).^2, 2));
% 
% idx = find(distances > 5);
order = pointsdown;
% order(1:idx(1),:) = pointsdown(1:idx(1),:);
% 
% for i = 1:length(idx)-1
%     if mod(i,2) == 0
%         order(idx(i)+1:idx(i+1),:) = pointsdown(idx(i)+1:idx(i+1),:);
%     else
%         order(idx(i)+1:idx(i+1),:) = flip(pointsdown(idx(i)+1:idx(i+1),:));
%     end
% end
% 
% order(length(pointsdown),:) = pointsdown(end,:);

% Plot the path using the order in the `points` array
figure;
hold on;

% Plot the points as a scatter plot
scatter3(order(:, 1), order(:, 2), order(:, 3), 'filled', 'MarkerFaceColor', 'b');

% Plot the path by connecting each point in sequence
plot3(order(:, 1), order(:, 2), order(:, 3), '-r', 'LineWidth', 2);

% Mark the starting point in green and the end point in red
scatter3(order(1, 1), order(1, 2), order(1, 3), 100, 'g', 'filled');  % Starting point
scatter3(order(end, 1), order(end, 2), order(end, 3), 100, 'r', 'filled');  % End point

% Add labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
% title('Path Traversed in Order of Points');
grid on;
axis equal;
hold off;

pathlength = pathLengthCoords(order);
fprintf('Raster Path Length: %.2f units\n', pathlength);

% Function to compute the length of a path using 3D coordinates
    function dist = pathLengthCoords(path)
        dist = 0;
        for t = 1:(size(path, 1) - 1)
            dist = dist + sqrt(sum((path(t, :) - path(t + 1, :)).^2));
        end
    end

end