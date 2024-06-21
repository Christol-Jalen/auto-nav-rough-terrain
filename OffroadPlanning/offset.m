% This code is to apply the map offset to the planned path, after running
% planning.mlx

% If the planner type is plannerAStarGrid, convert the start and goal locations to grid coordinates.
if strcmp(plannerType,"plannerAStarGrid")
    for i = 1:length(paths)
        paths{i} = grid2world(costMap, paths{i}); % Convert each path to world coordinates
    end
end

% clear previous data if there is
clear paths_offset

paths_offset = cell(4,1);
for i = 1:length(paths)
    paths_offset{i}(:, 1) = paths{i}(:, 1)*step_size + x_offset; % Apply x offset
    paths_offset{i}(:, 2) = paths{i}(:, 2)*step_size + y_offset; % Apply y offset
end

save('paths_saved.mat', 'paths_offset');
