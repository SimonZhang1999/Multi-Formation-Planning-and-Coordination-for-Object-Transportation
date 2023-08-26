function obstacle = createobstcell(obstcub)
n_obs=length(obstcub);
for i=1:n_obs
    obstacle_pts(:,:,i) = obstcub(i).verticesStates.position(1:2,1:4);
end
% reset obstacles vertices in its convexhull (iris requires obstacles to be convex)
obsidx = cell(1,size(obstacle_pts,3));
obstacle.vert = cell(1,size(obstacle_pts,3));
for j = 1:n_obs
    obs = obstacle_pts(:,:,j);
    if size(obs, 2) > 1
        if size(obs, 2) > 2
            % calculate the counterclockwise index of the convex region
            obsidx{j} = convhull(obs(1,:), obs(2,:));
        else
            obsidx{j} = [1,2,1];
        end
    end
    obstacle.vert{j} = obs(:,obsidx{j})';
    obstacle.calc{j} = obs(:,obsidx{j});
end
% obstacles cell
obstacle.poly = obstacle.vert{1}';
for i=2:size(obstacle.vert,1)
    obstacle.poly = [obstacle.poly,NaN(2,1),obstacle.vert{i}'];
end
end

