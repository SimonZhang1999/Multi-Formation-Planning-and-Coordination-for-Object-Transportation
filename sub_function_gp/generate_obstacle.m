%% Generate obstacles
function [obstacle,obscell] = generate_obstacle(o_num,o_size,range)
for i = 1:o_num
    o = [rand()*(range(2,1)-2),rand()*(range(2,2)-2)];
    obstacle(i)= GeoProperties(o,rand()*o_size);
end
obs_num=length(obstacle);
obs_vert_point = [];
for i=1:obs_num
    obs_vert_point(1:2,1:length(obstacle(1,i).verticesStates.position),i) = obstacle(i).verticesStates.position(1:2,1:length(obstacle(1,i).verticesStates.position));
end
% reset obstacles vertices in its convexhull 
obs_vert_ind = cell(1,size(obs_vert_point,3));
obscell.vert = cell(1,size(obs_vert_point,3));
for j = 1:obs_num
    obs = obs_vert_point(:,:,j);
    if size(obs, 2) > 1
        if size(obs, 2) > 2
            % return the clockwise index of the convexhull
            obs_vert_ind{j} = convhull(obs(1,:), obs(2,:));
        else
            obs_vert_ind{j} = [1,2,1];
        end
    end
    obscell.vert{j} = obs(:,obs_vert_ind{j})';
    obscell.calc{j} = obs(:,obs_vert_ind{j});
end
% obstacles cell
obscell.poly = obscell.vert{1}';
for i=2:size(obscell.vert,1)
    obscell.poly = [obscell.poly,NaN(2,1),obscell.vert{i}'];
end
end

