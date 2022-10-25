function Computing_LargeConvexRegions_test
clc
close all
clear all
tic
% obstacles
range = [0 0;12 12];
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
x = 6;
y = 4.5;
o_num = 40;
o_size = 1;
[obs,obstcell] = generate_obstacle(o_num,o_size,range);
% obstacles cell
%obstcell = generate_obstacle(obs);
figure
hold on
obs_num = numel(obstcell.vert);
if isempty(obstcell.vert)
    obs_num = 0;
end
% Draw obstacle interiors
for j = 1:obs_num
    obst = obstcell.vert{j}';
    vertice_num = size(obst, 2);
    if vertice_num > 1
        if vertice_num > 2
            k = convhull(obst(1,:), obst(2,:));
        else
            k = [1,2,1];
        end
    patch(obst(1,k), obst(2,k), 'k', 'FaceColor', [0.5 0.5 0.5], 'LineWidth', 0.1);
    else
        plot(obst(1,:), obst(2,:), 'ko');
    end
end
% Draw obstacle boundaries 
for j = 1:obs_num
    obst = obstcell.vert{j}';
    vertice_num = size(obst, 2);
    if vertice_num > 1
        if vertice_num > 2
          k = convhull(obst(1,:), obst(2,:));
        else
          k = [1,2,1];
        end
        plot(obst(1,k), obst(2,k), 'k', 'LineWidth', 2);
    end
end
A_boundary = [-1,0;0,-1;1,0;0,1];
b_boundary = [-lb; ub];
[A,b,C,d] = Computing_LargeConvexRegions(obstcell.calc, A_boundary, b_boundary, [x;y]);
hold on
plot(x,y, '*r');
toc
end
%% Generate obstacles
% function obscell = generate_obstacle(obstacle)
% obs_num=length(obstacle);
% obs_vert_point = [];
% for i=1:obs_num
%     obs_vert_point(1:2,1:length(obstacle(1,i).verticesStates.position),i) = obstacle(i).verticesStates.position(1:2,1:length(obstacle(1,i).verticesStates.position));
% end
% % reset obstacles vertices in its convexhull 
% obs_vert_ind = cell(1,size(obs_vert_point,3));
% obscell.vert = cell(1,size(obs_vert_point,3));
% for j = 1:obs_num
%     obs = obs_vert_point(:,:,j);
%     if size(obs, 2) > 1
%         if size(obs, 2) > 2
%             % return the clockwise index of the convexhull
%             obs_vert_ind{j} = convhull(obs(1,:), obs(2,:));
%         else
%             obs_vert_ind{j} = [1,2,1];
%         end
%     end
%     obscell.vert{j} = obs(:,obs_vert_ind{j})';
%     obscell.calc{j} = obs(:,obs_vert_ind{j});
% end
% % obstacles cell
% obscell.poly = obscell.vert{1}';
% for i=2:size(obscell.vert,1)
%     obscell.poly = [obscell.poly,NaN(2,1),obscell.vert{i}'];
% end
% end