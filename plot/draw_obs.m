%% Draw the obstacles
function draw_obs(obstcell,range)
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
figure
pad = (ub - lb) * 0.05;
obs_num = numel(obstcell.vert);
if isempty(obstcell.vert)
    obs_num = 0;
end
% draw obstacle interiors
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
hold on;
% draw obstacle boundaries 
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
xlim([lb(1)-pad(1),ub(1)+pad(1)]);
ylim([lb(2)-pad(2),ub(2)+pad(2)]);
end

