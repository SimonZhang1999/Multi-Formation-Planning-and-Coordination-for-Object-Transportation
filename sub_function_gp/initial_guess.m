function [cell_x,cell_y,cell_dis] = initial_guess(obstacle,obscell,range)
ob_temp_ = [];
ob_temp = [];
dis = [];
cell_x_1 = range(1,1) + (range(2,1)-range(1,1))/200:(range(2,1)-range(1,1))/100:range(2,1)-(range(2,1)-range(1,1))/200;
cell_y_1 = range(2,1)-(range(2,1)-range(1,1))/200:-(range(2,1)-range(1,1))/100:range(1,1) + (range(2,1)-range(1,1))/200;
for i = 1:100
    for j = 1:100
        cell_x(i,:) = cell_x_1;
        cell_y(:,i) = cell_y_1';
    end
end
for obn = 1:length(obstacle)
    ob_temp_ = obstacle(1,obn).verticesStates.position;
    ob_temp_ = ob_temp_';
    hull = convhull(ob_temp_(:,1),ob_temp_(:,2));
    for j = 1:4
        ob_tem(:,1) = (ob_temp_(hull(j,1),1):(ob_temp_(hull(j+1,1),1)-ob_temp_(hull(j,1),1))/10:ob_temp_(hull(j+1,1),1))';
        ob_tem(:,2) = interp1([ob_temp_(hull(j,1),1);ob_temp_(hull(j+1,1),1)],[ob_temp_(hull(j,1),2);ob_temp_(hull(j+1,1),2)],(ob_temp_(hull(j,1),1):(ob_temp_(hull(j+1,1),1)-ob_temp_(hull(j,1),1))/10:ob_temp_(hull(j+1,1),1))');
        % slope((j-1)*10+obn,1) = atan((ob_tem(end,2)-ob_tem(1,2))/(ob_tem(end,1)-ob_tem(1,1)));
        ob_temp = [ob_temp;ob_tem];
    end
end
obs_ind(1,1) = 1;
obs_ind(1,2) = 11;
for i = 2:obn*4
    obs_ind(i,1) = obs_ind(i-1,2) + 1;
    obs_ind(i,2) = obs_ind(i,1) + 10;
end
[M,~]=size(ob_temp);
for i = 1:100
    for j = 1:100
        for k = 1:M
            dis(k) = norm([cell_x(i,j)-ob_temp(k,1) cell_y(i,j)-ob_temp(k,2)]);
        end
        [min_dis,ind] = min(dis);
        cell_dis(i,j) = min_dis;
        dis = [];
    end
end
for i = 1:100
    for j = 1:100
        flag = in_obstacle(obscell,[cell_x(i,j),cell_y(i,j)]);
        if ~flag
            cell_dis(i,j) = -inf;
        end
    end        
end
end
%% Check if the cell is inside the obstacles
function flag = in_obstacle(obscell,cell_vertice)
flag = true;
for obs_num = 1:length(obscell.calc)
    if inpolygon(cell_vertice(1),cell_vertice(2),...
            obscell.calc{1,obs_num}(1,:),obscell.calc{1,obs_num}(2,:))
        flag = false;
        break;
    end
end
end