%% Generate the tasks
function [init_goal,init_pose,goal_set,IND] = generate_task(O_x,O_y,fm,cir,range)
goalset = [];
init_pose = [];
for i = 1:length(fm)
    init_goal_cen(i,:) = [rand*(range(2,1)-range(1,1)),rand*(range(2,2)-range(1,2))];
    [~,~,init_goal] = c2v(init_goal_cen(i,:),[O_x,O_y],fm(i),cir(i));
    goalset = [goalset;init_goal];
end
goalset = [1.35 1.5 3.1416;
    1.5 1.35 -1.5708;
    1.65 1.5 0;
    1.5 1.65 1.5708;
    7.5 1.1167 1.5708;
    7.4 0.8711 1.0472;
    7.6 0.8711 -1.0472;
    1 7.1167 1.5708;
    0.9 6.8711 1.0472;
    1.1 6.8711 -1.0472];
goalset(:,3) = zeros(10,1);
for j = 1:size(goalset,1)
    init_pose = [init_pose;[rand*(range(2,1)-range(1,1)),rand*(range(2,2)-range(1,2)),0]];
end
init_pose = [1 1 0;1 10 0;9 1 0;2.7 5.2 0;6 3.2 0;
             7 9 0;3.5 4 0;1 4 0;3 1.5 0;9 7 0];
for i = 1:1:size(goalset,1)
    for j = 1:size(init_pose,1)
        cost(j,i) = norm([init_pose(j,1)-goalset(i,1),init_pose(j,2)-goalset(i,2)]);
    end
end
fm_ind = [];
for i = 1:length(fm)
    fm_ind = [fm_ind;repmat(i,fm(i),1)];
end
goalset(1:sum(fm),4) = fm_ind(1:sum(fm),1);
[goal_set,IND]  = TaskAllocation(goalset,cost);
end
