function [] = motion_planning_ta_test
close all;
clear;
%% Constent
start = [1.5,1.5];    
goal = [8.5,8.5];    
fm = [4 3 3];
cir = [0 0 0];
% obstacle detection range
detect_R = 0.2;
O_x = 0.1; O_y = 0.1; % size of object
P_x = 0.1; P_y = 0.1; % size of platform
% range,lower bound&upper bound of map
range = [0 0;10 10];
% obstacles
o_num = 10;
o_size = 1;
IND = [];
rob_fmind = [];
for i = 1:length(fm)
    rob_fmind = [rob_fmind,repelem(cir(i),fm(i))];
end
[obstacle,obscell] = generate_obstacle(o_num,o_size,range);
% save('obs.mat','obstacle','obscell');
% load('obs.mat','obstacle','obscell');
draw_obs(obscell,range);
% draw_conv_region(nodepoly(polyroute),[],range);
% drawformation(nodez(noderoute,:),[O_x,O_y],[P_x,P_y],3,0);
hold on
% grid = 0.05:0.1:9.95;
% for i = 1:100
%     hold on;
%     plot(ones(1,100)*grid(i),grid,'Color','b');
% end
% for i = 1:100
%     hold on;
%     plot(grid,ones(1,100)*grid(i),'Color','b');
% end
%% Generate start position and goal position
% [init_goal,init_pose,goal_set,IND] = generate_task(O_x,O_y,fm,cir,range);
load('task_allocation1.mat','obstacle','obscell','init_goal','init_pose','goal_set','IND');
test_obs = [init_pose; goal_set];
for pose_num = 1:length(test_obs)
    for obs_num = 1:length(obscell.calc)
        if inpolygon(test_obs(pose_num,1),test_obs(pose_num,2),obscell.calc{1,obs_num}(1,:),obscell.calc{1,obs_num}(2,:))
            error('Invalid initial pose or goal point!');
        end
    end
end
% load('task_allocation', 'obstacle','obscell','init_goal','init_pose','goal_set','IND');
% load('task_allocation_pose', 'pose_x','pose_y','pose_th','v_x','v_y','rep_lx','rep_ly','rep_fx','rep_fy','rep_ind');
[pose_x,pose_y,pose_th,v_x,v_y,rep_lx,rep_ly,rep_fx,rep_fy,rep_ind] = apf_con(init_pose,goal_set,obstacle,goal_set(end,:),size(goal_set,1),detect_R,detect_R);
% load('task_allocation1_pose.mat', 'pose_x','pose_y','pose_th','v_x','v_y','rep_lx','rep_ly','rep_fx','rep_fy','rep_ind');
if length(pose_x) == 502
    error('Task allocation failure!')
else
    disp(mean(abs(pose_x(:,end)-goal_set(:,1))));
    disp(mean(abs(pose_y(:,end)-goal_set(:,2))));
end
rob_col = [];
color='mgbk';
for i = 1:length(fm)
    rob_col = [rob_col,repelem(i,fm(i))];
end
hold on
for i=1:sum(fm)
    plot(1:length(pose_x),(abs(pose_x(i,:)-goal_set(i,1))+abs(pose_y(i,:)-goal_set(i,2)))/2,'Color',color(rob_col(IND(i))));
end
legend('Formation 1','Formation 2','Formation 3');
xlabel('time');
ylabel('error');
title('Formation position error');
draw_obs(obscell,range);
hold on;
draw_path_ta(pose_x,pose_y,pose_th,obscell,O_x,O_y,P_x,P_y,fm,rob_fmind,0,IND);
end
function draw_path_ta(pose_x,pose_y,pose_th,obscell,O_x,O_y,P_x,P_y,fm,cir,arrival,IND)
node_num = size(pose_th,2);
range = [0 0;10 10];
% plot(pose_x(end,:),pose_y(end,:),'*-','Color',[0,0,1],'linewidth',3);
draw_obs(obscell,range);
hold on;
hold on;
for j=1:node_num
    %draw_obs(obscell,range);
    hold on
    for rr = 1:size(pose_x,1)
        pose_matrix(rr,1) = pose_x(rr,j);
        pose_matrix(rr,2) = pose_y(rr,j);
        pose_matrix(rr,3) = pose_th(rr,j);
    end
    if arrival == 1
        [h1,h2] = drawformation_control_ta(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir(rr),arrival,IND);
        pause(0.1);
        if j ~= node_num
            delete(h1);
            delete(h2);
        end
    else
        h1 = drawformation_control_ta(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir(rr),arrival,IND);
        pause(0.1);
        %if j ~= node_num
           delete(h1);
        %end
    end
end
end
