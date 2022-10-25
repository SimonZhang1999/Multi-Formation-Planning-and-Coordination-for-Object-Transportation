function [pose_x,pose_y,pose_th] = Local_Planning(~)
clc
clear all
close all
load('initialization3.mat','obs','best_val','O_x','O_y','P_x','P_y','fm','cir','x_init');
load('smooth_demo.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
load('obs.mat','obstacle','obscell');
% goal=[12 12];   % Destination point
range = [0 0;10 10];
node_route = nodez(noderoute,:);
init_goal = nodez(1,1:2);
goal_set = node_route(2:size(node_route,1),1:2);
obstcell = createobstcell(obstacle);
x_init_fm = x_init;
%[pose_xp2f,pose_yp2f,pose_thp2f,V_x,V_y,dyobs] = pursuit_con(x_init,x_init_fm,obs,goal_set,3,0.2,0.001);
% obs_num = size(dyobs,1);
% dyobs(obs_num+1,:) = dyobs(1,:);
% dyobs(obs_num+2,:) = dyobs(1,:);
load('2.mat','pose_xp2f','pose_yp2f','pose_thp2f','V_x','V_y','dyobs');
for i = 1:size(V_x,1)
    for j = 1:size(V_x,2)
        V(i,j) = V_x(i,j)/cos(pose_thp2f(i,j));
    end
end
load('smtraj3.mat','x','y');
% [pose_xi2p,pose_yi2p,pose_thi2p] = apf(pose_i2p,x_init_fm,obs,[1.5 1.5],4,0.2,0.1);
% [pose_xi2p,pose_yi2p,pose_thi2p] = motion_control(pose_i2p,x_init_fm,obs,init_goal,fm,0.2,0.1);
% [pose_xp2f,pose_yp2f,pose_thp2f,traj,dynamic_obs] = DWA_con(x_init,x_init_fm,obs,goal_set,fm,0.2,0.1);
% dynamic_obs(length(dynamic_obs)+1,:) = dynamic_obs(length(dynamic_obs),:);
% save('dwa.mat','pose_xi2p','pose_yi2p','pose_thi2p','pose_xp2f','pose_yp2f','pose_thp2f','traj','dynamic_obs');
%load('dwa.mat','pose_xi2p','pose_yi2p','pose_thi2p','pose_xp2f','pose_yp2f','pose_thp2f','traj','dynamic_obs');
figure
%title('The Global Trajectory after Optimization');
% patch('Vertices',[1.8+0.05,4+0.05;1.8+0.05,4-0.05;...
%             1.8-0.05,4-0.05;1.8-0.05,4+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
grid = 0.05:0.1:9.95;
for i = 1:100
    hold on;
    plot(ones(1,100)*grid(i),grid,'Color','b');
end
for i = 1:100
    hold on;
    plot(grid,ones(1,100)*grid(i),'Color','b');
end
draw_conv_region(nodepoly,obstcell.vert,range);
%draw_path(pose_xi2p,pose_yi2p,pose_thi2p,obs,O_x,O_y,P_x,P_y,fm,cir,0);
hold on;
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'*-','Color','m','linewidth',3);
plot(x,y,'*-','Color','k','linewidth',3);
% draw_path(pose_xp2f,pose_yp2f,pose_thp2f,obs,O_x,O_y,P_x,P_y,fm,cir,1,traj);
% draw_diag(pose_xi2p,pose_yi2p,pose_xp2f,pose_yp2f,obstcell,fm);
% draw_path(pose_xp2f,pose_yp2f,pose_thp2f,obs,O_x,O_y,P_x,P_y,fm,cir,1,dyobs);
% draw_diag(pose_xi2p,pose_yi2p,pose_xp2f,pose_yp2f,obstcell,fm);
draw_diag(pose_xp2f,pose_yp2f,V,obstcell,fm,x_init,cir);
end