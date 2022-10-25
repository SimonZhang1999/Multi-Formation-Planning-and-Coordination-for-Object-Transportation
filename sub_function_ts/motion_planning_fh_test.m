function [] = motion_planning_fh_test
clc
clear all
close all
load('initialization.mat','obs','best_val','O_x','O_y','P_x','P_y','fm','cir','x_init');
load('smooth.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
load('obs.mat','obstacle','obscell');
range = [0 0;10 10];
node_route = nodez(noderoute,:);
init_goal = nodez(1,1:2);
goal_set = node_route(2:size(node_route,1),1:2);
obstcell = createobstcell(obstacle);
x_init_fm = x_init;
[pose_xp2f,pose_yp2f,pose_thp2f,V_x,V_y,dyobs] = pursuit_con(x_init,x_init_fm,obs,goal_set,4,0.2);
%load('1.mat','pose_xp2f','pose_yp2f','pose_thp2f','V_x','V_y');
for i = 1:size(V_x,1)
    for j = 1:size(V_x,2)
        V(i,j) = V_x(i,j)/cos(pose_thp2f(i,j));
    end
end
% figure
% grid = 0.05:0.1:9.95;
% for i = 1:100
%     hold on;
%     plot(ones(1,100)*grid(i),grid,'Color','b');
% end
% for i = 1:100
%     hold on;
%     plot(grid,ones(1,100)*grid(i),'Color','b');
% end
load('smtraj.mat','x','y');
draw_conv_region(nodepoly,obstcell.vert,range);
hold on;
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'*-','Color','m','linewidth',3);
plot(x,y,'*-','Color','k','linewidth',3);
% draw_diag(pose_xi2p,pose_yi2p,pose_xp2f,pose_yp2f,obstcell,fm);
draw_path(pose_xp2f,pose_yp2f,pose_thp2f,obs,O_x,O_y,P_x,P_y,fm,cir,1);
% draw_diag(pose_xi2p,pose_yi2p,pose_xp2f,pose_yp2f,obstcell,fm);
draw_diag(pose_xp2f,pose_yp2f,V,obstcell,fm,x_init,cir);
end
function draw_path(pose_x,pose_y,pose_th,obs,O_x,O_y,P_x,P_y,fm,cir,arrival,~)
obstcell = createobstcell(obs);
hold on
node_num = size(pose_th,2);
% plot(pose_x(end,:),pose_y(end,:),'*-','Color',[0,0,1],'linewidth',3);
for j=1:node_num
    for rr = 1:fm+1
        pose_matrix(rr,1) = pose_x(rr,j);
        pose_matrix(rr,2) = pose_y(rr,j);
        pose_matrix(rr,3) = pose_th(rr,j);
    end
    if arrival == 1
        [h1,h2] = drawformation_control(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir,arrival);
        pause(0.1);
        delete(h1);
        delete(h2);
    else
        h1 = drawformation_control(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir,arrival);
        pause(0.1);
        delete(h1);
    end
end
end