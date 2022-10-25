function Project_Presentations()
clc;
clear
close all
color='mgbkrc'; %%%corresponding to 6 colors
load('smooth.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
nodepoly1 = nodepoly;
load('smooth2.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
nodepoly2 = nodepoly;
load('smooth3.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
nodepoly3 = nodepoly;
load('smtraj.mat','x','y');
x1 = x; y1= y;
load('smtraj2.mat','x','y');
x2= x; y2= y;
load('smtraj3.mat','x','y');
x3 = x; y3= y;
load('task_allocation1', 'obstacle','obscell','init_goal','init_pose','goal_set','IND');
load('task_allocation1_pose', 'pose_x','pose_y','pose_th','v_x','v_y','rep_lx','rep_ly','rep_fx','rep_fy','rep_ind');
load('1.mat','pose_xp2f','pose_yp2f','pose_thp2f','V_x','V_y','dyobs');
pose_x1 = pose_xp2f;
pose_y1 = pose_yp2f;
pose_th1 = pose_thp2f;
vx1 = V_x;
vy1 = V_y;
for i = 1:size(vx1,1)
    for j = 1:size(vx1,2)
        v1(i,j) = vx1(i,j)/cos(pose_th1(i,j));
    end
end
obj1 = [pose_x1(1:4,1) pose_y1(1:4,1)];
load('2.mat','pose_xp2f','pose_yp2f','pose_thp2f','V_x','V_y','dyobs');
pose_x2 = pose_xp2f;
pose_y2 = pose_yp2f;
pose_th2 = pose_thp2f;
vx2 = V_x;
vy2 = V_y;
for i = 1:size(vx2,1)
    for j = 1:size(vx2,2)
        v2(i,j) = vx2(i,j)/cos(pose_th2(i,j));
    end
end
obj2 = [pose_x2(1:3,1) pose_y2(1:3,1)];
load('3.mat','pose_xp2f','pose_yp2f','pose_thp2f','V_x','V_y','dyobs');
pose_x3 = pose_xp2f;
pose_y3 = pose_yp2f;
pose_th3 = pose_thp2f;
vx3 = V_x;
vy3 = V_y;
for i = 1:size(vx3,1)
    for j = 1:size(vx3,2)
        v3(i,j) = vx3(i,j)/cos(pose_th3(i,j));
    end
end
obj3 = [pose_x3(1:3,1) pose_y3(1:3,1)];
% pose_1_x = [pose_x(1,1:340) pose_x1(2,:)];
% pose_2_x_ = pose_x2(1,end)*ones(1,455);
% pose_2_x = [pose_x(2,:) pose_x2(1,:) pose_2_x_];
% pose_3_x_ = pose_x3(3,end)*ones(1,546);
% pose_3_x = [pose_x(3,:) pose_x3(3,:) pose_3_x_];
% pose_4_x_ = pose_x2(2,end)*ones(1,455);
% pose_4_x = [pose_x(4,:) pose_x2(2,:) pose_4_x_];
% pose_5_x_ = pose_x3(2,end)*ones(1,546);
% pose_5_x = [pose_x(5,:) pose_x3(2,:) pose_5_x_];
% pose_6_x_ = pose_x2(3,end)*ones(1,455);
% pose_6_x = [pose_x(6,:) pose_x2(3,:) pose_6_x_];
% pose_7_x = [pose_x(7,1:340) pose_x1(4,:)];
% pose_8_x = [pose_x(8,1:340) pose_x1(1,:)];
% pose_9_x = [pose_x(9,1:340) pose_x1(3,:)];
% pose_10_x_ = pose_x3(1,end)*ones(1,546);
% pose_10_x = [pose_x(10,:) pose_x3(1,:) pose_10_x_];
% pose_1_y = [pose_y(1,1:340) pose_y1(2,:)];
% pose_2_y_ = pose_y2(1,end)*ones(1,455);
% pose_2_y = [pose_y(2,:) pose_y2(1,:) pose_2_y_];
% pose_3_y_ = pose_y3(3,end)*ones(1,546);
% pose_3_y = [pose_y(3,:) pose_y3(3,:) pose_3_y_];
% pose_4_y_ = pose_y2(2,end)*ones(1,455);
% pose_4_y = [pose_y(4,:) pose_y2(2,:) pose_4_y_];
% pose_5_y_ = pose_y3(2,end)*ones(1,546);
% pose_5_y = [pose_y(5,:) pose_y3(2,:) pose_5_y_];
% pose_6_y_ = pose_y2(3,end)*ones(1,455);
% pose_6_y = [pose_y(6,:) pose_y2(3,:) pose_6_y_];
% pose_7_y = [pose_y(7,1:340) pose_y1(4,:)];
% pose_8_y = [pose_y(8,1:340) pose_y1(1,:)];
% pose_9_y = [pose_y(9,1:340) pose_y1(3,:)];
% pose_10_y_ = pose_y(1,end)*ones(1,546);
% pose_10_y = [pose_y(10,:) pose_y3(1,:) pose_10_y_];
% pose_1_th = [pose_th(1,1:340) pose_th1(2,:)];
% pose_2_th_ = pose_th2(1,end)*ones(1,455);
% pose_2_th = [pose_th(2,:) pose_th2(1,:) pose_2_th_];
% pose_3_th_ = pose_th3(3,end)*ones(1,546);
% pose_3_th = [pose_th(3,:) pose_th3(3,:) pose_3_th_];
% pose_4_th_ = pose_th2(2,end)*ones(1,455);
% pose_4_th = [pose_th(4,:) pose_th2(2,:) pose_4_th_];
% pose_5_th_ = pose_th3(2,end)*ones(1,546);
% pose_5_th = [pose_th(5,:) pose_th3(2,:) pose_5_th_];
% pose_6_th_ = pose_th2(3,end)*ones(1,455);
% pose_6_th = [pose_th(6,:) pose_th2(3,:) pose_6_th_];
% pose_7_th = [pose_th(7,1:340) pose_th1(4,:)];
% pose_8_th = [pose_th(8,1:340) pose_th1(1,:)];
% pose_9_th = [pose_th(9,1:340) pose_th1(3,:)];
% pose_10_th_ = pose_th(1,end)*ones(1,546);
% pose_10_th = [pose_th(10,:) pose_th3(1,:) pose_10_th_];
% pose_f_x = [pose_1_x;pose_2_x;pose_3_x;pose_4_x;pose_5_x;pose_6_x;pose_7_x;pose_8_x;pose_9_x;pose_10_x];
% pose_f_y = [pose_1_y;pose_2_y;pose_3_y;pose_4_y;pose_5_y;pose_6_y;pose_7_y;pose_8_y;pose_9_y;pose_10_y];
% pose_f_th = [pose_1_th;pose_2_th;pose_3_th;pose_4_th;pose_5_th;pose_6_th;pose_7_th;pose_8_th;pose_9_th;pose_10_th];
load('demo.mat','pose_f_x','pose_f_y','pose_f_th');
% pose_x1 = pose_x(:,1:343);
% pose_y1 = pose_y(:,1:343);
% pose_th1 = pose_th(:,1:343);
% load('C:\Users\admin\Desktop\final project\src\shortestpath.mat','nodez','nodepoly','edgez','noderoute','polyroute');
load('initialization.mat','obs','best_val','O_x','O_y','P_x','P_y','fm','cir','x_init');
N = 5;
for i = 1:N-1
    for j = 1:size(pose_x1,2)
        centre = cal_centre(pose_x1(1:N-1,j),pose_y1(1:N-1,j),4,0);
        error_p1(i,j) = norm([pose_x1(i,j)-centre(1),pose_y1(i,j)-centre(2)]) -...
            norm([x_init(i,1)-x_init(N,1),x_init(i,2)-x_init(N,2)]);
    end
end
load('initialization2.mat','obs','best_val','O_x','O_y','P_x','P_y','fm','cir','x_init');
N = 4;
for i = 1:N-1
    for j = 1:size(pose_x2,2)
        centre = cal_centre(pose_x2(1:N-1,j),pose_y2(1:N-1,j),4,0);
        error_p2(i,j) = norm([pose_x2(i,j)-centre(1),pose_y2(i,j)-centre(2)]) -...
            norm([x_init(i,1)-x_init(N,1),x_init(i,2)-x_init(N,2)]);
    end
end
load('initialization3.mat','obs','best_val','O_x','O_y','P_x','P_y','fm','cir','x_init');
N = 4;
for i = 1:N-1
    for j = 1:size(pose_x3,2)
        centre = cal_centre(pose_x3(1:N-1,j),pose_y3(1:N-1,j),4,0);
        error_p3(i,j) = norm([pose_x3(i,j)-centre(1),pose_y3(i,j)-centre(2)]) -...
            norm([x_init(i,1)-x_init(N,1),x_init(i,2)-x_init(N,2)]);
    end
end
O_x = 0.2;
O_y = 0.2;
range = [0 0;10 10];
fm = [4 3 3];
cir = [0 0 0];
rob_fmind = [];
% all_x = [];
% all_y = [];
% all_th = [];
for i = 1:length(fm)
    rob_fmind = [rob_fmind,repelem(cir(i),fm(i))];
end
draw_path_ta(pose_f_x,pose_f_y,pose_f_th,obscell,O_x,O_y,P_x,P_y,fm,rob_fmind,0,IND,nodepoly1,nodepoly2,nodepoly3,...
   x1,y1,x2,y2,x3,y3,obj1,obj2,obj3);
% patch('Vertices',[1.8+0.05,4+0.05;1.8+0.05,4-0.05;...
%             1.8-0.05,4-0.05;1.8-0.05,4+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
draw_conv_region(nodepoly1,obscell.vert,range);
hold on;
draw_conv_region(nodepoly2,obscell.vert,range);
hold on;
draw_conv_region(nodepoly3,obscell.vert,range);
hold on;
%draw_path(pose_xi2p,pose_yi2p,pose_thi2p,obs,O_x,O_y,P_x,P_y,fm,cir,0);
hold on;
h1 = plot(x1,y1,'*-','Color','m','linewidth',3);
hold on;
h2 = plot(x2,y2,'*-','Color','g','linewidth',3);
legend('Formation 1','Formation 2','Formation 3');
hold on;
h3 = plot(x3,y3,'*-','Color','b','linewidth',3);
legend([h1,h2,h3],'Formation 1','Formation 2','Formation 3');
title('The Global Trajectory after Optimization');
dt = 0.1;
for i = 1:size(pose_x1,2)
    for j = 1:size(x1,2)
        error(i,j) = norm([pose_x1(5,i)-x1(j),pose_y1(5,i)-y1(j)]);
        error1(i) = min(error(i,:));
    end
end
t_data1 = 0:dt:dt*(size(pose_x1,2)-1);
for i = 1:size(pose_x2,2)
    for j = 1:size(x3,2)
        error(i,j) = norm([pose_x2(4,i)-x3(j),pose_y2(4,i)-y3(j)]);
        error2(i) = min(error(i,:));
    end
end
t_data2 = 0:dt:dt*(size(pose_x2,2)-1);
for i = 1:size(pose_x3,2)
    for j = 1:size(x2,2)
        error(i,j) = norm([pose_x3(4,i)-x2(j),pose_y3(4,i)-y2(j)]);
        error3(i) = min(error(i,:));
    end
end
t_data3 = 0:dt:dt*(size(pose_x3,2)-1);
figure
h1 = plot(t_data1,error1,'Color','m');
hold on;
h2 = plot(t_data2,error2,'Color','g');
hold on;
h3 = plot(t_data3,error3,'Color','b');
legend([h1,h2,h3],'Formation 1','Formation 2','Formation 3');
title('Trajectory Tracking Error');
load('obs.mat','obstacle','obscell');
draw_obs(obscell,range);
patch('Vertices',[1.8+0.05,4+0.05;1.8+0.05,4-0.05;...
            1.8-0.05,4-0.05;1.8-0.05,4+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
patch('Vertices',[5+0.05,3.2+0.05;5+0.05,3.2-0.05;...
            5-0.05,3.2-0.05;5-0.05,3.2+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
% Draw the path record of formation 
for i=1:5
    h(i) = plot(pose_x1(i,:),pose_y1(i,:),color(1,i),'LineWidth',2);
    hold on
end
for i=1:4
    plot(pose_x1(i,1),pose_y1(i,1),'bp','color',color(1,i),'LineWidth',1);
    hold on
end
plot(pose_x1(4,1),pose_y1(4,1),'*','color',color(1,4),'LineWidth',1);
hold on
for i=1:4
    plot(pose_x2(i,:),pose_y2(i,:),color(1,i),'LineWidth',2);
    hold on
end
for i=1:4
    plot(pose_x2(i,1),pose_y2(i,1),'bp','color',color(1,i),'LineWidth',1);
    hold on
end
plot(pose_x2(4,1),pose_y2(4,1),'*','color',color(1,4),'LineWidth',1);
hold on
for i=1:4
    plot(pose_x3(i,:),pose_y3(i,:),color(1,i),'LineWidth',2);
    hold on
end
for i=1:3
    plot(pose_x3(i,1),pose_y3(i,1),'bp','color',color(1,i),'LineWidth',1);
    hold on
end
plot(pose_x3(4,1),pose_y3(4,1),'*','color',color(1,4),'LineWidth',1);
legend(h,'Follower 1','Follower 2','Follower 3','Follower 4','Leader');
figure
for i = 1:4  
    h(1) = plot(t_data1',error_p1(i,:),'color','m');
    hold on
end
for i = 1:3 
    h(2) = plot(t_data2',error_p2(i,:),'color','g');
    hold on
end
for i = 1:3 
    h(3) = plot(t_data3',error_p3(i,:),'color','b');
    hold on
end
xlabel('time/s');ylabel('v/m*s-1');zlabel('v/m*s-1');grid;
title('Formation Position Error');
legend(h,'Formation 1','Formation 2','Formation 3');
figure
color1 = 'mmmmk';
for i = size(pose_x1,1):-1:1  
    h(1) = plot(t_data1',v1(i,:),'color',color1(1,i));
    hold on
end
color2 = 'gggk';
for i = size(pose_x2,1):-1:1   
    h(2) = plot(t_data2',v2(i,:),'color',color2(1,i));
    hold on
end
color3 = 'bbbk';
for i = size(pose_x3,1):-1:1   
    h(3) = plot(t_data3',v3(i,:),'color',color3(1,i));
    hold on
end
title('Formation Velocity Error');
xlabel('time/s');ylabel('v/m*s-1');grid;
legend(h,'Formation 1','Formation 2','Formation 3');
figure
for i = 1:size(pose_x1,1)  
    h(i) = plot3(t_data1',pose_x1(i,:),pose_y1(i,:),'color',color(1,i));
    hold on
end
hold on
for i = 1:size(pose_x2,1)  
    plot3(t_data2',pose_x2(i,:),pose_y2(i,:),'color',color(1,i));
    hold on
end
hold on
for i = 1:size(pose_x3,1)  
    plot3(t_data3',pose_x3(i,:),pose_y3(i,:),'color',color(1,i));
    hold on
end
hold on
plot3(t_data2',1.8*ones(1,length(t_data2)),4*ones(1,length(t_data2)),'color',color(1,5));
plot3(t_data3',5*ones(1,length(t_data3)),3.2*ones(1,length(t_data3)),'color',color(1,5));
title('Formation Trajectories in Time-configuration Space');
xlabel('time/s');ylabel('x/m');zlabel('y/m');grid;
legend(h,'Follower 1','Follower 2','Follower 3','Follower 4','Leader');
% load('motion1.mat','pose_x','pose_y','pose_th','traj');
% all_x = [all_x;pose_x(1:4,:)];
% all_y = [all_y;pose_y(1:4,:)];
% all_th = [all_th;pose_th(1:4,:)];
% load('motion2.mat','pose_x','pose_y','pose_th','traj');
% for i = 537:593
%     pose_x(1:3,i) = pose_x(1:3,536);
%     pose_y(1:3,i) = pose_y(1:3,536);
%     pose_th(1:3,i) = pose_th(1:3,536);
% end
% all_x = [all_x;pose_x(1:3,:)];
% all_y = [all_y;pose_y(1:3,:)];
% all_th = [all_th;pose_th(1:3,:)];
% load('motion3.mat','pose_x','pose_y','pose_th','traj');
% for i = 388:593
%     pose_x(1:3,i) = pose_x(1:3,387);
%     pose_y(1:3,i) = pose_y(1:3,387);
%     pose_th(1:3,i) = pose_th(1:3,387);
% end
% all_x = [all_x;pose_x(1:3,:)];
% all_y = [all_y;pose_y(1:3,:)];
% all_th = [all_th;pose_th(1:3,:)];
%%
% draw_path(all_x,all_y,all_th,obstacle,P_x,P_y,10)
% hold on;
% load('path1.mat','nodez1','nodepoly1','edge_zind1','noderoute1','polyroute1');
% hold on;
% x1=nodez1(noderoute1,1);
% y1=nodez1(noderoute1,2);
% plot(x1,y1,'*-','Color',[1,0.5,0],'linewidth',3);
% load('motion1.mat','pose_x','pose_y','pose_th','traj');
% draw_path(pose_x,pose_y,pose_th,obstacle,O_x,O_y,P_x,P_y,4,0,1,traj);
% formation 1
% centre_1 = cal_centre(goal_set([1 2 5 6],:),4,0);
% Global_planning(O_x,O_y,P_x,P_y,4,cir,range,obstacle,centre_1,[1,1]);
% [nodez1,nodepoly1,edge_zind1,noderoute1,polyroute1] = Global_planning(O_x,O_y,P_x,P_y,4,cir,range,obstacle,centre_1,[1,1]);
% save('path1.mat','nodez1','nodepoly1','edge_zind1','noderoute1','polyroute1');
% formation 2
% centre_2 = cal_centre(goal_set([3 7 9],:),3,0);
% [nodez2,nodepoly2,edge_zind2,noderoute2,polyroute2] = Global_planning(O_x,O_y,P_x,P_y,3,cir,range,obstacle,centre_2,[7.5 9]);
% save('path2.mat','nodez2','nodepoly2','edge_zind2','noderoute2','polyroute2');
% formation 3
% centre_3 = cal_centre(goal_set([4 8 10],:),3,0);
% [nodez3,nodepoly3,edge_zind3,noderoute3,polyroute3] = Global_planning(O_x,O_y,P_x,P_y,3,cir,range,obstacle,centre_3,[4 5]);
% save('path3.mat','nodez3','nodepoly3','edge_zind3','noderoute3','polyroute3');
% load('C:\Users\admin\Desktop\final project\src\path3.mat','nodez3','nodepoly3','edgez3','noderoute3','polyroute3');
% node_route = nodez3(noderoute3,:);
% init_goal = nodez3(1,1:2);
% goalset = node_route(2:size(node_route,1),1:2);
% obstcell = createobstcell(obs);
% x_init_fm = goal_set([4 8 10],:);
% x_init_fm(4,:) = [centre_3 0];
% [pose_x,pose_y,pose_th,traj] = DWA_con(x_init_fm,x_init_fm,obstacle,goalset,3,0.2,0.1);
% hold on;
% x1=nodez3(noderoute3,1);
% y1=nodez3(noderoute3,2);
% plot(x1,y1,'*-','Color',[1,0.5,0],'linewidth',3);
% draw_path(pose_x,pose_y,pose_th,obstacle,O_x,O_y,P_x,P_y,3,0,1,traj);
% save('motion3.mat','pose_x','pose_y','pose_th','traj');
end

% function centre = cal_centre(pose_matrix,fm,cir)
% x_min = min(pose_matrix(1:fm,1));
% x_max = max(pose_matrix(1:fm,1));
% y_min = min(pose_matrix(1:fm,2));
% y_max = max(pose_matrix(1:fm,2));
% if cir == 1 || fm ==4
%     centre = [(x_min+x_max)/2 (y_min+y_max)/2];
% elseif fm == 3
%     centre = [(x_min+x_max)/2 y_min+(y_max-y_min)/3];
% end
% end
function centre = cal_centre(pose_x,pose_y,fm,cir)
x_min = min(pose_x(:,1));
x_max = max(pose_x(:,1));
y_min = min(pose_y(:,1));
y_max = max(pose_y(:,1));
if cir == 1 || fm ==4
    centre = [(x_min+x_max)/2 (y_min+y_max)/2];
elseif fm == 3
    centre = [(x_min+x_max)/2 y_min+tan(1.0060)*(x_max-x_min)/2];
end
end
function draw_path(pose_x,pose_y,pose_th,obs,P_x,P_y,fm)
obstcell = createobstcell(obs);
hold on
node_num = size(pose_th,2);
% plot(pose_x(end,:),pose_y(end,:),'*-','Color',[0,0,1],'linewidth',3);
for j=1:node_num
    for rr = 1:fm
        pose_matrix(rr,1) = pose_x(rr,j);
        pose_matrix(rr,2) = pose_y(rr,j);
        pose_matrix(rr,3) = pose_th(rr,j);
    end
    [h1,h2,h3,h4] = drawformation_control(pose_matrix,[P_x,P_y],fm);
    pause(0.1);
    if j ~= node_num
        delete(h1);
        delete(h2);
        delete(h3);
        delete(h4);
    end
end
end
function [h1,h2,h3,h4] = drawformation_control(pose_matrix,lm,fm)
import rvctools.*
color='mmmmbbbggg';
hold on
xm = pose_matrix;
% build mobile platform cuboid
for i=1:fm
    plat(i) = GeoProperties(xm(i,:),lm);
    plot(pose_matrix(i,1),pose_matrix(i,2),'o','Color',color(i),'LineWidth',0.05);
    h1(i) = patch('Vertices',plat(i).verticesStates.position','Faces',plat(i).faces,'FaceColor','r','FaceAlpha',0.5);
    object_face(i) = i;
end
h2 = patch('Vertices',xm(1:4,:),'Faces',[1 2 4 3],'FaceColor','k','FaceAlpha',0.5);
h3 = patch('Vertices',xm(5:7,:),'Faces',[1 2 3],'FaceColor','k','FaceAlpha',0.5);
h4 = patch('Vertices',xm(8:10,:),'Faces',[1 2 3],'FaceColor','k','FaceAlpha',0.5);  
hold off
end