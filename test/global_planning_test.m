function [] = global_planning_test
close all;
clear;
%% Initialization
start = [1.5,1.5];    
goal = [8.5,8.5];    
fm = [4 3 3];
cir = [0 0 0];
% obstacle detection range
O_x = 0.1; O_y = 0.1; % size of object
P_x = 0.1; P_y = 0.1; % size of platform
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
save('obs.mat','obstacle','obscell');
load('obs.mat','obstacle','obscell');
% [nodez,nodepoly,edge_zind,noderoute,nodepoly_zind,polyroute,dis_total] = Global_planning_orig(O_x,O_y,P_x,P_y,fm(1),cir(1),range,obstacle,start,goal);
% [nodez,nodepoly,edge_zind,noderoute,nodepoly_zind,polyroute,occupied_area,dis_total] = Global_planning(O_x,O_y,P_x,P_y,fm(2),cir(1),range,obstacle,start,goal);
draw_obs(obscell,range);%draw_conv_region(nodepoly(polyroute),[],range);
drawformation(nodez(noderoute,:),[O_x,O_y],[P_x,P_y],3,0);
hold on
grid = 0.05:0.1:9.95;
for i = 1:100
    hold on;
    plot(ones(1,100)*grid(i),grid,'Color','b');
end
for i = 1:100
    hold on;
    plot(grid,ones(1,100)*grid(i),'Color','b');
end
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'*-','Color',[1,0,1],'linewidth',3);
[new_waypts,polys_x,polys_y,ts,r_set] = minimum_snap_box(nodez(noderoute,:)',nodepoly_zind,noderoute,nodepoly);
[x,y] = draw_smooth(nodez(noderoute,:)',new_waypts,polys_x,polys_y,ts,r_set);
end 