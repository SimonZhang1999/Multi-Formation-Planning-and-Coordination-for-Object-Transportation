clc
close all
clear all
load('smtraj.mat','x','y');
load('smooth.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
load('obs.mat','obstacle','obscell');
% range,lower bound&upper bound of map
range = [0 0;10 10];
fm = 4;
cir = 0; 
% drawplat3d(nodez,[lcx,lcy,lcz],[lmx,lmy,lmz],fm,cir)
% zarray轨迹的经过点，dzarray为每个点的速度，tarray为时间片信息
%k = 0.1;  % look forward gain前视距离系数
Lfc = 0.1;  % look-ahead distance前视距离
Kp = 1.0 ; % speed proportional gain速度P控制器系数
dt = 0.1  ;% 时间间隔，单位：s
L = 0.1  ;% [m] wheel base of vehicle车辆轴距，单位：m

pos_num = size(x,2);
for i = 1:1:pos_num
    cx(i) = x(1,i);
    cy(i) = y(1,i);
end
goal = [x(1,pos_num),y(1,pos_num)];
 
% i = 1;
target_speed = 2/3.6;  %目标车速
T = 300;                 %最大模拟时间
lastIndex = length(cx); %最后索引
x = cx(1); y = cy(1); 
yaw = 0; v = 0;         %初始偏航角  初始车速
time = 0;
Lf = 0.1 * v + Lfc;      %Ld预瞄距离
 
figure;
plot(cx(pos_num),cy(pos_num),'m-*');
hold on
plot(x,y,'m-*');
target_ind = calc_target_index(x,y,cx,cy,Lf);
draw_conv_region(nodepoly,obscell.vert,range);
hold on
while T  > time && lastIndex > target_ind
    target_ind = calc_target_index(x,y,cx,cy,Lf);
    ai = PIDcontrol(target_speed, v,Kp);
    [delta, target_ind] = pure_pursuit_control(x,y,yaw,v,cx,cy,target_ind,0.1,Lfc,L,Lf); 
    [x,y,yaw,v] = update(x,y,yaw,v, ai, delta,dt,L);
    
    time = time + dt;
       %pause(n) 暂停执行 n 秒，然后继续执行。必须启用暂停，此调用才能生效。
  %  hold off
    %h1 = plot(x,y,'r-*');
    %pause(0.1);
    % plot(x,y,'r-*')
    % drawnow  %drawnow 更新图窗并处理任何挂起的回调。如果您修改图形对象并且需要在屏幕上立即查看这次更新，请使用该命令。
    % hold on
    if sqrt((cx(pos_num)-x)^2+(cy(pos_num)-y)^2)<0.05
        break
    end
    %delete(h1);
    %delete(h2);
end
% plot(cx,cy,x,y,'*')
% hold on
 
%更新x  y  yaw  v
function [x, y, yaw, v] = update(x, y, yaw, v, a, delta, dt, L)
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;   % v / L * tan(delta) * dt = v / R * dt =w*dt 
    v = v + a * dt;
    if v > 0.3
        v = 0.3;
    end
end
 
%PID控制
function [a] = PIDcontrol(target_v, current_v, Kp)  %由当前车速调控目标车速
a = Kp * (target_v - current_v);                    %加速度
end
 
%pure pursuit算法
function [delta, ind] = pure_pursuit_control(x,y,yaw,v,cx,cy,ind,~,Lfc,L,Lf)
% ind = calc_target_index(x,y,cx,cy,Lf);
 
if ind < length(cx)  %若目标点没有超过范围，去具体坐标赋予 tx，ty用作目标
    tx = cx(ind);
    ty = cy(ind);
else                %若超过了，把最后一个点赋给目标
    tx = cx(ind); 
    ty = cy(ind);
    ind = length(cx) - 1;
end
 
alpha = atan2((ty-y),(tx-x))-yaw;   %求航向角
 
if v < 0
    alpha = pi - alpha;
else
    alpha = alpha;
end
 
Lf = 0.1 * v + Lfc;    %前视距离的选取与速度有关，也与单位时间距离有关
delta = atan2(2*L * sin(alpha)/Lf,1)  ;         %必须用atan2()：四象限       atan()：二象限
end
 
%计算目标索引
function ind = calc_target_index(x,y, cx,cy,Lf)
N =  length(cx);
Distance = zeros(N,1);  %14x1
dx = zeros(N,1);
dy = zeros(N,1);
for i = 1:N
    dx(i) = x - cx(i);
end
 
for i = 1:N
    dy(i) = y - cy(i);
end
 
for i = 1:N
    Distance(i) = abs( sqrt(dx(i)^2 + dy(i)^2));  %离上一个点的距离
end
[~, location]= min(Distance);    %出最小的Distance所在的位置
                                 %[M,I] = min(___) 在上述语法基础上返回 A 中最小值在运算维度上的索引。
ind = location;
%     首先从目标点中找到一个离当前点最近的点
%     然后计算离这个点距离满足前视距离的下一个点
%     当两点之间的距离小于前视距离，需要累加几个点直至距离超过前视距离
%       
%     # search look ahead target point index
%     # 解读：从path point 接下来中找到 离当前点最接近于 前视距离的一个点
%     # 当路径中的下一个点离当前很远时，这里保证了目标点至少下移一个点，不会停留在原地
 
LL = 0;
 
    while Lf > LL && (ind + 1) < length(cx)
        dx = cx(ind + 1 )- cx(ind);
        dy = cx(ind + 1) - cx(ind);
        LL = LL + sqrt(dx^2 + dy^2);
        ind  = ind + 1;
    end
 
end
function obstacle = createobstcell(obstcub)
n_obs=length(obstcub);
for i=1:n_obs
    obstacle_pts(:,:,i) = obstcub(i).verticesStates.position(1:2,1:4);
end
% reset obstacles vertices in its convexhull (iris requires obstacles to be convex)
obsidx = cell(1,size(obstacle_pts,3));
obstacle.vert = cell(1,size(obstacle_pts,3));
for j = 1:n_obs
    obs = obstacle_pts(:,:,j);
    if size(obs, 2) > 1
        if size(obs, 2) > 2
            % 返回凸包的逆时针索引
            obsidx{j} = convhull(obs(1,:), obs(2,:));
        else
            obsidx{j} = [1,2,1];
        end
    end
    obstacle.vert{j} = obs(:,obsidx{j})';
    obstacle.calc{j} = obs(:,obsidx{j});
end
% obstacles cell
obstacle.poly = obstacle.vert{1}';
for i=2:size(obstacle.vert,1)
    obstacle.poly = [obstacle.poly,NaN(2,1),obstacle.vert{i}'];
end
end



