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
% zarray�켣�ľ����㣬dzarrayΪÿ������ٶȣ�tarrayΪʱ��Ƭ��Ϣ
%k = 0.1;  % look forward gainǰ�Ӿ���ϵ��
Lfc = 0.1;  % look-ahead distanceǰ�Ӿ���
Kp = 1.0 ; % speed proportional gain�ٶ�P������ϵ��
dt = 0.1  ;% ʱ��������λ��s
L = 0.1  ;% [m] wheel base of vehicle������࣬��λ��m

pos_num = size(x,2);
for i = 1:1:pos_num
    cx(i) = x(1,i);
    cy(i) = y(1,i);
end
goal = [x(1,pos_num),y(1,pos_num)];
 
% i = 1;
target_speed = 2/3.6;  %Ŀ�공��
T = 300;                 %���ģ��ʱ��
lastIndex = length(cx); %�������
x = cx(1); y = cy(1); 
yaw = 0; v = 0;         %��ʼƫ����  ��ʼ����
time = 0;
Lf = 0.1 * v + Lfc;      %LdԤ�����
 
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
       %pause(n) ��ִͣ�� n �룬Ȼ�����ִ�С�����������ͣ���˵��ò�����Ч��
  %  hold off
    %h1 = plot(x,y,'r-*');
    %pause(0.1);
    % plot(x,y,'r-*')
    % drawnow  %drawnow ����ͼ���������κι���Ļص���������޸�ͼ�ζ�������Ҫ����Ļ�������鿴��θ��£���ʹ�ø����
    % hold on
    if sqrt((cx(pos_num)-x)^2+(cy(pos_num)-y)^2)<0.05
        break
    end
    %delete(h1);
    %delete(h2);
end
% plot(cx,cy,x,y,'*')
% hold on
 
%����x  y  yaw  v
function [x, y, yaw, v] = update(x, y, yaw, v, a, delta, dt, L)
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;   % v / L * tan(delta) * dt = v / R * dt =w*dt 
    v = v + a * dt;
    if v > 0.3
        v = 0.3;
    end
end
 
%PID����
function [a] = PIDcontrol(target_v, current_v, Kp)  %�ɵ�ǰ���ٵ���Ŀ�공��
a = Kp * (target_v - current_v);                    %���ٶ�
end
 
%pure pursuit�㷨
function [delta, ind] = pure_pursuit_control(x,y,yaw,v,cx,cy,ind,~,Lfc,L,Lf)
% ind = calc_target_index(x,y,cx,cy,Lf);
 
if ind < length(cx)  %��Ŀ���û�г�����Χ��ȥ�������긳�� tx��ty����Ŀ��
    tx = cx(ind);
    ty = cy(ind);
else                %�������ˣ������һ���㸳��Ŀ��
    tx = cx(ind); 
    ty = cy(ind);
    ind = length(cx) - 1;
end
 
alpha = atan2((ty-y),(tx-x))-yaw;   %�����
 
if v < 0
    alpha = pi - alpha;
else
    alpha = alpha;
end
 
Lf = 0.1 * v + Lfc;    %ǰ�Ӿ����ѡȡ���ٶ��йأ�Ҳ�뵥λʱ������й�
delta = atan2(2*L * sin(alpha)/Lf,1)  ;         %������atan2()��������       atan()��������
end
 
%����Ŀ������
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
    Distance(i) = abs( sqrt(dx(i)^2 + dy(i)^2));  %����һ����ľ���
end
[~, location]= min(Distance);    %����С��Distance���ڵ�λ��
                                 %[M,I] = min(___) �������﷨�����Ϸ��� A ����Сֵ������ά���ϵ�������
ind = location;
%     ���ȴ�Ŀ������ҵ�һ���뵱ǰ������ĵ�
%     Ȼ�������������������ǰ�Ӿ������һ����
%     ������֮��ľ���С��ǰ�Ӿ��룬��Ҫ�ۼӼ�����ֱ�����볬��ǰ�Ӿ���
%       
%     # search look ahead target point index
%     # �������path point ���������ҵ� �뵱ǰ����ӽ��� ǰ�Ӿ����һ����
%     # ��·���е���һ�����뵱ǰ��Զʱ�����ﱣ֤��Ŀ�����������һ���㣬����ͣ����ԭ��
 
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
            % ����͹������ʱ������
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



