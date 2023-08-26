%% Draw diagram
function draw_diag(pose_xp2f,pose_yp2f,V,obstcell,fm,x_init,cir)
pose_x = [pose_xp2f];
pose_y = [pose_yp2f];
color='mgbkrc'; %%%corresponding to 6 colors
type=[2,1,0.5,0.5,2,2];%%%different line type
N = fm + 1;
for i = 1:N-1
    for j = 1:size(pose_xp2f,2)
        centre = cal_centre(pose_xp2f(1:N-1,j),pose_yp2f(1:N-1,j),fm,cir);
        error_p(i,j) = norm([pose_xp2f(i,j)-centre(1),pose_yp2f(i,j)-centre(2)]) -...
            norm([x_init(i,1)-x_init(N,1),x_init(i,2)-x_init(N,2)]);
    end
end
figure
% Draw the path record of formation 
for i=1:N
    plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',2);
    hold on
end
for i=1:N-1
    plot(pose_x(i,1),pose_y(i,1),'bp','color',color(1,i),'LineWidth',1);
    hold on
end
plot(pose_x(N,1),pose_y(N,1),'*','color',color(1,N),'LineWidth',1);
hold on
% for i=1:N-1
%     plot(pose_xi2p(i,end),pose_yi2p(i,end),'d','color',color(1,i),'LineWidth',2);
%     hold on
% end
hold on
for i=1:N-1
    plot(pose_x(i,end),pose_y(i,end),'m^','color',color(1,i),'LineWidth',2);
    hold on
end
plot(pose_x(N,end),pose_y(N,end),'o','color',color(1,N),'LineWidth',2);
hold on
%     if size(ob_temp)~=[0 0]
%         plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
%     end
obs_num = numel(obstcell.vert);
if isempty(obstcell.vert)
    obs_num = 0;
end
% Draw obstacle interiors
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
% Draw obstacle boundaries 
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
patch('Vertices',[1.8+0.05,4+0.05;1.8+0.05,4-0.05;...
            1.8-0.05,4-0.05;1.8-0.05,4+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
grid on;
xlabel('x');
ylabel('y');
if fm == 3
    legend('Follower 1','Follower 2','Follower 3','Leader');
elseif fm == 2
    legend('Follower 1','Follower 2','Leader');
else
    legend('Follower 1','Follower 2','Follower 3','Follower 4','Leader');
end
xlabel('x(m)');
ylabel('y(m)');
title('Formation Consensus');
figure 
dt = 0.1;
t_data = 0:dt:dt*(size(pose_x,2)-1);
for i = 1:N-1  
    plot(t_data',error_p(i,:),'color',color(1,i));
    hold on
end
xlabel('time/s');ylabel('v/m*s-1');zlabel('v/m*s-1');grid;
title('Formation Position Error');
if fm == 3
    legend('Follower 1','Follower 2','Follower 3','Leader');
elseif fm == 2
    legend('Follower 1','Follower 2','Leader');
else
    legend('Follower 1','Follower 2','Follower 3','Follower 4','Leader');
end
figure 
dt = 0.1;
t_data = 0:dt:dt*(size(pose_x,2)-1);
for i = 1:size(pose_x,1)  
    plot(t_data',V(i,:),'color',color(1,i));
    hold on
end
title('Formation Velocity Error');
xlabel('time/s');ylabel('v/m*s-1');zlabel('v/m*s-1');grid;
if fm == 3
    legend('Follower 1','Follower 2','Follower 3','Leader');
elseif fm == 2
    legend('Follower 1','Follower 2','Leader');
else
    legend('Follower 1','Follower 2','Follower 3','Follower 4','Leader');
end
figure
for i = 1:size(pose_x,1)  
    plot3(t_data',pose_x(i,:),pose_y(i,:),'color',color(1,i));
    hold on
end
hold on
plot3(t_data',1.8*ones(1,length(t_data)),4*ones(1,length(t_data)),'color',color(1,5));
title('Formation Trajectories in Time-configuration Space');
xlabel('time/s');ylabel('x/m');zlabel('y/m');grid;
if fm == 3
    legend('Follower 1','Follower 2','Follower 3','Leader');
elseif fm == 2
    legend('Follower 1','Follower 2','Leader');
else
    legend('Follower 1','Follower 2','Follower 3','Follower 4','Leader');
end
end
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