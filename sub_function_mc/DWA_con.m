function [pose_x,pose_y,pose_th,traj] = DWA_con(x_init,x_init_fm,obs,goal_set,fm,detect_R_l,detect_R)
% obstacles cell
fol_num = fm;      
N = fm + 1;            
countmax = 1500; 
dt = 0.1;         % Control and communication period
gama = 0.5;
beta = 10;  
beta_l = 30;
K_follower = 1;
K_leader = 0.4;       
% control parameters
max_vel_x = 0.3;
max_vel_y = 0.3;
max_acc_x = 0.3;
max_acc_y = 0.3;
% adjacent matrix
A = ones(fm+1);    % a(ij)
%% Initial position matrix
% [x y th]
init_f = x_init;
pose_x = init_f(:,1);
pose_y = init_f(:,2);
pose_th = init_f(:,3);
pose_x(:,2) = init_f(:,1);
pose_y(:,2) = init_f(:,2);
pose_th(:,2) = init_f(:,3);
ob_temp = [];
for obn = 1:length(obs)
    obs_temp = obs(1,obn).verticesStates.position;
    obs_temp = obs_temp';
    ob_temp = [ob_temp;obs_temp];
end
ob_temp_ = [];
ob_temp = [];
for obn = 1:length(obs)
    ob_temp_ = obs(1,obn).verticesStates.position;
    ob_temp_ = ob_temp_';
    hull = convhull(ob_temp_(:,1),ob_temp_(:,2));
    for j = 1:4
        ob_tem(:,1) = (ob_temp_(hull(j,1),1):(ob_temp_(hull(j+1,1),1)-ob_temp_(hull(j,1),1))/100:ob_temp_(hull(j+1,1),1))';
        ob_tem(:,2) = interp1([ob_temp_(hull(j,1),1);ob_temp_(hull(j+1,1),1)],[ob_temp_(hull(j,1),2);ob_temp_(hull(j+1,1),2)],(ob_temp_(hull(j,1),1):(ob_temp_(hull(j+1,1),1)-ob_temp_(hull(j,1),1))/100:ob_temp_(hull(j+1,1),1))');
        ob_temp = [ob_temp;ob_tem];
    end
end
%% Consensus relative position 
x_init_fm = x_init_fm';
% relative position between leader and follwers
error_x = x_init_fm(1,:) - x_init_fm(1,end);  
error_y = x_init_fm(2,:) - x_init_fm(2,end);   
V_x(:,1) = [0;0;0;0];
V_y(:,1) = [0;0;0;0]; 
k = 1;   
d_max = 2;
% detect_R_l = 0.2; 
% detect_R = 0.1;
edge_w = [];
sum_weight = [0;0;0;0];  
x = [x_init(N,1) x_init(N,2) atan2(goal_set(1,2)-x_init(N,2),goal_set(1,1)-x_init(N,1)) 0 0]'; 
obstacleR = 0.5;
global dt; 
dt = 0.1;
% Vmax,wmax,amax,beta_max,
% Resolution of velocity,Resolution of angular velocity 
Kinematic = [0.3,20/180*pi,0.2,50/180*pi,0.01,1/180*pi];
% direction,disance,velocity,prediction time
cost_param = [0.045, 0.1 ,0.1, 3.0];
% cost_param = [2, 0.2 ,0.2, 3.0];
vel_obstacle = 0.1;
temp = 0;
% ob_poses = [1 1;
%     5 -2+4*rand(1);
%     9 3+2*rand(1)];
% obpose(1,1) = ob_poses(2,1);
% obpose(1,2) = ob_poses(2,2);
% obpose(1,3) = ob_poses(3,1);
% obpose(1,4) = ob_poses(3,2);
ob_poses = [0.1 0.1];
%% Motion control
via_num = 1;
for count = 1:countmax
    goal = goal_set(via_num,:);      
    k = k+1;
    %% Leader
%     disance = sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);
%     th = atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));
    
%     if disance > d_max
%         disance = d_max;
%     end
%   V_x(N,k+1) = K_leader*disance*cos(th);
%   V_y(N,k+1) = K_leader*disance*sin(th);
    %% Calculate the agents velocity
%     flag_obstacle = [1-2*rand(1) 1-2*rand(1) 1-2*rand(1)];
%     for jj = 2:3
%         if ob_poses(2,2) > 2 && flag_obstacle(jj) > 0 || ob_poses(2,2) < -2 && flag_obstacle(jj) < 0 ...
%                 || ob_poses(3,2) > 5 && flag_obstacle(jj) > 0 || ob_poses(2,2) < 3 && flag_obstacle(jj) < 0
%             flag_obstacle(jj) = -flag_obstacle(jj);
%         end
%         ob_poses(jj,2)=ob_poses(jj,2)+flag_obstacle(jj)*vel_obstacle;
%     end
%     obpose(k,1) = ob_poses(2,1);
%     obpose(k,2) = ob_poses(2,2);
%     obpose(k,3) = ob_poses(3,1);
%     obpose(k,4) = ob_poses(3,2);
    [u,traj{k-1}] = DynamicWindowApproach(x,Kinematic,goal,cost_param,ob_poses,obstacleR);%??????¡¤?????u/?¡À?¡ã????u
    x(1) = x(1) + u(1,1)*cos(x(3))*dt;
    x(2) = x(2) + u(1,1)*sin(x(3))*dt;
    x(3) = x(3) + u(2,1)*dt;
    x(4) = u(1,1);
    x(5) = u(2,1);
    V_x(N,k+1) = x(4)*cos(x(3));
    V_y(N,k+1) = x(4)*sin(x(3));
%     repulsion = compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R_l);        
%     V_x(N,k+1) = V_x(N,k+1)+beta_l*repulsion(1);
%     V_y(N,k+1) = V_y(N,k+1)+beta_l*repulsion(2);
%     if(disance > 1 && abs(V_x(N,k+1)) <= 0.1 && abs(V_y(N,k+1)) <= 0.1)
%         V_x(N,k+1) = -1+2*rand(1);
%         V_y(N,k+1) = -1+2*rand(1);
%     end
%      v_next = confine([V_x(N,k) V_y(N,k)],[V_x(N,k+1) V_y(N,k+1)],max_vel_x,max_vel_y,max_acc_x,max_acc_y,0.1);
%      V_x(N,k+1) = v_next(1);
%      V_y(N,k+1) = v_next(2);
     pose_x(N,k+1) = x(1);
     pose_y(N,k+1) = x(2);
     pose_th(N,k+1) = atan2(V_y(N,k+1),V_x(N,k+1));
     %% Followers
    for i = 1:fol_num        
        sum_error_x = 0;
        sum_error_y = 0;
        sum_edge_weight = 0;
        % One-order Consensus control
        for j = 1:N       
            if A(i,j) == 1
                w_ij = 2-exp(-((pose_x(j,k-1)-pose_x(i,k)-(error_x(j)-error_x(i)))^2+(pose_y(j,k-1)-pose_y(i,k)-(error_y(j)-error_y(i)))^2));  
                sum_error_x = sum_error_x+A(i,j)*w_ij*((pose_x(j,k-1)-pose_x(i,k))-(error_x(j)-error_x(i)));
                sum_error_y = sum_error_y+A(i,j)*w_ij*((pose_y(j,k-1)-pose_y(i,k))-(error_y(j)-error_y(i)));
                sum_edge_weight = sum_edge_weight+w_ij;
            end
        end
        edge_w(i,k) = sum_edge_weight;
        if edge_w(i,k) > edge_w(i,k-1) && k>2
            sum_weight(i,k) = sum_weight(i,k-1)+abs(edge_w(i,k)-edge_w(i,k-1));
        else
            sum_weight(i,k) = sum_weight(i,k-1);
        end
        if mod(k,100) == 1 
            sum_weight(i,k) = 0;
        end
        disance = sqrt(sum_error_x^2+ sum_error_y^2);
        th = atan2(sum_error_y, sum_error_x);
        if disance > d_max
            disance = d_max;
        end
        V_x(i,k+1) = K_follower*V_x(N,k) + gama*disance*cos(th);
        V_y(i,k+1) = K_follower*V_y(N,k) + gama*disance*sin(th);
        v_next = param_lim([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],max_vel_x,max_vel_y,max_acc_x,max_acc_y,0.1);
        V_x(i,k+1) = v_next(1);
        V_y(i,k+1) = v_next(2);
        kk = 0;
        for j = 1:N
            if j ~= i
                kk = kk+1;
                obs_pose(kk,1) = pose_x(j,k);
                obs_pose(kk,2) = pose_y(j,k);
            end
        end
        ob_pose = [obs_pose;ob_temp];
        % repulsion = [0 0];
        repulsion = compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
        V_x(i,k+1) = V_x(i,k+1) + beta*repulsion(1);
        V_y(i,k+1) = V_y(i,k+1) + beta*repulsion(2);
    end
    for i=1:N-1
        v_next = param_lim([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],max_vel_x,max_vel_y,max_acc_x,max_acc_y,0.1);
        V_x(i,k+1) = v_next(1);
        V_y(i,k+1 ) =v_next(2);
        pose_x(i,k+1) = pose_x(i,k) + dt*V_x(i,k+1);
        pose_y(i,k+1) = pose_y(i,k) + dt*V_y(i,k+1);
        pose_th(i,k+1) = atan2(V_y(i,k+1),V_x(i,k+1));
    end
    % Judge whether arrived
    if norm(x(1:2)-goal') < 0.05
        disp('Arrive subgoal!');
        if via_num ~= size(goal_set,1) 
            via_num = via_num + 1;
            x(3) = atan2(goal_set(via_num,2)-x(2),goal_set(via_num,1)-x(1));
        else
            break;
        end
    end
end
end
%% DWA
function [u,traj] = DynamicWindowApproach(x,model,goal,cost_param,ob,R)
% state, model -> current allowed range of parameters
current_window = cal_currrent_window(x,model);  
% velocity, angular velocity, Angle score, distance score, velocity score
[score,traj]= cost(x,current_window,goal,ob,R,model,cost_param);  
% Normalization
if sum(score(:,3))~= 0
    score(:,3) = score(:,3)/sum(score(:,3)); 
else
    score(:,3) = score(:,3);
end
if sum(score(:,4))~= 0
    score(:,4) = score(:,4)/sum(score(:,4));
else
    score(:,4) = score(:,4);
end
if sum(score(:,5))~= 0
    score(:,5) = score(:,5)/sum(score(:,5));
else
    score(:,5) = score(:,5);
end
if isempty(score)
    error('Fail to generate the path to goal!');
    u = [0;0]; return;
end

score_result = cal_score_result(score,cost_param);
% final score
score = [score score_result]; 
% return the best control prameters
[~,ind] = max(score_result);
u = score(ind,1:2)';
end
%% Scoring function
% score:N*5
function [score,traj] = cost(x,current_window,goal,obs,R,model,cost_param)
score = []; traj = [];
v_min = current_window(1);
v_res = model(5);
v_max = current_window(2);
w_min = current_window(3);
w_res = model(6);
w_max = current_window(4);
for v = v_min:v_res:v_max     
    for w = w_min:w_res:w_max
        [dis,hit] = cal_obs_dis(xt,obs,R);    
        [xt,traj] = generate_traj(x,v,w,cost_param(4));  
        stopdis = cal_breakdis(abs(v),model); 
        % Directional deviation score
        heading = 180 - abs(atan2(goal(2)-xt(2),goal(1)-xt(1))/pi*180-xt(3)/pi*180); 
        % velocity = abs(v);                     
        % discard the path if it is likely to hit the nearest obstacle
        if dis > stopdis && hit == 0 
            score_ = [v w heading dis abs(v)];
            score = [score;score_];
            traj = [traj;traj];    
        end
    end
end
end
 
%% Generate predicted trajectories
function [x,traj] = generate_traj(x,v,w,pred_time)
global dt; time = 0; traj = x;  
while time <= pred_time   
    time = time + dt; 
    % Update the state
    x(1) = x(1) + v*cos(x(3))*dt;
    x(2) = x(2) + v*sin(x(3))*dt;
    x(3) = x(3) + w*dt;
    x(4) = v;
    x(5) = w;
    traj = [traj x]; 
end
end
%% Calculating braking distance
function stopdis = cal_breakdis(vel,model)
global dt;
stopdis=0;
% given the acceleration, calculate the breaking distance 
while vel > 0   
    stopdis = stopdis + vel*dt;
    vel = vel - model(3)*dt;
end
end
%% Obstacle distance evaluation function
function [dis,hit] = cal_obs_dis(x,obs,R)
for i = 1:length(obs(:,1))  
    if norm(obs(i,:)-x(1:2)')-R < 0
        hit = 1; break;
    else
        hit = 0;
    end   
    dis = min(100, norm(obs(i,:)-x(1:2)')-R);
end
% Maximum score limit
dis = min(dis, 3*R);
end
%% Calculate current dynamic windows
function current_window = cal_currrent_window(x,model)
global dt;
% minimum velocity, maximum velocity, minimum angular velocity, maximum angular velocity 
V_now = [x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
V_pre = [0 model(1) -model(2) model(2)];
Vtmp = [V_pre;V_now]; 
current_window = [max(Vtmp(:,1)) min(Vtmp(:,2))... 
    max(Vtmp(:,3)) min(Vtmp(:,4))];
end
%% Compute the repulsion
function [ repulsion] = compute_repulsion(robot_pose,obs_pose,detect_R)
[M,~] = size(obs_pose);
repulsion(1) = 0;
repulsion(2) = 0; 
for i = 1:M
    disance = sqrt((robot_pose(1)-obs_pose(i,1))^2 + (robot_pose(2)-obs_pose(i,2))^2);
    if disance <= detect_R
        temp = (1/disance-1/detect_R)/(disance^3);
        repulsion(1) = repulsion(1)+temp*(robot_pose(1)-obs_pose(i,1));
        repulsion(2) = repulsion(2)+temp*(robot_pose(2)-obs_pose(i,2));
    end
end
end
%% Calculate the score for each feasible path 
function score_result = cal_score_result(score,cost_param)
score_result=[];
for id=1:length(score(:,1))
    score_result = [score_result;
        cost_param(1:3)*score(id,3:5)']; 
end
end