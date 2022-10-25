%% Motion Control
function [pose_x,pose_y,pose_th,V_x,V_y,rep_lx,rep_ly,rep_fx,rep_fy,rep_ind] = apf_con(x_init,x_init_fm,obs,goal_set,fm,detect_R_l,detect_R)
% x_init_fm(3,1) = 9.2205;
% x_init_fm(4,2) = 9.7613;
% x_init_fm(6,2) = 9.3613;
% x_init_fm(10,1) = 9.6205;
fol_num = fm-1;      
N = fm;      
% fm_arr = [2 0 0;1 0 0;3 0 0;3 0 0;1 0 0;3 0 0;2 0 0;1 0 1;2 0 1;3 0 1];
Itermax = 1000;   
dt = 0.1;                           % time lapse
gama = 0.5;
K_leader = 0.2;
K_follower = 0.2;      
% control parameters
max_vel_x = 0.3;
max_vel_y = 0.3;
max_acc_x = 0.3;
max_acc_y = 0.3;
A = ones(fm);                       % adjacent matrix
arr_ind = repmat(0,fm,1);
%% Initial position matrix
% [x y th]
init_f = x_init;
dest_set = x_init_fm;
pose_x=init_f(:,1);
pose_y=init_f(:,2);
pose_th=init_f(:,3);
pose_x(:,2)=init_f(:,1);
pose_y(:,2)=init_f(:,2);
pose_th(:,2)=init_f(:,3);
ob_temp_ = [];
ob_temp = [];
for obn = 1:length(obs)
    ob_temp_ = obs(1,obn).verticesStates.position;
    ob_temp_ = ob_temp_';
    hull = convhull(ob_temp_(:,1),ob_temp_(:,2));
    for j = 1:4
        ob_tem(:,1) = (ob_temp_(hull(j,1),1):(ob_temp_(hull(j+1,1),1)-ob_temp_(hull(j,1),1))/1000:ob_temp_(hull(j+1,1),1))';
        ob_tem(:,2) = interp1([ob_temp_(hull(j,1),1);ob_temp_(hull(j+1,1),1)],[ob_temp_(hull(j,1),2);ob_temp_(hull(j+1,1),2)],(ob_temp_(hull(j,1),1):(ob_temp_(hull(j+1,1),1)-ob_temp_(hull(j,1),1))/1000:ob_temp_(hull(j+1,1),1))');
        % slope((j-1)*10+obn,1) = atan((ob_tem(end,2)-ob_tem(1,2))/(ob_tem(end,1)-ob_tem(1,1)));
        ob_temp = [ob_temp;ob_tem];
    end
end
obs_ind(1,1) = 1;
obs_ind(1,2) = 1001;
obs_ind(1,3) = ob_temp(1,1);
obs_ind(1,4) = ob_temp(1,2);
obs_ind(1,5) = ob_temp(1001,1);
obs_ind(1,6) = ob_temp(1001,2);
for i = 2:obn*4
    obs_ind(i,1) = obs_ind(i-1,2) + 1;
    obs_ind(i,2) = obs_ind(i,1) + 1000;
    obs_ind(i,3) = ob_temp(obs_ind(i,1),1);
    obs_ind(i,4) = ob_temp(obs_ind(i,1),2);
    obs_ind(i,5) = ob_temp(obs_ind(i,2),1);
    obs_ind(i,6) = ob_temp(obs_ind(i,2),2);
end
%% formation consensus relative position 
x_init_fm = x_init_fm';
error_x = x_init_fm(1,:) - x_init_fm(1,end);  
error_y = x_init_fm(2,:) - x_init_fm(2,end);   
V_x(:,1)=[0;0;0;0];
V_y(:,1)=[0;0;0;0]; 
k=1;   
d_max = 1;
beta_l = 300;
beta_f = 300;
edge_w = [];                           % edge weighted matrix
sum_weight = zeros(fm-1,1);      
local_min_M = zeros(fol_num,5);
%% Motion Control
via_num = 1;
for count=1:Itermax
	goal = goal_set(via_num,:);      
	k=k+1;
    %% Leader
    % attraction from goal 
    if abs(pose_x(N,end)-dest_set(N,1)) < 0.01 && abs(pose_y(N,end)-dest_set(N,2)) < 0.01
        arr_ind(N) = 1;
    end
    if arr_ind(N,1) == 1
         V_x(N,k+1) = 0;
         V_y(N,k+1) = 0;
    else
%         if abs(pose_x(N,end-1)-dest_set(N,1)) < 0.3 && abs(pose_y(N,end-1)-dest_set(N,2)) < 0.3
%             fm_arr(N,2) = 1;
%         end
        flag = false;
        if mean(abs(pose_x(1:N-1,end)-dest_set(1:N-1,1))) < 0.2 && mean(abs(pose_y(1:N-1,end)-dest_set(1:N-1,2))) < 0.2
            flag = false;
        end
        if flag 
            K_leader = 1;
        else
            distance = norm([goal(1)-pose_x(N,k),goal(2)-pose_y(N,k)]);
            theta = atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));
            if distance > d_max
                distance = d_max;
            end
            V_x(N,k+1) = K_leader*distance*cos(theta);
            V_y(N,k+1) = K_leader*distance*sin(theta);
            rest_rob = 0;
            for j = 1:N
                if j ~= N
                    rest_rob = rest_rob+1;
                    obs_pose(rest_rob,1) = pose_x(j,k);
                    obs_pose(rest_rob,2) = pose_y(j,k);
                end
            end
            ob_set = [ob_temp];
            repulsion = compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_set,detect_R_l,length(obs_pose),N,x_init_fm);        
            % repulsion from obstacle 
            V_x(N,k+1) = V_x(N,k+1)+beta_l*repulsion(1);
            V_y(N,k+1) = V_y(N,k+1)+beta_l*repulsion(2);
            rep_lx(1,k-1) = repulsion(1);
            rep_ly(1,k-1) = repulsion(2);
            % escape from local optimal 
            if distance > d_max && abs(V_x(N,k+1)) <= 0.1 && abs(V_y(N,k+1)) <= 0.1
                V_x(N,k+1) = -1 + 2*rand(1);
                V_y(N,k+1) = -1 + 2*rand(1);
            end
             v_next = param_lim([V_x(N,k) V_y(N,k)],[V_x(N,k+1) V_y(N,k+1)],max_vel_x,max_vel_y,max_acc_x,max_acc_y,0.1);
             V_x(N,k+1) = v_next(1);
             V_y(N,k+1) = v_next(2);
         end
     %end
    %% Followers
    for i = 1:fol_num     
        if abs(pose_x(i,end)-dest_set(i,1)) < 0.01 && abs(pose_y(i,end)-dest_set(i,2)) < 0.01
            arr_ind(i) = 1;
        end
        if arr_ind(i) == 1
             V_x(i,k+1) = 0;
             V_y(i,k+1) = 0;
             A(i,:) = 0;
        else
            % flag = mean(abs(pose_x(:,end)-dest_set(:,1))) < 0.1 && mean(abs(pose_y(:,end)-dest_set(:,2))) < 0.1;
            sum_error_x = 0;
            sum_error_y = 0;
            sum_edge_weight = 0;
            % one-order Consensus control
            for j=1:N       
                if  A(i,j) == 1
                    % calculate edge weights
                    % the difference between the predicted error and the actual error 
                    wij = 2 - exp(-((pose_x(j,k-1) - pose_x(i,k) - (error_x(j) - error_x(i)))^2+(pose_y(j,k-1) - pose_y(i,k) - (error_y(j) - error_y(i)))^2)); 
                    sum_error_x = sum_error_x + A(i,j)*wij*( (pose_x(j,k-1) - pose_x(i,k)) - (error_x(j) - error_x(i)));
                    sum_error_y = sum_error_y + A(i,j)*wij*((pose_y(j,k-1) - pose_y(i,k)) - (error_y(j) - error_y(i)));
                    sum_edge_weight = sum_edge_weight + wij;
                end
            end
            edge_w(i,k) = sum_edge_weight;
            % store the weight error into sum_weight matrix
            if edge_w(i,k) > edge_w(i,k-1) && k>2
                sum_weight(i,k) = sum_weight(i,k-1)+abs(edge_w(i,k)-edge_w(i,k-1));
            else
                sum_weight(i,k) = sum_weight(i,k-1);
            end
            if mod(k,100) == 1 % reset matrix to 0 after period 10s
                sum_weight(i,k)=0;
            end
            % ideal velocity without repulsion from obstalcle     
            distance = norm([dest_set(i,1)-pose_x(i,k),dest_set(i,2)-pose_y(i,k)]);
            distance = min(distance,d_max);
            theta = atan2(dest_set(i,2)-pose_y(i,k), dest_set(i,1)-pose_x(i,k));
            V_x(i,k+1) = K_follower*distance*cos(theta);
            V_y(i,k+1) = K_follower*distance*sin(theta);
            if flag
                K_follower = 1;
                distance = norm([sum_error_x,sum_error_y]);
                theta = atan2(sum_error_y, sum_error_x);
                distance = min(distance,d_max);
                V_x(i,k+1) = K_follower*V_x(N,k) + gama*distance*cos(theta); 
                V_y(i,k+1) = K_follower*V_y(N,k) + gama*distance*sin(theta);
            end
            v_next = param_lim([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],max_vel_x,max_vel_y,max_acc_x,max_acc_y,0.1);
            V_x(i,k+1) = v_next(1);
            V_y(i,k+1) = v_next(2);
            % if mean(abs(pose_x(:,end)-dest_set(:,1))) < 0.1 && mean(abs(pose_y(:,end)-dest_set(:,2))) < 0.1 
            % consider repulsion between robots
            rest_rob = 0;
            for j = 1:N
                if j ~= i
                    rest_rob = rest_rob+1;
                    obs_pose(rest_rob,1) = pose_x(j,k);
                    obs_pose(rest_rob,2) = pose_y(j,k);
                end
            end
            ob_set=[obs_pose;ob_temp];
            % end
            [repulsion,rep_obs_list,ind] = compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_set,detect_R,length(obs_pose),i,x_init_fm);        
            V_x(i,k+1) = V_x(i,k+1) + beta_f*repulsion(1);
            V_y(i,k+1) = V_y(i,k+1) + beta_f*repulsion(2);
            rep_fx(i,k-1) = repulsion(1);
            rep_fy(i,k-1) = repulsion(2);
            rep_ind(i,k-1) = ind;
            v_next = param_lim([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],max_vel_x,max_vel_y,max_acc_x,max_acc_y,0.1);
            V_x(i,k+1) = v_next(1);
            V_y(i,k+1) = v_next(2);
           %% Local minimal
            if local_min_M(i,1) == 1 && norm([pose_x(i,k)-local_min_M(i,2),pose_y(i,k)-local_min_M(i,3)]) > 0.2 && ...
                    repulsion(1) ~= 0 && repulsion(2) ~= 0
                V_x(i,k+1) = local_min_M(i,4);
                V_y(i,k+1) = local_min_M(i,5);
            % local minimal -> Sub Algorithm
            elseif (abs(V_x(i,k+1)) < 0.06 || abs(V_y(i,k+1)) < 0.06) && ...
                    all((rep_obs_list(1:fol_num,1))==inf) && k > 2 ...
                    && ~all((rep_obs_list(1:end,1))==inf)
                local_min_M(i,1) = 1;
                [~,ind] = min(rep_obs_list);
                ind = ind - fol_num;
                for j = 1:length(obs_ind)
                    if obs_ind(j,1) <= ind && ind <= obs_ind(j,2)
                        if norm([dest_set(i,1)-obs_ind(j,3),dest_set(i,2)-obs_ind(j,4)]) > ...
                                norm([dest_set(i,1)-obs_ind(j,5),dest_set(i,2)-obs_ind(j,6)])
                            temp_dir = atan2((obs_ind(j,6)-obs_ind(j,4)),(obs_ind(j,5)-obs_ind(j,3)));
                            local_min_M(i,2) = obs_ind(j,5);
                            local_min_M(i,3) = obs_ind(j,6);
                        else
                            temp_dir = atan2((obs_ind(j,4)-obs_ind(j,6)),(obs_ind(j,3)-obs_ind(j,5)));
                            local_min_M(i,2) = obs_ind(j,3);
                            local_min_M(i,3) = obs_ind(j,4);
                        end
                    end
                end
                V_x(i,k+1) = max_vel_x*cos(temp_dir);
                V_y(i,k+1) = max_vel_y*(temp_dir);
                local_min_M(i,4) = V_x(i,k+1);
                local_min_M(i,5) = V_y(i,k+1);
            else
                V_x(i,k+1) = v_next(1);
                V_y(i,k+1) = v_next(2);
                local_min_M(i,1:5) = 0;
            end
       end
    end
    % update the position and calculate error between prediction and real position
	for i=1:N
        pose_x(i,k+1) = pose_x(i,k) + dt*V_x(i,k+1);
        pose_y(i,k+1) = pose_y(i,k) + dt*V_y(i,k+1);
        pose_th(i,k+1) = atan2(V_y(i,k+1),V_x(i,k+1));
	end
	% Judge whether arrive the goal
	if mean(abs(pose_x(:,end)-dest_set(:,1))) < 0.01 && mean(abs(pose_y(:,end)-dest_set(:,2))) < 0.01 
        disp('Arrive Goal!!');
        if via_num ~= size(goal_set,1) 
            via_num = via_num + 1;
        else
            break;
        end
    end
    if count == Itermax
        %error('fail!');
        return
    end
end
end