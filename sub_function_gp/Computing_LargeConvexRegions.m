function [A, b, C, d] = Computing_LargeConvexRegions(obstacles, A_boundary, b_boundary, start, varargin)
obs = obstacles;
p = inputParser();
p.addOptional('iter_limit', 100, @isnumeric);
p.addOptional('flag_start', false, @isscalar);
p.addOptional('termination_threshold', 2e-2, @(x) x > 0);
p.addOptional('require_containment', false, @isscalar);
p.parse(varargin{:});                       
if iscell(obstacles)
    obstacle_pts = cell2mat(reshape(obstacles, size(obstacles, 1), [], size(obstacles,2)));
else
    obstacle_pts = obstacles;
end

% Initializing ellipse
dim = size(A_boundary, 2);
C = 1.0e-04 * eye(dim); d = start;
best_val = -inf;
iter = 1;

while true
	[A, b, invalid_start] = SeparatingHyperplane(obstacle_pts, C, d);
	if invalid_start && p.Results.flag_start 
        error('Invalid Start Point!');
	end
	if iter > 1
    for i = 1:length(b)
        assert(min(eig([(b(i) - A(i,:) * d) * eye(dim), C * (A(i,:)');
        (C * (A(i,:)'))', (b(i) - A(i,:) * d)])) >= -1e-3);
    end
	end
	A = [A; A_boundary];
	b = [b; b_boundary];
    range = [0 0;10 10];
   nodez = [1.5 1.5 0.3 0.15;8.5 8.5 0.3 0.15];
    O_x = 0.1; O_y = 0.1; 
    P_x = 0.1; P_y = 0.1; 
    fm = 4;
    cir = 0;
    if C(1,1) < 2.0e-04
        if size(obs{1,2},2) > 2
            for j = 1:length(obs)
                temp = obs{1,j};
                obs{1,j} = temp';
            end
        end
        region.A = A;
        region.b = b;
        region.C = C;
        region.d = d;
        draw_conv_region(region,obs,range);
    end
	[C, d, opt_val] = InscribedEllipsoid(A,b);
    region.A = A;
    region.b = b;
    region.C = C;
    region.d = d;
    drawformation(nodez,[O_x,O_y],[P_x,P_y],fm,cir);
    if size(obs{1,2},2) > 2
        for j = 1:length(obs)
            temp = obs{1,j};
            obs{1,j} = temp';
        end
    end
    disp(A);
    disp(b);
    disp(C);
    disp(d);
	if invalid_start && p.Results.flag_start 
        error('Invalid Start Point!');
	end
   draw_conv_region(region,obs,range);
    if ~config_in_obs(nodez,obstacle_pts,[O_x,O_y],[P_x,P_y],fm,cir)
        error('Start or goal point is inside the obstacles!');
    end
	% check iteration termination conditions
    if iter >= p.Results.iter_limit || abs(opt_val - best_val)/best_val < 2e-2 
        break
    end
    iter = iter + 1;
	best_val = opt_val;	
end
end
%% Compute the separation plane
function [A, b, invalidstart] = SeparatingHyperplane(obs_vertice, C, d)
invalidstart = false;
obs_num = size(obs_vertice, 3);
vertice_num = size(obs_vertice, 2);
Cinv = inv(C);
Cinv_2 = (Cinv * Cinv');
if obs_num == 0 || isempty(obs_vertice)
    invalidstart = false;
    A = zeros(0, size(C,1));
    b = zeros(0, 1);
    return;
end
detected_obstacles = true(obs_num,1);
obstacles_free = false(obs_num, 1);

% obstacle vertex coordinates after mapping
obs_vertice_cal = reshape(obs_vertice, size(C,1), []);
for dim = 1:size(C,1)
    diff(dim,:) = obs_vertice_cal(dim,:) - d(dim);
end
Cinv_m_diff = reshape(Cinv * diff, size(obs_vertice));
  % Compute x^2
obs_dis = reshape(sum(Cinv_m_diff.^2, 1), size(obs_vertice, 2), size(obs_vertice, 3));
obs_min_diff = min(obs_dis, [], 1);
[~, obs_sort_idx] = sort(obs_min_diff);
A = zeros(obs_num,size(C,1));
b = zeros(obs_num,1);

for i = obs_sort_idx
	if detected_obstacles(i)
        obs = obs_vertice(:,:,i);
        % multiply difference
        md = Cinv_m_diff(:,:,i);
        dis = obs_dis(:,i);
        [~,idx] = min(dis);
        xi = obs(:,idx);
        % vectors that are perpendicular to the surface of the ellipse
        % aj=2C-1C-T(x*-d)
        nor_vec = 2 * Cinv_2 * (xi - d);
        nor_vec = nor_vec / (sqrt((nor_vec(1))^2+(nor_vec(2))^2));
        % bj=ajTx*
        btem = nor_vec' * xi;
        if all(nor_vec' * obs - btem>= 0)
        % nor_vec is feasible, so we can skip the optimization
        A(i,:) = nor_vec';
        b(i) = btem;
        else
            md(isnan(md)) = 0;
            md(isinf(md)) = 10^5;
            % find linear constraints for given vertices
            [As,bs] = vert2lcon(md');
            % minimize 0.5*||eye(size(md,1))*x-[0;0]||^2 (subject to As*x <= bs) 
            x_mstar = lsqlin(eye(size(md,1)),[0;0],As,bs);
        if sqrt((x_mstar(1))^2+(x_mstar(2))^2) < 1e-3
            % ellipse is inside the obstacle
            % reverse nor_vec to push the ellipsoid out of the obstacle
            warning('Ellipse center is inside the obstacle.');
            invalidstart = true;
            A(i,:) = -nor_vec';
            b(i) = -nor_vec' * xi;
        else
          xstar = C*x_mstar + d;
          nor_vec = 2 * Cinv_2 * (xstar - d);
          nor_vec = nor_vec / (sqrt((nor_vec(1))^2+(nor_vec(2))^2));
          A(i,:) = nor_vec;
          b(i) = nor_vec' * xstar;
        end
        end
        check = A(i,:) * obs_vertice_cal >= b(i);
        check = reshape(check', vertice_num, []);
        obs_check_ind = all(check, 1);
        detected_obstacles(obs_check_ind) = false;
        detected_obstacles(i) = false;
        obstacles_free(i) = true;
        if ~any(detected_obstacles)
            break
        end
	end
end
A = A(obstacles_free,:);
b = b(obstacles_free);
end
%% Solve the Semidefinite optimization with SDPT3 solver
function [C, d, opt_val] = InscribedEllipsoid(A,b)
[C, ia] = unique(A,'rows');
A = C;
b = b(ia);
A(isnan(A)) = 0;
A(isinf(A)) = 10^5;
b(isnan(b)) = 0;
b(isinf(b)) = 10^5;
cvx_begin sdp quiet
  cvx_solver SDPT3
  variable C(size(A,2),size(A,2)) semidefinite
  variable d(size(A,2))
  % log(det(C)), concave and nonmonotonic
  maximize(det_rootn(C))
  subject to
    for i = 1:length(b)
      [(b(i) - A(i,:) * d) * eye(size(A,2)), C * (A(i,:)');(C * (A(i,:)'))', (b(i) - A(i,:) * d)] >= 0;
    end
cvx_end
opt_val = det(C);
end
%% Check if the configuration is in existing obstacles
function flag = config_in_obs(node,obstacle_pts,O,P,fm,cir)
flag = true;
import rvctools.*
for sg = 1:2
    xr = [node(sg,1:2),0]; 
    % xm(4x6)£¬the position and orientation of each robot
    Tr2d = l2T(O,fm,cir);
    for i=1:fm
        % T: HTM
        % x: [xyz rpy] 
        T_f = transl2(xr(1:2))*rotz(xr(3)); 
        T = T_f*Tr2d{i};
        t = T(1:2,3);
        rpy = tr2rpy(T);
        x = [t',rpy(3)];
        xm(i,:) = x;
    end
    for i=1:size(xm,1)
        TransMatrix(:,i) = xm(i,1:3)';
        RotMatrix(:,:,i) = rotz(xm(i,3));
        robot_c(i) = GeoProperties(xm(i,:),P);
        rob_orig_v(:,:,i) = robot_c(i).vertices(1:4,:)';
        % the coordinates of the four robot vertices
        rob_trans_v(:,4*i-3:4*i) = TransMatrix(1:2,i)+RotMatrix(1:2,1:2,i)*rob_orig_v(:,:,i);
    end
    % inequality constraint A*x-b¡Ü0
    for num = 1:size(rob_trans_v,2)
        for j = 1:size(obstacle_pts,3)
            judge = inpolygon(rob_trans_v(1,num),rob_trans_v(2,num),obstacle_pts(1,:,j)',obstacle_pts(2,:,j)');
            for k = 1:size(judge,1)
                if judge(k,1) == true
                    flag = false;
                    return
                end
            end
        end
    end
end
end