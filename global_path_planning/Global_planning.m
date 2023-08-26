%% Global planning
function [nodez,nodepoly,edge_zind,noderoute,nodepoly_zind,polyroute,occupied_area,dis_total] = Global_planning(O_x,O_y,P_x,P_y,fm,cir,range,obs,start,goal)
close all
clc
% obstacles cell
obstaclecell = generate_obstacle(obs);
[cell_x,cell_y,cell_dis] = initial_guess(obs,obstaclecell,range);
sum = 0;
for i = 1:size(cell_x,1)
    for j = 1:size(cell_x,2)
        if cell_dis(i,j)~= -Inf
            sum = sum + cell_dis(i,j);
        end
    end
end
cell_dis = cell_dis / sum;
%% initialize an empty graph
nodez = []; % z vector in row
nodepoly = []; % polytope struct
nodepoly_zind = {}; % z index
% initialize edges
edge_zind = []; % z index
%% Calculate z
z0 = start;
[~,xc0,xm0]= c2v(z0,[O_x,O_y],fm,cir);
[nodez, nodepoly, nodepoly_zind]= SGNode(nodez,nodepoly,nodepoly_zind,[start;goal],obstaclecell,range,[O_x,O_y],[P_x,P_y],fm,cir);
%% Visualization
figure;
draw_conv_region(nodepoly,obstaclecell.vert,range);
x_init = drawformation(nodez,[O_x,O_y],[P_x,P_y],fm,cir);
grid = 0.05:0.1:9.95;
for i = 1:100
    hold on;
    plot(ones(1,100)*grid(i),grid,'Color','b');
end
for i = 1:100
    hold on;
    plot(grid,ones(1,100)*grid(i),'Color','b');
end
if ~exist('initialization3.mat', 'file')
     save('initialization3.mat','obs','range','O_x','O_y','P_x','P_y','fm','cir','x_init');
end
%% add goal node to graph
[nodez,nodepoly_zind,edge_adjacent,edge_zind]=graphsearch(nodez,nodepoly,nodepoly_zind,edge_zind,1,range,[O_x,O_y],[P_x,P_y],fm,cir);
[noderoute,polyroute,dis_total] = shortestpath(nodez,nodepoly_zind,edge_adjacent,1,2);
%% generate random nodes
counter = 0;
cell_dis_total = [];
route_dis = [];
rand_coord_list = [];
route_ind = 1;
%while isempty(noderoute)
while length(find(cell_dis_total==-Inf)) < size(cell_x,1)*size(cell_x,2)*0.98
    node_num = 1;
    [nodez,nodepoly,nodepoly_zind,cell_dis_total,rand_coord_list,invalid] = randnode(nodez,nodepoly,nodepoly_zind,node_num,obstaclecell,...
        cell_x,cell_y,cell_dis,cell_dis_total,rand_coord_list,range,[O_x,O_y],[P_x,P_y],fm,cir);
    % create undirected graph and its edges
    if ~invalid
        [nodez,nodepoly_zind,edge_adjacent,edge_zind]=graphsearch(nodez,nodepoly,nodepoly_zind,edge_zind,node_num,range,[O_x,O_y],[P_x,P_y],fm,cir);
        [noderoute,polyroute,dis_total] = shortestpath(nodez,nodepoly_zind,edge_adjacent,1,2);
        if ~isempty(noderoute)
            route_dis(route_ind) = dis_total;
            if dis_total ~= route_dis(route_ind)
                route_ind = route_ind + 1;
            end
        end
        counter = counter + 1;
        draw_conv_region(nodepoly(polyroute),[],range);
        drawformation(nodez(noderoute,:),[O_x,O_y],[P_x,P_y],fm,cir);
    end
end
if isempty(noderoute)
    disp("No feasible path is found!");
end
occupied_area = length(find(cell_dis_total==-Inf))/(size(cell_x,1)*size(cell_x,2));
draw_conv_region(nodepoly(polyroute),[],range);
drawformation(nodez(noderoute,:),[O_x,O_y],[P_x,P_y],fm,cir);
hold on
% Visualise the A_star path
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'*-','Color',[1,0,1],'linewidth',3);
[new_waypts,polys_x,polys_y,ts,r_set] = minimum_snap_box(nodez(noderoute,:)',nodepoly_zind,noderoute,nodepoly);
[x,y] = draw_smooth(nodez(noderoute,:)',new_waypts,polys_x,polys_y,ts,r_set);
hold off
% figure
% plot(ddt,time,'*','Color','b','linewiprofile viewdth',3);
% xlabel('Iterations');
% ylabel('Time/s');
% title('fmincon');
if ~exist('smooth2.mat', 'file')
    % save('shortestpath.mat','nodez','nodepoly','edge_zind','noderoute','polyroute','x','y');
    save('smooth2.mat','nodepoly_zind','polyroute','nodepoly','range','nodez','noderoute','O_x','O_y','P_x','P_y');
end
end

%% Generate obstacles
function obscell = generate_obstacle(obstacle)
obs_num=length(obstacle);
obs_vert_point = [];
for i=1:obs_num
    obs_vert_point(1:2,1:length(obstacle(1,i).verticesStates.position),i) = obstacle(i).verticesStates.position(1:2,1:length(obstacle(1,i).verticesStates.position));
end
% reset obstacles vertices in its convexhull 
obs_vert_ind = cell(1,size(obs_vert_point,3));
obscell.vert = cell(1,size(obs_vert_point,3));
for j = 1:obs_num
    obs = obs_vert_point(:,:,j);
    if size(obs, 2) > 1
        if size(obs, 2) > 2
            % return the clockwise index of the convexhull
            obs_vert_ind{j} = convhull(obs(1,:), obs(2,:));
        else
            obs_vert_ind{j} = [1,2,1];
        end
    end
    obscell.vert{j} = obs(:,obs_vert_ind{j})';
    obscell.calc{j} = obs(:,obs_vert_ind{j});
end
% obstacles cell
obscell.poly = obscell.vert{1}';
for i=2:size(obscell.vert,1)
    obscell.poly = [obscell.poly,NaN(2,1),obscell.vert{i}'];
end
end

%% Generate initial and goal nodes
function [nodez, nodepoly, nodepoly_zind]= SGNode(nodez,nodepoly,nodepoly_zind,randnode,obscell,range,O,P,fm,cir)
% set all nodez to 0
z_num = size(nodez,1);
poly_num = length(nodepoly);
% generate nodes outside obstacles
rand_num = size(randnode,1);
for i=1:rand_num
    % generate ploytope, return A,b,C,d
    randpoly = polytope(obscell.calc,randnode(i,:)',range);
    if ~config_in_obs(randnode,obscell.calc,O,P,fm,cir)
        error('Start or goal point is inside the obstacles!');
    end
    % check whether formation exists in a polytope
    [randz,fg] = formation(randpoly,randnode(i,:),range,O,P,fm,cir);
    if fg==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodepoly_zind{poly_num+i} = z_num+i;
        hold on
        plot(randz(1),randz(2),'r*');
        draw_conv_region(randpoly,[],range);
        drawformation(randz,O,P,fm,cir);
    end
end
end

%% Generate nodes in a heuristic way
function [nodez, nodepoly, nodepoly_zind,cell_dis_total,rand_coord_list,invalid]= randnode(nodez,nodepoly,nodepoly_zind,node_num,obscell,...
    cell_x,cell_y,cell_obs_dis,cell_dis_total,rand_coord_list,range,O_,P_,fm,cir)
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
% Initialize node z
z_num = size(nodez,1);
poly_num = length(nodepoly);
invalid = false;
% initialise the polygen-distance matrix
for i = 1:size(cell_x,1)
    for j = 1:size(cell_x,2)
        dis = 0;
        for k = 1:2
            dis = dis + norm([cell_x(i,j)-nodepoly(k).d(1),cell_y(i,j)-nodepoly(k).d(2)]);
        end
        cell_poly_dis(i,j) = dis;
    end
end
if isempty(cell_dis_total)
    cell_dis_total = zeros(size(cell_x,1),size(cell_x,1));
end
for i = 1:size(cell_x,1)
    for j = 1:size(cell_x,2)
        if inside_existed_poly([cell_x(i,j);cell_y(i,j)],nodepoly(1)) ||...
                inside_existed_poly([cell_x(i,j);cell_y(i,j)],nodepoly(2))
            cell_dis_total(i,j) = -Inf;
        end
    end
end
cell_poly_dis_norm = Norm(cell_poly_dis);
% generate nodes outside obstacles
count=1; % counter
while (count <= node_num)
%% Heuristic Sampling Start
    %if
    % update the polygen-distance matrix
    if length(nodepoly) > 2
        for i = 1:size(cell_x,1)
            for j = 1:size(cell_x,2)
                cell_poly_dis(i,j) = 0;
                for k = 1:poly_num
                    cell_poly_dis(i,j) = cell_poly_dis(i,j) + norm([cell_x(i,j)-nodepoly(poly_num).d(1),cell_y(i,j)-nodepoly(poly_num).d(2)]);      
                end
            end
        end
        cell_poly_dis_norm = Norm(cell_poly_dis);
    end
%     for ii = 1:size(cell_x,1)
%         for jj = 1:size(cell_x,2)
%             if inside_existed_poly([cell_x(ii,jj);cell_y(ii,jj)],nodepoly)
%                 cell_dis_total(ii,jj) = -Inf;
%             end
%         end
%     end
% calculate the evaluation matrix
    for i = 1:size(cell_x,1)
        for j = 1:size(cell_x,2)
            if cell_dis_total(i,j) ~= -Inf
                cell_dis_total(i,j) = 0.2*cell_obs_dis(i,j) + 0.8*cell_poly_dis_norm(i,j);
            end
        end
    end
    % choose the best sampling node
    for i = 1:size(cell_dis_total,1)*size(cell_dis_total,2)
        [x_ind y_ind] = find(cell_dis_total==max(max(cell_dis_total)));
        rand_coord = [cell_x(x_ind,y_ind);cell_y(x_ind,y_ind)];  
        % chech if inside existing polytopes
        if inside_existed_poly(rand_coord,nodepoly)
            cell_dis_total(x_ind,y_ind) = -Inf;
        else
            break;
        end
    end
    if isempty(rand_coord_list)
        rand_coord_list = [rand_coord_list;[x_ind y_ind]];
    else
        % if the new node is not far away from the existing nodes, discard
        % it
        exist_rand_coord = false;
        for rand_coord_ind = 1:size(rand_coord_list,1)
            if norm([rand_coord_list(rand_coord_ind,1)-x_ind,... 
                    rand_coord_list(rand_coord_ind,2)-y_ind]) < 5
                exist_rand_coord = true;
                cell_dis_total(x_ind,y_ind) = -Inf;
                break;
            end
        end
        if ~exist_rand_coord
            rand_coord_list = [rand_coord_list;[x_ind y_ind]];   
        else
            invalid = true;
            return
        end
    end
%     else
%         warning('The new node is inside existing polytopes!');
%    end
%     check if this node is not inside any obstacle
%     flag = true;
%     for obs_num = 1:length(obscell.calc)
%         if inpolygon(rand_coord(1),rand_coord(2),obscell.calc{1,obs_num}(1,:),obscell.calc{1,obs_num}(2,:))
%             flag = false;
%             break;
%         end
%     end
%     if flag
        % generate the ploytope of new node
        randpoly = polytope(obscell.calc,rand_coord,range);
        exist_poly = false;
        for node_ind = 1:length(nodepoly)
            if norm([nodepoly(node_ind).d(1)-randpoly.d(1),...
                    nodepoly(node_ind).d(2)-randpoly.d(2)]) < cell_x(1,1)*20
                cell_dis_total(x_ind,y_ind) = -Inf;
                invalid = true;
                return
            end
        end
        if ~exist_poly    
            % check whether formation exists in the new polytope
            % calculate the configuration of the new node
            [rand_coord,fg1] = formation(randpoly,rand_coord',range,O_,P_,fm,cir);
            % calculate the configuration which minimize the distance to
            % the goal point
            [randz,fg2] = formation(randpoly,nodez(2,:),range,O_,P_,fm,cir);
            if fg1 || fg2 
                % add this location to nodelocation list
                nodepoly = [nodepoly,randpoly];
                warning('New polygen is generated!');
                if fg1
                    nodez = [nodez;rand_coord];     
                    nodepoly_zind{poly_num+1} = z_num + count;
                    count = count + 1;
                end
                if fg2
                    nodez = [nodez;randz];               
                    nodepoly_zind{poly_num+1} = z_num + count;
                end
                for ii = 1:size(cell_x,1)
                    for jj = 1:size(cell_x,2)
                        if inside_existed_poly([cell_x(ii,jj);cell_y(ii,jj)],nodepoly)
                            cell_dis_total(ii,jj) = -Inf;
                        end
                    end
                end
%% Heuristic Sampling End
                hold on
                % plot generated node
                plot(rand_coord(1),rand_coord(2),'r*');
                % plot optimal node
                plot(randz(1),randz(2),'b*');
                draw_conv_region(randpoly,[],range);
                drawformation(randz,O_,P_,fm,cir);
                % hold off
            else
                warning('No formation exists in the new polytope!');
            end
%     else
%         warning('The new node is inside the obstacles!');
%     end
        end
    count=count+1;
end
end

%% Check if the configuration is in existing polytopes
function flag = inside_existed_poly(rand_coord,nodepoly)
poly_num = length(nodepoly);
for i=1:poly_num
    if nodepoly(i).A*rand_coord<=nodepoly(i).b
        flag = true;
        return
    end
end
flag = false;
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
        T_f = transl2(xr(1:2))*rotz(xr(3)); 
        T = T_f*Tr2d{i};
        t = T(1:2,3);
        rpy = tr2rpy(T);
        x = [t',rpy(3)];
        xm(i,:) = x;
    end
    for i=1:size(xm,1)
        robot_c(i) = GeoProperties(xm(i,:),P);
        rob_orig_v(:,:,i) = robot_c(i).vertices(1:4,:)';
        RotMatrix(:,:,i) = rotz(xm(i,3));
        TransMatrix(:,i) = xm(i,1:3)';
        % the coordinates of the four robot vertices
        rob_trans_v(:,4*i-3:4*i) = TransMatrix(1:2,i)+RotMatrix(1:2,1:2,i)*rob_orig_v(:,:,i);
    end
    % inequality constraint A*x-b¡Ü0
    for num = 1:size(rob_trans_v,2)
        for j = 1:size(obstacle_pts,3)
            judge = inpolygon(rob_trans_v(1,num),rob_trans_v(2,num),obstacle_pts{1,j}(1,:)',obstacle_pts{1,j}(2,:)');
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
%% Determining polygon coefficient
function convex_region_coef = polytope(obstacle_pts,path_pts,range)
ptnum = size(path_pts,2);
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
convex_region_coef = struct;
A_boundary = [-1,0;0,-1;1,0;0,1];
b_boundary = [-lb; ub];
for i=1:ptnum
    [A,b,C,d] = Computing_LargeConvexRegions(obstacle_pts, A_boundary, b_boundary, path_pts(:,i));
    convex_region_coef(i).A = A;
    convex_region_coef(i).b = b;
    convex_region_coef(i).C = C;
    convex_region_coef(i).d = d;
end
end
%% Normalization
function cell_norm = Norm(cell_matrix)
dis_sum = 0;
for i = 1:size(cell_matrix,1)
    for j = 1:size(cell_matrix,2)
        if cell_matrix(i,j)~= -Inf
            dis_sum = dis_sum + cell_matrix(i,j);
        end
    end
end
cell_norm = cell_matrix / dis_sum;
end