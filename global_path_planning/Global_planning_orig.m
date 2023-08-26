%% Global planning
function [nodez,nodepoly,edge_zind,noderoute,polyroute,dis_total] = Global_planning_orig(O_x,O_y,P_x,P_y,fm,cir,range,obs,start,goal)
close all
clc
% obstacles cell
obstaclecell = generate_obstacle(obs);
sum = 0;
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
if ~exist('initialization.mat', 'file')
     save('initialization.mat','obs','range','O_x','O_y','P_x','P_y','fm','cir','x_init');
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
while isempty(noderoute)
% while length(find(cell_dis_total==-Inf)) < size(cell_x,1)*size(cell_x,2)*0.9 
    node_num = 1;
    [nodez,nodepoly,nodepoly_zind] = randnode(nodez,nodepoly,nodepoly_zind,node_num,obstaclecell,...
    range,[O_x,O_y],[P_x,P_y],fm,cir);
% create undirected graph and its edges
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
if isempty(noderoute)
    disp("No feasible path is found!");
end
draw_conv_region(nodepoly(polyroute),[],range);
drawformation(nodez(noderoute,:),[O_x,O_y],[P_x,P_y],fm,cir);
hold on
% Visualise the A_star path
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'*-','Color',[1,0,1],'linewidth',3);
hold off
% figure
% plot(ddt,time,'*','Color','b','linewiprofile viewdth',3);
% xlabel('Iterations');
% ylabel('Time/s');
% title('fmincon');
if ~exist('shortestpath.mat', 'file')
    save('shortestpath.mat','nodez','nodepoly','edge_zind','noderoute','polyroute');
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
function [nodez, nodepoly, nodepoly_zind]= randnode(nodez,nodepoly,nodepoly_zind,node_num,obscell,...
    range,O_,P_,fm,cir)
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
% Initialize node z
z_num = size(nodez,1);
poly_num = length(nodepoly);
% generate nodes outside obstacles
count=1; % counter
while (count<=node_num)
    % generate random two number in range of map's border
    rand_coord = [rand* (ub(1)-lb(1)) + lb(1);rand* (ub(2)-lb(2)) + lb(2)];
    % check if this node is not inside any obstacle
    if ~inpolygon(rand_coord(1),rand_coord(2),obscell.poly(1,:),obscell.poly(2,:))
        % chech if inside existing polytopes
        if ~inside_existed_poly(rand_coord,nodepoly)
            % generate ploytope
            randpoly = polytope(obscell.calc,rand_coord,range);
            % check whether formation exists in a polytope
            [randz,fg] = formation(randpoly,rand_coord',range,O_,P_,fm,cir);
            if fg==1
                % add this location to nodelocation list
                nodez = [nodez;randz];
                nodepoly = [nodepoly,randpoly];
                nodepoly_zind{poly_num+count} = z_num+count;
                hold on
                plot(rand_coord(1),rand_coord(2),'go');
                plot(randz(1),randz(2),'r*');
                draw_conv_region(randpoly,[],range);
                drawformation(randz,O_,P_,fm,cir);
                % hold off
                count=count+1;
            end
        end
    end
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