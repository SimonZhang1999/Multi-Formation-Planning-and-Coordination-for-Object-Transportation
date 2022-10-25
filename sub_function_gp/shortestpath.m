function [noderoute,poly_route,dis_total] = shortestpath(nodez,nodepoly_zind,edge_adjacent,nodestart,nodegoal)
num_edge = size(edge_adjacent,1);
Cost = inf(num_edge,1);
current = nodestart;
Route = cell(num_edge,1);
Cost(current) = 0;
close = [current];
c_route = [current];
for i=1:num_edge
    hcost(i,1) = normby(nodez(i,:) - nodez(nodegoal,:),1);
end
i=1;
while i < num_edge && current ~= nodegoal    
    i = i+1;
    T = 1:num_edge;
    T(close)=[];  
    % find all adjacent nodes from current node and calculate the cost
    ind = find(edge_adjacent(current,T)==1);
    cost = normby(nodez(T(ind),:) - nodez(current,:),1) + Cost(current);
    % find if calculated cost is less than the way before explored then update
    temp = find(Cost(T(ind))>cost); Cost(T(ind(temp)))=cost(temp);  
    for j = 1:length(temp)
        Route{T(ind(temp(j)))}=[c_route T(ind(temp(j)))];        
    end  
    % find minimum of normal cost plus heuristic cost of possible node
    [min_val min_ind] = min(Cost(T)+hcost(T));
    if isinf(min_val)
        i = num_edge;
    end
    % set minimum total cost's node as current node
    current = T(min_ind);    
    c_route = Route{current};
    % add current node to close list 
    close = [close current];
end
if current ==nodegoal
    noderoute = c_route;
else % i not means there is no way
    noderoute = [];
end
poly_route = [];
for i=1:length(noderoute)
    for j=1:length(nodepoly_zind)
        if ~isempty(find(nodepoly_zind{j} == noderoute(i), 1))
            poly_route = [poly_route,j];
        end
    end
end
if ~isempty(noderoute)
    node_route = nodez(noderoute,:);
    init_point = nodez(1,1:2);
    via_point = [init_point;node_route(2:size(node_route,1),1:2)];
    dis_total = 0;
    for i = 1:length(via_point)-1
        dis_total = dis_total + norm([via_point(i,1)-via_point(i+1,1),via_point(i,2)-via_point(i+1,2)]);
    end
else
    dis_total = [];
end