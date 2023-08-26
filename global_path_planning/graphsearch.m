function [nodez,nodepoly_zind,edge_adjacent,edge_zind]=graphsearch(nodez,nodepoly,nodepoly_zind,edge_zind,rdpoly_num,range,lc,lm,fm,cir)
% count the number of existing polygons
poly_num = length(nodepoly);
% traverse the existing polygons and the newly generated polygons separately
num_new_poly = poly_num-rdpoly_num+1;
num_exist_poly = poly_num-rdpoly_num;
for i = num_new_poly:poly_num % random ploytope Pp
    for j = 1:num_exist_poly % exist ploytope P
        % check if formation exists in the intersection of any two polytopes
        [intersec_z,fg] = formation([nodepoly(i),nodepoly(j)],nodez(nodepoly_zind{i}(1),1:2),range,lc,lm,fm,cir);
        hold on;
        if fg == 1
            plot(intersec_z(1),intersec_z(2),'m*');
%         else
%             plot(intersec_z(1),intersec_z(2),'r*');
            nodez = [nodez;intersec_z]; 
            z_num = size(nodez,1);
            num_P = size(nodepoly_zind{j},2);
            for kj = 1:num_P
                edge_zind = [edge_zind;nodepoly_zind{j}(kj),z_num;];
                edge_zind = [edge_zind;z_num,nodepoly_zind{j}(kj)];
            end
            num_Pp = size(nodepoly_zind{i},2);
            nodepoly_zind{j} = [nodepoly_zind{j},z_num];   
            for ki = 1:length(num_Pp)
                edge_zind = [edge_zind;nodepoly_zind{i}(ki),z_num;];
                edge_zind = [edge_zind;z_num,nodepoly_zind{i}(ki)];
            end
            nodepoly_zind{i} = [nodepoly_zind{i},z_num];
        end
    end
end
node_num = size(nodez,1);
edge_num = size(edge_zind,1);
edge_adjacent=zeros(node_num,node_num);
for i = 1:edge_num
    edge_adjacent(edge_zind(i,1),edge_zind(i,2)) = 1;
    edge_adjacent(edge_zind(i,2),edge_zind(i,1)) = 1;
end
% hold on;
% plot(intersec_z,'r*');
for j = 1:2:size(edge_zind,1)
    point_ind_1 = edge_zind(j,1);
    point_ind_2 = edge_zind(j,2);
    x1 = [nodez(point_ind_1,1);nodez(point_ind_2,1)];
    y1 = [nodez(point_ind_1,2);nodez(point_ind_2,2)]; 
    line(x1,y1,'color','k');
end
% hold off;
end