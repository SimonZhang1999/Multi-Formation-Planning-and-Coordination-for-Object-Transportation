%% Calculate the radius of the corrider
function min_r = cal_r(bound_poly,nodepoly,point_x,point_y)
for i = 1:length(bound_poly)
    for j = 1:length(nodepoly(bound_poly(i)).A)
        min_dis(i) = cal_dis_min(nodepoly(bound_poly(i)).A,nodepoly(bound_poly(i)).b,point_x,point_y);
    end
end
min_r = sqrt(2)*(min(min_dis))/2-0.15;
if min_r <= 0
    min_r = 0.001;
end
end
function dis = cal_dis_min(A,b,point_x,point_y)
for ii = 1:length(A)
    dis_poly(ii) = abs((A(ii,1)*point_x+A(ii,2)*point_y-b(ii))/(sqrt((A(ii,1))^2+(A(ii,2))^2)));
end
dis = min(dis_poly);
end
        