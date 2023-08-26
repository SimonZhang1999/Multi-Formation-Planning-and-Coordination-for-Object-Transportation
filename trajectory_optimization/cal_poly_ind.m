%% Find the related polygons
function bound_poly = cal_poly_ind(nodepoly_zind,noderoute)
ind = 0;
bound_poly = [];
for i = 1:length(noderoute)
    for j = 1:length(nodepoly_zind)
        if ~isempty(find(noderoute(i)==nodepoly_zind{1,j})) && isempty(find(j==bound_poly))
            bound_poly(ind + 1) = j;
            ind = ind + 1;
        end
    end
end
end