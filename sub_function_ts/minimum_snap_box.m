function [smooth_waypts,polys_x,polys_y,ts,r_set] = minimum_snap_box(waypts,nodepoly_zind,noderoute,nodepoly)
% kinematic parameters
v_start = 0;
a_start = 0;
v_end = 0;
a_end = 0;
bound_poly = cal_poly_ind(nodepoly_zind,noderoute);
% guiding weight
lambda = 1000000; 
step = 0.2;
% minimum snap parameters
T = 5;
n_order = 5;
% interpolate the middle points
smooth_waypts = waypts(:,1);
for i=2:size(waypts,2)
    p1 = [waypts(1,i-1),waypts(2,i-1)];
    p2 = [waypts(1,i),waypts(2,i)];
    n = ceil(hypot(p1(1)-p2(1),p1(2)-p2(2))/step)+1;
    inter_pts = [linspace(p1(1),p2(1),n);linspace(p1(2),p2(2),n)];
    smooth_waypts = [smooth_waypts inter_pts(:,2:end)];
end
ts = cal_time_stamp(smooth_waypts,T);
%tic
[polys_x,~] = cal_optimized_traj(smooth_waypts,smooth_waypts(1,:),ts,n_order,...
    v_start,a_start,v_end,a_end,lambda,bound_poly,nodepoly);
[polys_y,r_set] = cal_optimized_traj(smooth_waypts,smooth_waypts(2,:),ts,n_order,...
    v_start,a_start,v_end,a_end,lambda,bound_poly,nodepoly);
%toc
end