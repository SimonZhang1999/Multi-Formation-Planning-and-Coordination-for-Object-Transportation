function [polys,r_set] = cal_optimized_traj(points,waypts,ts,n_order,...
    v_start,a_start,v_end,a_end,lambda,bound_poly,nodepoly)
% initialize the parameters of the quadratic programming problem
b_constraints = [];
H_constraints = [];
% compute Q
Q_final = [];
p_start = waypts(1);
p_end = waypts(end);
num_poly = length(waypts)-1;
n_coef = n_order+1;
for i = 1:num_poly
    Q_final = blkdiag(Q_final,cal_Q_Matrix(n_order,3,ts(i),ts(i+1)));
end
b_final = zeros(size(Q_final,1),1);
% Bounding box initialization
tic
for i = 1:num_poly
    ci = zeros(n_coef,1);
    a_temp = (waypts(i+1)-waypts(i))/(ts(i+1)-ts(i));
    a_start = waypts(i)-a_temp*ts(i);
    % constrain polynomial curve
    ci(1:2,1) = [a_start;a_temp];  
    Qi = cal_Q_Matrix(n_order,0,ts(i),ts(i+1));
    bi = -Qi'*ci;
    b_constraints = [b_constraints;bi];
    H_constraints = blkdiag(H_constraints,Qi);   
end
toc
Q_final = Q_final + lambda*H_constraints;
b_final = b_final + lambda*b_constraints;
Aeq = zeros(3*num_poly+3,n_coef*num_poly);
beq = zeros(3*num_poly+3,1);
% 6 equations constraints
% start point and goal point equality constraints (postion, velocity, acceleration)  
num_s = 3;
num_e = 3;
Aeq(1:num_s,1:n_coef) = [
    cal_t_vector(ts(1),n_order,0);
    cal_t_vector(ts(1),n_order,1);
    cal_t_vector(ts(1),n_order,2)];
Aeq(num_e+1:num_s+num_e,n_coef*(num_poly-1)+1:n_coef*num_poly) = [
    cal_t_vector(ts(end),n_order,0);
    cal_t_vector(ts(end),n_order,1);
    cal_t_vector(ts(end),n_order,2)];
beq(1:num_s+num_e,1) = [p_start,v_start,a_start,p_end,v_end,a_end]';
neq = 6;
% continuous equality constraints  
% (num_poly-1)*3 equality constraints in total
for i=1:num_poly-1
    tvec_p = cal_t_vector(ts(i+1),n_order,0);
    tvec_v = cal_t_vector(ts(i+1),n_order,1);
    tvec_a = cal_t_vector(ts(i+1),n_order,2);
    neq = neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1)) = [tvec_p,-tvec_p];
    neq = neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1)) = [tvec_v,-tvec_v];
    neq = neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1)) = [tvec_a,-tvec_a];
end
% n_ploy-1 inequality constraints in total
Aieq = zeros(2*(num_poly-1),n_coef*num_poly);
bieq = zeros(2*(num_poly-1),1);
% Bounding box generation
tic
for i = 1:num_poly-1
    r = cal_r(bound_poly,nodepoly,points(1,i),points(2,i));
    r_set(i) = r;
    tvec_p = cal_t_vector(ts(i+1),n_order,0);
    % introduce inequality constraints
    Aieq(2*i-1:2*i,n_coef*i+1:n_coef*(i+1)) = [tvec_p;-tvec_p];
    bieq(2*i-1:2*i) = [waypts(i+1)+r r-waypts(i+1)];
end
toc
% solve quadratic programming problem
%p = quadprog(Q_final,b_final,Aieq,bieq,Aeq,beq);
% Trajectory generation
tic
p = quadprog(Q_final,b_final,[],[],Aeq,beq);
toc
polys = reshape(p,n_coef,num_poly);
end