function [z,flag] = formation(polytope,z_target,range,lc,lm,fm,cir)
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];  
polynum = length(polytope);
poly_A = []; 
poly_b = [];    
% get the A and b from polytope                
for i=1:polynum
    poly_exist_A = polytope(i).A;
    poly_A = [poly_A;poly_exist_A];
    poly_exist_b = polytope(i).b;
    poly_b = [poly_b;poly_exist_b];
end
% calculate z
% find the minimum of nonlinear function under constraint
% t0 = cputime;
[z,~,flag] = fmincon(@(z) (z-z_target)*(z-z_target)',z_target,...
    [],[],[],[],lb,ub,@(z) addcon(z,poly_A,poly_b,lc,lm,fm,cir));
% t1 = cputime;
% dt = t1-t0;
end
%% Additional linear constraints
function [c,ceq] = addcon(z,A,b,lc,lm,fm,cir)
import rvctools.*
xr = [z(1:2),0]; 
% xm(4x6)£¬the position and orientation of each robot
Tr2d = l2T(lc,fm,cir);
for i=1:fm
    T_f = transl2(xr(1:2))*rotz(xr(3)); 
    T = T_f*Tr2d{i};
    t = T(1:2,3);
    rpy = tr2rpy(T);
    x = [t',rpy(3)];
    xm(i,:) = x;
end
for i = 1:size(xm,1)
    TransMatrix(:,i) = xm(i,1:3)';
    RotMatrix(:,:,i) = rotz(xm(i,3));
    robot_c(i) = GeoProperties(xm(i,:),lm);  
    rob_orig_v(:,:,i) = robot_c(i).vertices(1:4,:)';   
    % the coordinates of the four robot vertices
    rob_trans_v(:,4*i-3:4*i) = TransMatrix(1:2,i)+RotMatrix(1:2,1:2,i)*rob_orig_v(:,:,i);
end
% inequality constraint A*x-b¡Ü0
ceq = 0;
for i = 1:size(rob_trans_v,2)
    x_con = rob_trans_v(1:2,i);
    c(:,i) = A*x_con - b;
end
end