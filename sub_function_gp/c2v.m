function [xr,xc,xm] = c2v(z,lc,fm,cir)
if length(z)==12
    xr = [z(1:2),0,0,0,z(3)]; 
else   
    xr = [z(1:2),0]; 
end
Tr2c0 = transl2([0,0]);
T_f = transl2(xr(1:2))*rotz(xr(3)); 
T_c = T_f*Tr2c0;
[~,t] = tr2rt(T_c);
rpy = tr2rpy(T_c);
xc = [t',rpy(3)];
Tr2d = l2T(lc,fm,cir);
for i=1:fm
    % T: Homogeneous Transformation Matrix
    % x: [xyz rpy] 
    T = T_f*Tr2d{i};
    [~,t] = tr2rt(T);
    rpy = tr2rpy(T_c);
    x = [t',rpy(3)];
    xm(i,:) = x;
end
end