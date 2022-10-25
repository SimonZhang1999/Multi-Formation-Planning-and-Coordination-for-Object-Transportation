function xm0 = r2m(xr0,th0,lc,fm,cir)
Tr2d = l2T(lc,fm,cir);
xr0(3) = th0;
for i=1:fm
    % T: HTM
    % x: [xyz rpy] 
    T_f = transl2(xr0(1:2))*rotz(xr0(3));  
    T = T_f*Tr2d{i};
    [~,t] = tr2rt(T);
    rpy = tr2rpy(T);
    x = [t',rpy(3)];
    xm0(i,:) = x;
end
end