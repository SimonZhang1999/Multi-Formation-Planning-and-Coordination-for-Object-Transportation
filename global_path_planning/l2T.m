function Tr2d = l2T(lc,fm,cir)
if cir == 1
    r = lc(1);
    Tr2d{1} = transl2([-r*2/3,0])*rotz(pi);
    Tr2d{2} = transl2([0,-r*2/3]);
    Tr2d{3} = transl2([r*2/3,0]);
    Tr2d{4} = transl2([0,r*2/3])*rotz(pi);
else
    switch fm
        case 4
            lcx = 2*lc(1); lcy = 2*lc(2);
            Tr2d{1} = transl2([-lcx/2,0])*rotz(pi);
            Tr2d{2} = transl2([0,-lcy/2])*rotz(-pi/2);
            Tr2d{3} = transl2([lcx/2,0]);
            Tr2d{4} = transl2([0,lcy/2])*rotz(pi/2);
        case 3
            lcx = lc(1);
            Tr2d{1} = transl2([0,lcx*2/3])*rotz(pi/2);
            Tr2d{2} = transl2([-lcx/2,-lcx*sqrt(3)/6-lcx/2])*rotz(pi/3);
            Tr2d{3} = transl2([lcx/2,-lcx*sqrt(3)/6-lcx/2])*rotz(-pi/3);    
        case 2
            lcx = lc(1);
            Tr2d{1} = transl2([-lcx/2,0])*rotz(pi);
            Tr2d{2} = transl2([lcx/2,0]);
    end
end
end