function x_init = drawformation(nodez,lc,lm,fm,cir)
hold on
nodenum = size(nodez,1);
for j=1:nodenum
    xc = [nodez(j,1:2),0];
    xr = [nodez(j,1:2),0]; th = 0;
    % obj = GeoProperties(xc,lc);
    % patch('Vertices',obj.verticesStates.position','Faces',obj.faces,'FaceColor','k','FaceAlpha',0.5);
    xm = r2m(xr,th,lc,fm,cir);
    if j == 1
        x_init = [xm;xc];
    end
    % build mobile platform objoid
    for i=1:fm
        plat(i) = GeoProperties(xm(i,:),lm);
        patch('Vertices',plat(i).verticesStates.position','Faces',plat(i).faces,'FaceColor','r');
        object_face(i) = i;
    end
    if cir == 1
       plot(xr(1),xr(2), 'ko', 'MarkerSize',lc(1)*50, 'MarkerFaceColor','k');  
    elseif fm==3 
        patch('Vertices',xm(1:fm,:),'Faces',object_face,'FaceColor','k');
    else
        patch('Vertices',xm(1:fm,:),'Faces',object_face,'FaceColor','k');
    end
end
hold off
end