function [h1,h2] = drawformation_control_ta(pose_matrix,lc,lm,fm,cir,arrival,IND)
import rvctools.*
rob_col = [];
color='mgbk';
for i = 1:length(fm)
    rob_col = [rob_col,repelem(i,fm(i))];
end
hold on
xm = pose_matrix;
for i=1:sum(fm)
    plat(i) = GeoProperties(xm(i,:),lm);
    plot(pose_matrix(i,1),pose_matrix(i,2),'o','Color',color(rob_col(IND(i))),'LineWidth',0.05);
    h1(i) = patch('Vertices',plat(i).verticesStates.position','Faces',plat(i).faces,'FaceColor','r','FaceAlpha',0.5);
    object_face(i) = i;
end
if arrival == 1
    if cir == 1
        h2 = plot(0.5*(xm(1,1)+xm(3,1)),0.5*(xm(2,2)+xm(4,2)), 'ko', 'MarkerSize',lc(1)*50, 'MarkerFaceColor','k');  
    elseif fm==3 
        h2 = patch('Vertices',xm(:,1:fm),'Faces',object_face,'FaceColor','k','FaceAlpha',0.5);
    else
        h2 = patch('Vertices',xm(1:fm,:),'Faces',object_face,'FaceColor','k','FaceAlpha',0.5);
    end
end
hold off
end