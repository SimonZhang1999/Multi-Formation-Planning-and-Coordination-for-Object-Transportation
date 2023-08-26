%function [h1,h2,h3,h4] = drawformation_control(pose_matrix,lc,lm,fm,cir,arrival,traj,pose_th,ind)
function [h1,h2] = drawformation_control(pose_matrix,lc,lm,fm,cir,arrival)
import rvctools.*
color='mgbk';
hold on
xm = pose_matrix;
% build mobile platform cuboid
for i=1:fm
    plat(i) = GeoProperties(xm(i,:),lm);
    plot(pose_matrix(i,1),pose_matrix(i,2),'o','Color',color(i),'LineWidth',0.05);
    h1(i) = patch('Vertices',plat(i).verticesStates.position','Faces',plat(i).faces,'FaceColor','r','FaceAlpha',0.5);
    object_face(i) = i;
end
if arrival == 1
    if cir == 1
        h2 = plot(0.5*(xm(1,1)+xm(3,1)),0.5*(xm(2,2)+xm(4,2)), 'ko', 'MarkerSize',lc(1)*50, 'MarkerFaceColor','k');  
    elseif fm==3 
        h2 = patch('Vertices',xm(:,1:fm),'Faces',object_face,'FaceColor','k','FaceAlpha',0.5);
    else
        h2 = patch('Vertices',xm(1:fm,:),'Faces',[1 2 4 3],'FaceColor','k','FaceAlpha',0.5);
    end
%     r = 0.2; 
%     theta = 0:pi/20:2*pi;
    %for id=1:length(dynamic_obs)
%         x1 = r * cos(theta) + dynamic_obs(1,1)-0.2; 
%         y1 = r  *sin(theta) + dynamic_obs(1,2)-0.5;
%         x2 = r * cos(theta) + dynamic_obs(1,3)+0.5; 
%         y2 = r  *sin(theta) + dynamic_obs(1,4)-0.5;
%         h5(1) = plot(x1,y1,'-m'); 
%         h5(2) = plot(x2,y2,'-m'); 
%         h6(1) = fill(x1,y1,'m');
%         h6(2) = fill(x2,y2,'m');
    %end
%     centre = cal_centre(pose_matrix,fm,cir);
%     error = [centre(1)-pose_matrix(end,1),centre(2)-pose_matrix(end,2)];
%     ArrowLength = 0.5;
%     h3 = quiver(xm(fm+1,1)+error(1), xm(fm+1,2)+error(2), ArrowLength*cos(pose_th(end,ind)), ArrowLength*sin(pose_th(end,ind)),'ok');
%     if ind > 2 && ind <= length(traj)%轨迹非空
%         for it=1:size(traj{1,ind-2},1)/5    
%             index = 1+(it-1)*5; 
%             h4(it) = plot(traj{1,ind-2}(index,:)+error(1),traj{1,ind-2}(index+1,:)+error(2),'Color',[0.5 0 0]);%根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
%         end
%     else
%         h4 = [];
%     end
end
hold off
end
function centre = cal_centre(pose_matrix,fm,cir)
x_min = min(pose_matrix(1:fm,1));
x_max = max(pose_matrix(1:fm,1));
y_min = min(pose_matrix(1:fm,2));
y_max = max(pose_matrix(1:fm,2));
if cir == 1 || fm ==4
    centre = [(x_min+x_max)/2 (y_min+y_max)/2];
elseif fm == 3
    centre = [(x_min+x_max)/2 y_min+(y_max-y_min)/3];
end
end