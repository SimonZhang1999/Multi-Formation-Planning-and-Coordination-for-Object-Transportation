function draw_path(pose_x,pose_y,pose_th,obs,O_x,O_y,P_x,P_y,fm,cir,arrival,dyobs,~)
obstcell = createobstcell(obs);
hold on
node_num = size(pose_th,2);
% plot(pose_x(end,:),pose_y(end,:),'*-','Color',[0,0,1],'linewidth',3);
for j=1:node_num
    for rr = 1:fm+1
        pose_matrix(rr,1) = pose_x(rr,j);
        pose_matrix(rr,2) = pose_y(rr,j);
        pose_matrix(rr,3) = pose_th(rr,j);
    end
    if arrival == 1
        [h1,h2] = drawformation_control(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir,arrival);
        h3 = patch('Vertices',[dyobs(j,1)+0.05,dyobs(j,2)+0.05;dyobs(j,1)+0.05,dyobs(j,2)-0.05;...
            dyobs(j,1)-0.05,dyobs(j,2)-0.05;dyobs(j,1)-0.05,dyobs(j,2)+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
        pause(0.1);
        delete(h1);
        delete(h2);
        delete(h3);
%         delete(h3);
%         if ~isempty(h4)
%             delete(h4);
%         end
%         delete(h5);
%         delete(h6)
    else
        h1 = drawformation_control(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir,arrival);
        pause(0.1);
        delete(h1);
    end
end
end
