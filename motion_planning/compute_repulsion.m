%% Compute the repulsion
function [repulsion,rep_obs_list,ind] = compute_repulsion(robot_pose,obs_pose,detect_R,rest_num,robot_idx,x_init_fm)
% computer the repulsion using artificial potential field method
% obs_pose=[x1 y1;x2; y2;....]
[M,N]=size(obs_pose);
% repulsion(1)=0; %x direction
% repulsion(2)=0; %y direction
for i = 1:M
    dis(i) = norm([robot_pose(1,1)-obs_pose(i,1) robot_pose(1,2)-obs_pose(i,2)]);
end
[~,ind] = min(dis);
% distance=sqrt((robot_pose(1)-obs_pose(1,1))^2+(robot_pose(2)-obs_pose(1,2))^2);
% % if distance<=detect_R/2
% %     repulsion(1)=-2*vx;   
% %     repulsion(2)=-2*vy;   
% % end
% if distance<=detect_R
%     temp=(1/distance-1/detect_R)/(distance^3);
%     % temp = 1/distance-1/detect_R;
% %     repulsion(1)=repulsion(1)+30*temp+30*temp*temp*(robot_pose(1)-obs_pose(ind,1));
% %     repulsion(2)=repulsion(2)+30*temp+30*temp*temp*(robot_pose(2)-obs_pose(ind,2));
%     repulsion(1)=repulsion(1)+temp*(robot_pose(1)-obs_pose(i,1));
%     repulsion(2)=repulsion(2)+temp*(robot_pose(2)-obs_pose(i,2));
% end
x_init_fm = x_init_fm';
M = size(obs_pose,1);
repulsion(1) = 0; %x direction
repulsion(2) = 0; %y direction
%rep_obs_list = zeros(,1)
for i = 1:M
    if abs(robot_pose(1)-x_init_fm(robot_idx,1)) < 0.2 && abs(robot_pose(2)-x_init_fm(robot_idx,2)) < 0.2 ...
            && i < rest_num + 1
        detect_R = 0.1;
    else
        detect_R = 0.2;
    end
    dis = norm([robot_pose(1) - obs_pose(i,1),robot_pose(2) - obs_pose(i,2)]);
    if dis <= detect_R
        rep_obs_list(i,1) = dis;
        temp = (1/dis - 1/detect_R) / (dis^3);
        repulsion(1) = repulsion(1) + temp*(robot_pose(1) - obs_pose(i,1));
        repulsion(2) = repulsion(2) + temp*(robot_pose(2) - obs_pose(i,2));
    else
        rep_obs_list(i,1) = inf;
%     temp = 1/distance-1/detect_R;
%     repulsion(1)=repulsion(1)+temp+temp*temp*(robot_pose(1)-obs_pose(i,1));
%     repulsion(2)=repulsion(2)+temp+temp*temp*(robot_pose(2)-obs_pose(i,2));
    end
end
end
