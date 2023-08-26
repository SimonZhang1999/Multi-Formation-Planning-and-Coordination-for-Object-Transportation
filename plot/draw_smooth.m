function [x,y] = draw_smooth(orig_traj,inserted_traj,polys_x,polys_y,ts,r_set)
hold on;
inter_traj = [];
plot(inserted_traj(1,:),inserted_traj(2,:),'.g');hold on;
plot(orig_traj(1,:),orig_traj(2,:),'*r');hold on;
for i=2:size(orig_traj,2)
    p1 = [orig_traj(1,i-1),orig_traj(2,i-1)];
    p2 = [orig_traj(1,i),orig_traj(2,i)];
    n = 50;
    inter_pts = [linspace(p1(1),p2(1),n);linspace(p1(2),p2(2),n)];
    inter_traj = [inter_traj inter_pts(:,2:end)];
end
for i=1:size(inserted_traj,2)-2
    r = r_set(i);
    v1 = inserted_traj(:,i+1)+[-r;-r];
    v2 = inserted_traj(:,i+1)+[-r;r];
    v3 = inserted_traj(:,i+1)+[r;r];
    v4 = inserted_traj(:,i+1)+[r;-r];
    plot([v1(1,1),v2(1,1)],[v1(2,1),v2(2,1)],'color',[0.5 0 0]);
    plot([v2(1,1),v3(1,1)],[v2(2,1),v3(2,1)],'color',[0.5 0 0]);
    plot([v3(1,1),v4(1,1)],[v3(2,1),v4(2,1)],'color',[0.5 0 0]);
    plot([v4(1,1),v1(1,1)],[v4(2,1),v1(2,1)],'color',[0.5 0 0]);
end
x = cal_traj_val(polys_x,ts,0:0.01:5,0);
y = cal_traj_val(polys_y,ts,0:0.01:5,0);
plot(x,y,'*-','color','k','linewidth',3);
plot(x,y,'r');
for i = 1:size(x,2)
    for j = 1:size(inter_traj,2)
        error(i,j) = norm([x(i)-inter_traj(1,j),y(i)-inter_traj(2,j)]);
        error_st(i) = min(error(i,:));
    end
end
save('good.mat','error_st');
t_data = 0:1:size(x,2)-1;
figure
xlabel('time');ylabel('error');
plot(t_data,error_st,'Color','r');
title('Error between the optimized trajectory and the original trajectory');
figure
h1 = plot(inter_traj(1,:),inter_traj(2,:),'*-','Color','b','linewidth',1);
hold on
h2 = plot(x,y,'*-','Color','r','linewidth',1);
legend([h1,h2],'Original trajectory','Optimized trajectory');
end