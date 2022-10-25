function draw_path_ta(pose_x,pose_y,pose_th,obscell,O_x,O_y,P_x,P_y,fm,cir,arrival,IND,nodepoly1,nodepoly2,nodepoly3,...
    x1,y1,x2,y2,x3,y3,obj1,obj2,obj3)
node_num = size(pose_th,2);
range = [0 0;10 10];
% plot(pose_x(end,:),pose_y(end,:),'*-','Color',[0,0,1],'linewidth',3);
draw_obs(obscell,range);
hold on;
o1 = patch('Vertices',obj1,'Faces',[1 2 3 4],'FaceColor',[1,0.5,0]);
o2 = patch('Vertices',obj2,'Faces',[1 2 3],'FaceColor',[1,0.5,0]);
o3 = patch('Vertices',obj3,'Faces',[1 2 3],'FaceColor',[1,0.5,0]);
hold on;
for j=1:node_num
    %draw_obs(obscell,range);
    if j == 340
        delete(o1);
        draw_conv_region(nodepoly1,range);
        plot(x1,y1,'*-','Color','m','linewidth',3);
    end
    hold on
    if j > 340
        h2 = patch('Vertices',[pose_x(9,j),pose_y(9,j);pose_x(7,j),pose_y(7,j);...
            pose_x(8,j),pose_y(8,j);pose_x(1,j),pose_y(1,j)],...
            'Faces',[1 2 3 4],'FaceColor',[1,0.5,0]);
        hold on
    end
    if j == 434
        delete(o2);
        delete(o3);
        draw_conv_region(nodepoly2,range);
        draw_conv_region(nodepoly3,range);
        plot(x2,y2,'*-','Color','g','linewidth',3);
        plot(x3,y3,'*-','Color','b','linewidth',3);
    end
    hold on
    if j > 434
        h3 = patch('Vertices',[pose_x(2,j) pose_y(2,j);pose_x(4,j) pose_y(4,j);pose_x(6,j) pose_y(6,j)],...
            'Faces',[1 2 3],'FaceColor',[1,0.5,0]);
        h4 = patch('Vertices',[pose_x(3,j) pose_y(3,j);pose_x(5,j) pose_y(5,j);pose_x(10,j) pose_y(10,j)],...
            'Faces',[1 2 3],'FaceColor',[1,0.5,0]);
        patch('Vertices',[1.8+0.05,4+0.05;1.8+0.05,4-0.05;...
            1.8-0.05,4-0.05;1.8-0.05,4+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
        patch('Vertices',[5+0.05,3.2+0.05;5+0.05,3.2-0.05;...
            5-0.05,3.2-0.05;5-0.05,3.2+0.05],'Faces',[1 2 3 4],'FaceColor','r','FaceAlpha',0.5);
    end
    for rr = 1:size(pose_x,1)
        pose_matrix(rr,1) = pose_x(rr,j);
        pose_matrix(rr,2) = pose_y(rr,j);
        pose_matrix(rr,3) = pose_th(rr,j);
    end
    if arrival == 1
        [h1,h2] = drawformation_control_ta(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir(rr),arrival,IND);
        pause(0.1);
        if j ~= node_num
            delete(h1);
            delete(h2);
        end
    else
        h1 = drawformation_control_ta(pose_matrix,[O_x,O_y],[P_x,P_y],fm,cir(rr),arrival,IND);
        pause(0.1);
        %if j ~= node_num
           delete(h1);
        %end
        if j > 340
            delete(h2);
        end
        if j > 434
            delete(h3); delete(h4);
        end
    end
end
end
function draw_conv_region(region,range)
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
hold on
for i=1:length(region)
    if iscell(region)
        regionparam = [region{:}];
    else
        regionparam = region;
    end
    A = regionparam(i).A;
    b = regionparam(i).b;
    C = regionparam(i).C;
    d = regionparam(i).d;
    for j = 1:size(A,1)-4
        % a'x = b
        % a(1)*x(1)+a(2)x(2)=b
        % set x(1) = 0
        % x(2) = b / a(2)
        ai = A(j,:);
        bi = b(j);
        if ai(2) == 0
            x0 = [bi/ai(1); 0];
        else
            x0 = [0; bi/ai(2)];
        end
        u = [0,-1;1,0] * ai';
        pts = [x0 - 1000*u, x0 + 1000*u];
        plot(pts(1,:), pts(2,:), 'g--', 'LineWidth', 1.5);
    end
    if ~isempty(A)
        V = lcon2vert(A, b);
        k = convhull(V(:,1), V(:,2));
        plot(V(k,1), V(k,2), 'bo-', 'LineWidth', 2);
    end
    th = linspace(0,2*pi,100);
    y = [cos(th);sin(th)];
    x = bsxfun(@plus, C*y, d);
    plot(x(1,:), x(2,:), 'r-', 'LineWidth', 2);
end
plot([lb(1),ub(1),ub(1),lb(1),lb(1)], [lb(2),lb(2),ub(2),ub(2),lb(2)], 'k-');
pad = (ub - lb) * 0.05;
xlim([lb(1)-pad(1),ub(1)+pad(1)])
ylim([lb(2)-pad(2),ub(2)+pad(2)])
axis on
end