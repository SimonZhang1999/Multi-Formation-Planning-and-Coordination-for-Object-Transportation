function draw_conv_region(region,obstacles,range)
lb = [range(1,1);range(1,2)];
ub = [range(2,1);range(2,2)];
hold on
obs_num = numel(obstacles);
if isempty(obstacles)
  obs_num = 0;
end
% Draw obstacle interiors
for j = 1:obs_num
  obs = obstacles{j}';
  vertice_num = size(obs, 2);
  if vertice_num > 1
    if vertice_num > 2
      k = convhull(obs(1,:), obs(2,:));
    else
      k = [1,2,1];
    end
    patch(obs(1,k), obs(2,k), 'k', 'FaceColor', [0.5 0.5 0.5], 'LineWidth', 0.1);
  else
    plot(obs(1,:), obs(2,:), 'ko');
  end
end

% Draw obstacle boundaries 
for j = 1:obs_num
  obs = obstacles{j}';
  vertice_num = size(obs, 2);
  if vertice_num > 1
    if vertice_num > 2
      k = convhull(obs(1,:), obs(2,:));
    else
      k = [1,2,1];
    end
    plot(obs(1,k), obs(2,k), 'k', 'LineWidth', 2);
  end
end
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