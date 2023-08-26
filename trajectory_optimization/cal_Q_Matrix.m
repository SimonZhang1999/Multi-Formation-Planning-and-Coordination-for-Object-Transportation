%% Compute the Q Matrix
% n:polynormial order, r:derivertive order
% ts:start timestamp for polynormial, te:end timestap for polynormial
function Q = cal_Q_Matrix(n,r,ts,te)
Q = zeros(n);
T = zeros((n-r)*2+1,1);
for i = 1:(n-r)*2+1
    T(i) = te^i-ts^i;
end
for i = r+1:n+1
    for j = i:n+1
        Q(i,j) = prod(i-r:i-1)*prod(j-r:j-1)/(i+j-2*r-1)*T(i+j-2*r-1);
        % Q is symmetric matrices
        Q(j,i) = Q(i,j);
    end
end