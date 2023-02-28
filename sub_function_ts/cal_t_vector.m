%% Compute time vector
%% n=0:position n=1:velocity n=2:acceleration n=3:jerk
function t_vec = cal_t_vector(t,n_order,n)
    t_vec = zeros(1,n_order+1);
    for i= n+1:n_order+1
        t_vec(i) = prod(i-n:i-1)*t^(i-n-1);
    end
end