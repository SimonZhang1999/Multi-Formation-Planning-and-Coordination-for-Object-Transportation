function vals = cal_traj_val(polys,ts,time_step,r)
idx = 1;
vals = zeros(1,length(time_step));
t_error = 0.0001;
for i = 1:length(time_step)
    if time_step(i) < ts(idx)
        vals(i) = 0;
    else
        while idx < length(ts) &&...
                time_step(i) > ts(idx+1) + t_error
            idx = idx+1;
        end
        vals(i) = cal_poly_val(polys(:,idx),time_step(i),r);
    end
end
end

function val = cal_poly_val(poly,t,r)
    val = 0;
    if r <= 0
        for i = 0:length(poly)-1
            val = val+poly(i+1)*t^i;
        end
    else
        for i = r:length(poly)-1
            val = val + poly(i+1)*prod(i-r+1:i)*t^(i-r);
        end
    end
end