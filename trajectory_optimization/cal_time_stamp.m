function ts = cal_time_stamp(inserted_traj,T)
    dis = sum((inserted_traj(:,2:end) - inserted_traj(:,1:end-1)).^2,1).^0.5;
    dt = T/sum(dis);
    ts = [0 cumsum(dis*dt)];
end