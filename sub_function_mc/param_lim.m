%% Confine the control_params
function [ vel_next ] = param_lim(vel_current,vel_next,max_vel_x,max_vel_y,max_acc_x,max_acc_y,dt)
% limition on speed x
if vel_next(1) - vel_current(1) >= 0
    vel_next(1) = min(vel_current(1) + vel_next(1) - vel_current(1),vel_current(1) + max_acc_x*dt);
else
    vel_next(1) = max(vel_current(1) + vel_next(1) - vel_current(1),vel_current(1) - max_acc_x*dt);
end
if vel_next(1) >= 0
    vel_next(1) = min(vel_next(1),max_vel_x);
else
    vel_next(1) = max(vel_next(1),-max_vel_x);
end
% limition on speed y
if vel_next(2) - vel_current(2) >= 0
    vel_next(2) = min(vel_current(2) + vel_next(2) - vel_current(2),vel_current(2)+max_acc_y*dt);
else
    vel_next(2) = max(vel_current(2) + vel_next(2) - vel_current(2),vel_current(2)-max_acc_y*dt);
end
if vel_next(2) >= 0
    vel_next(2) = min(vel_next(2),max_vel_y);
else
    vel_next(2) = max(vel_next(2),-max_vel_y);
end
end
