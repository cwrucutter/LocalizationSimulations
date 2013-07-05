function [ v, accel ] = AccelLimit(vgoal, vlast, vdot, dt)
%ACCELLIMIT limits the velocities based on the previous velocities and the
%accelerations. All inputs must be scalars
% todo: generalize for matrix inputs
inc = 0;
accel = 0;
if vgoal > vlast % Going Up
    inc = vdot*dt;
    accel = vdot;
elseif vgoal < vlast % Going Down
    inc = -vdot*dt;
    accel = -vdot;
end
v = vlast + inc;
if (inc > 0 && v > vgoal) || (inc < 0 && v < vgoal)
    v = vgoal;
end

end

