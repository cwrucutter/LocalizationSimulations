function [ x ] = sample_motion_model_odometry( u, x_old )
% SAMPLE_MOTION_MODEL_ODOMETRY samples based on the odometry information.
% u: Control input [xbar; ybar; thetabar; xbarnew; ybarnew; thetabarnew]
% x_old: Previous state [x, y, theta]


% Robot specific noise parameters
% a1 = .00001;
% a2 = .00001;
% a3 = .00005;
% a4 = .00001;
a1 = .01;
a2 = .01;
a3 = .01;
a4 = .01;

% Calculate rotation and translation differences from odometry data
del_rot1 = CoerceAngle(atan2(u(5)-u(2),u(4)-u(1)) - u(3));
del_tran = sqrt((u(5)-u(2))^2 + (u(4)-u(1))^2);
del_rot2 = CoerceAngle(u(6) - u(3) - del_rot1);

% Add noise
del_rot1_hat = del_rot1 - sample_normal_distribution(a1*del_rot1^2 + a2*del_tran^2);
del_tran_hat = del_tran - sample_normal_distribution(a3*del_tran^2 + a4*del_rot1^2 + a4*del_rot2^2);
del_rot2_hat = del_rot2 - sample_normal_distribution(a1*del_rot2^2 + a2*del_tran^2);

% Propagate the old state
x = zeros(3,1);
x(1) = x_old(1) + del_tran_hat * cos(x_old(3)+del_rot1_hat);
x(2) = x_old(2) + del_tran_hat * sin(x_old(3)+del_rot1_hat);
x(3) = x_old(3) + del_rot1_hat + del_rot2_hat;
% x(3) = mod(x(3),2*pi);
x(3) = CoerceAngle(x(3));

end

