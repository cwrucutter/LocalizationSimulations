function [ b ] = sample_motion_model_relative_beacon( u, b_old )
% SAMPLE_MOTION_MODEL_ODOMETRY samples based on the odometry information.
% u: Control input [xbar; ybar; thetabar; xbarnew; ybarnew; thetabarnew]
% b_old: Previous beacon location [x, y]

% Robot specific noise parameters
a1 = .00001;
a2 = .00001;
a3 = .00005;
a4 = .00001;
% a1 = .0001;
% a2 = .0001;
% a3 = .0001;
% a4 = .0001;

% Calculate rotation and translation differences from odometry data
del_rot1 = CoerceAngle(atan2(u(5)-u(2),u(4)-u(1)) - u(3));
del_tran = sqrt((u(5)-u(2))^2 + (u(4)-u(1))^2);
val_tran = [u(4)-u(1);u(5)-u(2)];
del_rot2 = CoerceAngle(u(6) - u(3) - del_rot1);

% % % Add noise
del_rot1_hat = del_rot1 - sample_normal_distribution(a1*del_rot1^2 + a2*del_tran^2);
del_tran_hat = del_tran - sample_normal_distribution(a3*del_tran^2 + a4*del_rot1^2 + a4*del_rot2^2);
del_rot2_hat = del_rot2 - sample_normal_distribution(a1*del_rot2^2 + a2*del_tran^2);

% Propagate the old state
% % b1 = zeros(3,1);
% % b  = zeros(3,1);
% b1 = b_old - RotMatrix(pi/2)*val_tran;
% b  = RotMatrix(-del_rot1-del_rot2)*b1;
b1 = RotMatrix(-del_rot1_hat)*b_old;
b2 = b1 - [0;del_tran_hat];
b3 = RotMatrix(-del_rot2_hat)*b2;
b = b3;

end