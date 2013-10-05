
% INITIAL VALUES
x0 = 1;     % Initial x
y0 = 1;     % Initial y
tht0 = 2;   % Initial theta
% [x; y; tht; v; w; vRerr; vLerr]
x_true_E = [x0; y0; tht0; 0; 0; 0; 0; tpmR-10; tpmL; b];
x_true_F = [x0; y0; tht0; 0; 0; tpmR-10; tpmL; b];
x_true_G = [x0; y0; tht0; 0; 0; 1; 1; 1];
x_true_H = [x0; y0; tht0; 0; 0; 1; 1; 1];
x_true_9 = [x0; y0; tht0; 0; 0; 0; 0; 0; 0];
x_true_7 = [x0; y0; tht0; 0; 0; 0; 0];
x_true_5 = [x0; y0; tht0; 0; 0];

% % CREATE SYSTEM
f_sys_E = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4); %velocity
                x(5); %angular vel
                x(6); %vRerr
                x(7); %vLerr
                x(8); %tpmR
                x(9); %tpmL
                x(10)]; %wheelbase
f_sys_F = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4); %velocity
                x(5); %angular vel
                x(6); %tpmR
                x(7); %tpmL
                x(8)]; %wheelbase
f_sys_G = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4); %velocity
                x(5); %angular vel
                x(6); %scaleR
                x(7); %scaleL
                x(8)]; %scaleB
f_sys_H = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4); %velocity
                x(5); %angular vel
                x(6); %scaleR
                x(7); %scaleL
                x(8)]; %scaleB
f_sys_9 = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4)+x(8)*dt;
                x(5)+x(9)*dt;
                x(6);
                x(7);
                x(8);
                x(9)];
f_sys_7 = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4);
                x(5);
                x(6);
                x(7)];
f_sys_5 = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4);
                x(5);];
            
% % System 1: 7 States: [x;y;tht;v;w;verr1;verr2]
Asys_E = @(x,dt) ...
    [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0 0 0 0 0;
     0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0 0 0 0 0;
     0 0            1                         0                       dt                       0 0 0 0 0;
     0 0            0                         1                       0                        0 0 0 0 0;
     0 0            0                         0                       1                        0 0 0 0 0;
     0 0            0                         0                       0                        1 0 0 0 0;
     0 0            0                         0                       0                        0 1 0 0 0;
     0 0            0                         0                       0                        0 0 1 0 0;
     0 0            0                         0                       0                        0 0 0 1 0;
     0 0            0                         0                       0                        0 0 0 0 1];
Asys_F = @(x,dt) ...
    [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0 0 0 ;
     0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0 0 0 ;
     0 0            1                         0                       dt                       0 0 0 ;
     0 0            0                         1                       0                        0 0 0 ;
     0 0            0                         0                       1                        0 0 0 ;
     0 0            0                         0                       0                        1 0 0 ;
     0 0            0                         0                       0                        0 1 0 ;
     0 0            0                         0                       0                        0 0 1 ];
Asys_G = @(x,dt) ...
    [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0 0 0 ;
     0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0 0 0 ;
     0 0            1                         0                       dt                       0 0 0 ;
     0 0            0                         1                       0                        0 0 0 ;
     0 0            0                         0                       1                        0 0 0 ;
     0 0            0                         0                       0                        1 0 0 ;
     0 0            0                         0                       0                        0 1 0 ;
     0 0            0                         0                       0                        0 0 1 ];
Asys_H = @(x,dt) ...
    [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0 0 0 ;
     0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0 0 0 ;
     0 0            1                         0                       dt                       0 0 0 ;
     0 0            0                         1                       0                        0 0 0 ;
     0 0            0                         0                       1                        0 0 0 ;
     0 0            0                         0                       0                        1 0 0 ;
     0 0            0                         0                       0                        0 1 0 ;
     0 0            0                         0                       0                        0 0 1 ];
Asys_9 = @(x,dt) [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0  0 0 0;
           0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0  0 0 0;
           0 0            1                         0                       dt                       0  0 0 0;
           0 0            0                         1                       0                        0  0 dt 0;
           0 0            0                         0                       1                        0  0 0 dt;
           0 0            0                         0                       0                        1  0 0 0;
           0 0            0                         0                       0                        0  1 0 0;
           0 0            0                         0                       0                        0  0 1 0;
           0 0            0                         0                       0                        0  0 0 1];
Asys_7 = @(x,dt) [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0  0;
           0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0  0;
           0 0            1                         0                       dt                       0  0;
           0 0            0                         1                       0                        0  0;
           0 0            0                         0                       1                        0  0;
           0 0            0                         0                       0                        1  0;
           0 0            0                         0                       0                        0  1];
% % System 2: 5 States: [x;y;tht;v;w]
Asys_5 = @(x,dt) [ 1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2);
                   0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2);
                   0 0            1                         0                       dt                      ;
                   0 0            0                         1                       0                       ;
                   0 0            0                         0                       1                       ];


% ASSIGN PROCESS NOISE
sigma_x = .01;        % Uncertainty in x
sigma_y = .01;        % Uncertainty in y
sigma_tht = .001;     % Uncertainty in theta
sigma_v = .3;         % Uncertainty in velocity
sigma_w = .3;         % Uncertainty in omega
sigma_vRerr = 0.05;
sigma_vLerr = 0.05;
sigma_vdot = .5;
sigma_wdot = .5;
sigma_tpmR = 0.01;
sigma_tpmL = 0.01;
sigma_b = 0.0001;
sigma_tpmR_scale = 0.1;
sigma_tpmL_scale = 0.1;
sigma_b_scale = .5;
Q_E = [sigma_x^2 0 0 0 0 0 0 0 0 0; 0 sigma_y^2 0 0 0 0 0 0 0 0; 0 0 sigma_tht^2 0 0 0 0 0 0 0;
       0 0 0 sigma_v^2 0 0 0 0 0 0; 0 0 0 0 sigma_w^2 0 0 0 0 0; 0 0 0 0 0 sigma_vRerr^2 0 0 0 0; 0 0 0 0 0 0 sigma_vLerr^2 0 0 0;
       0 0 0 0 0 0 0 sigma_tpmR^2 0 0; 0 0 0 0 0 0 0 0 sigma_tpmL^2 0; 0 0 0 0 0 0 0 0 0 sigma_b^2];
Q_F = [sigma_x^2 0 0 0 0 0 0 0; 0 sigma_y^2 0 0 0 0 0 0; 0 0 sigma_tht^2 0 0 0 0 0;
       0 0 0 sigma_v^2 0 0 0 0; 0 0 0 0 sigma_w^2 0 0 0;
       0 0 0 0 0 sigma_tpmR^2 0 0; 0 0 0 0 0 0 sigma_tpmL^2 0; 0 0 0 0 0 0 0 sigma_b^2];
Q_G = [sigma_x^2 0 0 0 0 0 0 0; 0 sigma_y^2 0 0 0 0 0 0; 0 0 sigma_tht^2 0 0 0 0 0;
       0 0 0 sigma_v^2 0 0 0 0; 0 0 0 0 sigma_w^2 0 0 0;
       0 0 0 0 0 sigma_tpmR_scale^2 0 0; 0 0 0 0 0 0 sigma_tpmL_scale^2 0; 0 0 0 0 0 0 0 sigma_b_scale^2];
Q_H = [sigma_x^2 0 0 0 0 0 0 0; 0 sigma_y^2 0 0 0 0 0 0; 0 0 sigma_tht^2 0 0 0 0 0;
       0 0 0 sigma_v^2 0 0 0 0; 0 0 0 0 sigma_w^2 0 0 0;
       0 0 0 0 0 sigma_tpmR_scale^2 0 0; 0 0 0 0 0 0 sigma_tpmL_scale^2 0; 0 0 0 0 0 0 0 sigma_b_scale^2];
Q_9 = [sigma_x^2 0 0 0 0 0 0 0 0; 0 sigma_y^2 0 0 0 0 0 0 0; 0 0 sigma_tht^2 0 0 0 0 0 0;
       0 0 0 sigma_v^2 0 0 0 0 0; 0 0 0 0 sigma_w^2 0 0 0 0; 0 0 0 0 0 sigma_vRerr^2 0 0 0; 0 0 0 0 0 0 sigma_vLerr^2 0 0;
       0 0 0 0 0 0 0 sigma_vdot^2 0; 0 0 0 0 0 0 0 0 sigma_wdot^2; ];
Q_7 = [sigma_x^2 0 0 0 0 0 0; 0 sigma_y^2 0 0 0 0 0; 0 0 sigma_tht^2 0 0 0 0;
     0 0 0 sigma_v^2 0 0 0; 0 0 0 0 sigma_w^2 0 0; 0 0 0 0 0 sigma_vRerr^2 0; 0 0 0 0 0 0 sigma_vLerr^2];
Q_5 = [sigma_x^2 0 0 0 0; 0 sigma_y^2 0 0 0; 0 0 sigma_tht^2 0 0;
     0 0 0 sigma_v^2 0; 0 0 0 0 sigma_w^2];
Vr_sigma = 0;%.05;        % Uncertainty in left wheel
Vl_sigma = 0;%.05;        % Uncertainty in right wheel

% ENCODER MEASUREMENT
h_enc_E = @(x, dt) dt * ...
    [x(8)*x(4) + x(8)*x(10)/2*x(5) + x(8)*x(6);
     x(9)*x(4) - x(9)*x(10)/2*x(5) + x(9)*x(7)];
h_enc_F = @(x, dt) dt * ...
    [x(6)*x(4) + x(6)*x(8)/2*x(5);
     x(7)*x(4) - x(7)*x(8)/2*x(5)];
h_enc_G = @(x, dt) ...
    [x(6)*x(4) + x(6)*b*x(8)*x(5)/2;
     x(7)*x(4) - x(7)*b*x(8)*x(5)/2];
h_enc_H = @(x, dt) ...
    [1/2 * ( x(4)*(x(6)+x(7)) + x(5)*b*x(8)/2*(x(6)-x(7)) );
     1/b * ( x(4)*(x(6)-x(7)) + x(5)*b*x(8)/2*(x(6)+x(7)) )];
h_enc_7 = @(x, dt) dt * ...
    [tpmR*x(4) + tpmR*b/2*x(5) + tpmR*x(6);
     tpmL*x(4) - tpmL*b/2*x(5) + tpmL*x(7)];
H_enc_E = @(x, dt) dt * ...
    [0 0 0 x(8)  x(8)*x(10)/2 x(8) 0  (x(4)+x(10)/2*x(5)+x(6)) 0   x(8)/2*x(5);
     0 0 0 x(9) -x(9)*x(10)/2  0  x(9) 0 (x(4)-x(10)/2*x(5)+x(7)) -x(9)/2*x(5)];
H_enc_F = @(x, dt) dt * ...
    [0 0 0 x(6)  x(6)*x(8)/2 (x(4)+x(8)/2*x(5)) 0  x(6)/2*x(5);
     0 0 0 x(7) -x(7)*x(8)/2 0 (x(4)-x(8)/2*x(5)) -x(7)/2*x(5)];
H_enc_G = @(x, dt) ...
    [0 0 0 x(6)  x(6)*b*x(8)/2 (x(4)+b*x(8)*x(5)/2) 0  x(6)*b*x(5)/2 ;
     0 0 0 x(7) -x(7)*b*x(8)/2 0 (x(4)-b*x(8)*x(5)/2) -x(7)*b*x(5)/2];
% H_enc_H = @(x, dt) ...
%     [0 0 0 (x(6)+x(7))/2   b*(x(6)-x(7))/4   x(4)/2+b*x(5)/4   x(4)/2-b*x(5)/4  0 ;
%      0 0 0 (x(6)-x(7))/b     (x(6)-x(7))/2   x(4)/b+x(5)/2    -x(4)/b+x(5)/2      0 ];
H_enc_H = @(x, dt) ...
    [0 0 0 (x(6)+x(7))/2   b*x(8)*(x(6)-x(7))/4   x(4)/2+b*x(8)*x(5)/4   x(4)/2-b*x(8)*x(5)/4  b*x(5)*(x(6)-x(7))/4 ;
     0 0 0 (x(6)-x(7))/b     x(8)*(x(6)-x(7))/2   x(4)/b+x(8)*x(5)/2    -x(4)/b+x(8)*x(5)/2      x(5)*(x(6)+x(7))/2 ];
H_enc_9 = @(x) dt * ...
    [0 0 0 tpmR  tpmR*b/2 tpmR 0 0 0;
     0 0 0 tpmL -tpmL*b/2 0 tpmL 0 0];
% H_enc_7 = [0 0 0 1  b/2 1 0 ;
%            0 0 0 1 -b/2 0 1;
%            0 0 0 0   1  0 0];
H_enc_7 = @(x, dt) dt * ...
    [0 0 0 tpmR  tpmR*b/2 tpmR 0 ;
     0 0 0 tpmL -tpmL*b/2 0 tpmL];
H_enc_5 = @(x, dt) dt * ...
    [0 0 0 tpmR  tpmR*b/2;
     0 0 0 tpmL -tpmL*b/2];
sigma_enc = .001; % noise on encoder measurement % make this speed-dependent?
sigma_gyro = 0.05;
sigma_vR = 0.01*dt*tpmR;
sigma_vL = 0.01*dt*tpmL;
R_enc = [sigma_vR^2 0; 0 sigma_vL^2];
% R_enc = [sigma_v^2 0 0; 0 sigma_w^2 0;0 0 sigma_gyro^2];
Rk_enc = R_enc*dt;
meas_enc = 1;
meas_gyro = 1;

% GPS MEASUREMENT
xarm = 0; % Lever arm offset
yarm = 0;
% H_gps = [ 1 0 0 0 0 0 0;
%           0 1 0 0 0 0 0];
%           0 0 1 0 0 ];
%     0 0 1 ];
H_gps_E = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0 0 0 0];
H_gps_F = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0 0];
H_gps_G = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0 0];
H_gps_H = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0 0];
H_gps_9 = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0 0 0];
H_gps_7 = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0];
H_gps_5 = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0];
h_gps = @(x) [x(1) + xarm*cos(x(3)) - yarm*sin(x(3));
              x(2) + xarm*sin(x(3)) + yarm*cos(x(3))];
iekf_max = 1;
sigma_gps = .05;
sigma_head = .04;
R_gps = [ sigma_gps^2 0; 0 sigma_gps^2];
% R_gps = [ sigma_gps^2 0 0; 0 sigma_gps^2 0; 0 0 sigma_head^2];
timestep = .2;
Rk_gps = R_gps*timestep;

% Settings:
%    - System: 
%        1 = 5-state
%        2 = 7-state
%        3 = 9-state
%        4 = 7-state with wheel odometry
if settings.system == 1
    x_true = x_true_5;
    f_sys = f_sys_5;
    Asys = Asys_5;
    Q = Q_5;
    h_enc = h_enc_5;
    H_enc = H_enc_5;
    H_gps = H_gps_5;
elseif settings.system == 2
    x_true = x_true_7;
    f_sys = f_sys_7;
    Asys = Asys_7;
    Q = Q_7;
    h_enc = h_enc_7;
    H_enc = H_enc_7;
    H_gps = H_gps_7;
elseif settings.system == 3
    x_true = x_true_9;
    f_sys = f_sys_9;
    Asys = Asys_9;
    Q = Q_9;
    h_enc = h_enc_9;
    H_enc = H_enc_9;
    H_gps = H_gps_9;
elseif settings.system == 4
    x_true = x_true_E;
    f_sys = f_sys_E;
    Asys = Asys_E;
    Q = Q_E;
    h_enc = h_enc_E;
    H_enc = H_enc_E;
    H_gps = H_gps_E;
elseif settings.system == 5
    x_true = x_true_F;
    f_sys = f_sys_F;
    Asys = Asys_F;
    Q = Q_F;
    h_enc = h_enc_F;
    H_enc = H_enc_F;
    H_gps = H_gps_F;
elseif settings.system == 6
    x_true = x_true_G;
    f_sys = f_sys_G;
    Asys = Asys_G;
    Q = Q_G;
    h_enc = h_enc_G;
    H_enc = H_enc_G;
    H_gps = H_gps_G;
elseif settings.system == 7
    x_true = x_true_H;
    f_sys = f_sys_H;
    Asys = Asys_H;
    Q = Q_H;
    h_enc = h_enc_H;
    H_enc = H_enc_H;
    H_gps = H_gps_H;
else
    error('Not a valid option for settings.system')
end


% INITIALIZE ESTIMATES
phi = eye(length(x_true),length(x_true));
x_est = x_true;
P_est = 1*eye(length(x_true),length(x_true));
for i=1:5
    P_est(i,i) = 10;
end

if settings.system == 4
    P_est(10,10) = .005;
end
if settings.system == 5
    P_est(6,6) = 5;
    P_est(7,7) = 5;
    P_est(8,8) = .05;
end
if settings.system == 6
    P_est(6,6) = 0.1;
    P_est(7,7) = 0.1;
    P_est(8,8) = 0.01;
end
if settings.system == 7
    P_est(6,6) = 1;
    P_est(7,7) = 1;
    P_est(8,8) = 1;
end

Qk = Q*dt;
