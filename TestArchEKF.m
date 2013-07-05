function [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchEKF( SimulateEncoderFault, procNoise, measNoise )


dt = .1;    %Dt
T = 100;     % Sim time
b = .5;     %Track Width

% INITIAL VALUES
x0 = 1;     % Initial x
y0 = 1;     % Initial y
tht0 = 2;   % Initial theta
% [x; y; tht; v; w; vRerr; vLerr]
x_true_7 = [x0; y0; tht0; 0; 0; 0; 0];
x_true_5 = [x0; y0; tht0; 0; 0];
x_true = x_true_7;

% INITIALIZE ESTIMATES
phi = eye(length(x_true),length(x_true));
x_est = x_true;
P_est = 1*eye(length(x_true),length(x_true));
for i=1:length(x_est)
    P_est(i,i) = 10;
end
P_est(6,6) = 1; % Set vRerr covariance smaller
P_est(7,7) = 1; % Set vLerr covariance smaller

% % CREATE SYSTEM
f_sys_7 = @(x) [x(1)+x(4)*dt*cos(x(3)+x(5)*dt/2);
                x(2)+x(4)*dt*sin(x(3)+x(5)*dt/2);
                x(3)+x(5)*dt;
                x(4);
                x(5);
                x(6);
                x(7)];
f_sys = f_sys_7;   
% % System 1: 7 States: [x;y;tht;v;w;verr1;verr2]
Asys_7 = @(x,dt) [1 0 -x(4)*dt*sin(x(3)+x(5)*dt/2) dt*cos(x(3)+x(5)*dt/2) -x(4)*dt*dt/2*sin(x(3)+x(5)*dt/2) 0  0;
           0 1  x(4)*dt*cos(x(3)+x(5)*dt/2) dt*sin(x(3)+x(5)*dt/2)  x(4)*dt*dt/2*cos(x(3)+x(5)*dt/2) 0  0;
           0 0            1                         0                       dt                       0  0;
           0 0            0                         1                       0                        0  0;
           0 0            0                         0                       1                        0  0;
           0 0            0                         0                       0                        1  0;
           0 0            0                         0                       0                        0  1];
% % System 2: 5 States: [x;y;tht;v;w]
Asys = Asys_7;

% ASSIGN PROCESS NOISE
Q_7 = [procNoise(1)^2 0 0 0 0 0 0; 0 procNoise(2)^2 0 0 0 0 0; 0 0 procNoise(3)^2 0 0 0 0;
     0 0 0 procNoise(4)^2 0 0 0; 0 0 0 0 procNoise(5)^2 0 0; 0 0 0 0 0 procNoise(6)^2 0; 0 0 0 0 0 0 procNoise(7)^2];
Q = Q_7;
Qk = Q*dt;
Vr_sigma = 0;%.05;        % Uncertainty in left wheel
Vl_sigma = 0;%.05;        % Uncertainty in right wheel

% ENCODER MEASUREMENT
H_enc_7 = [0 0 0 1  b/2 1 0 ;
         0 0 0 1 -b/2 0 1];
H_enc = H_enc_7;
sigma_enc = .001; % make this speed-dependent?
sigma_vr = measNoise(1);
sigma_vl = measNoise(2);
R_enc = [measNoise(1)^2 0; 0 measNoise(2)^2];
Rk_enc = R_enc*dt;
meas_enc = 1;

% GPS MEASUREMENT
xarm = 0.5; % Lever arm offset
yarm = 0;
H_gps_7 = @(x) [ 1 0 -xarm*sin(x(3))-yarm*cos(x(3)) 0 0 0 0; 
               0 1  xarm*cos(x(3))-yarm*sin(x(3)) 0 0 0 0];
H_gps = H_gps_7;
h_gps = @(x) [x(1) + xarm*cos(x(3)) - yarm*sin(x(3));
              x(2) + xarm*sin(x(3)) + yarm*cos(x(3))];
iekf_max = 1;
R_gps = [ measNoise(3)^2 0; 0 measNoise(3)^2];
timestep = .1;
Rk_gps = R_gps*timestep;

% Initialize the history
len = T/dt;
hist_state = zeros(len+1,length(x_true));
hist_est   = zeros(len+1,length(x_true));
hist_est2   = zeros(len+1,length(x_true));
hist_cov   = zeros(len+1,length(x_true),length(x_true));
hist_iter = zeros(len+1,1);
hist_state(1,:) = x_true;
hist_est(1,:)   = x_est;
% hist_est2(1,:)   = x_est2;
hist_cov(1,:,:) = P_est;

% GENERATE TRACK
track = zeros(len,2);

track = [30 1 0;
         10 1 .1;
         20 1 0;
         10 0.5 -0.5;
         10 1 0;
         5 0 0.2
         10 1 0.1;
         20 0.5 0.3];
tracksum = cumsum(track(:,1));
track = [track;
         T-tracksum(end) 0.2 0];
tracksum = [tracksum; T];
trackindex = 1;
vdot = 1;
wdot = 1;
V = 0;
w = 0;

for i = 1:len
    
    % WHEEL VELOCITIES
    if i*dt > tracksum(trackindex,1)
        trackindex = trackindex + 1;
    end
    V = AccelLimit(track(trackindex,2),V,vdot,dt);
    w = AccelLimit(track(trackindex,3),w,wdot,dt);
    Vr = V + b*w/2; %Calculate Vr
    Vl = V - b*w/2; %Calculate Vl
    
    % ACTUATOR NOISE
    % Simulate the imperfect actuator by adding noise to Vl and Vr
    Vr_true = Vr ;%+ Vr_sigma*randn;
    Vl_true = Vl ;%+ Vl_sigma*randn;
    
    % SIMULATE TRUE ROBOT MOTION
    x_true = SimulateMotion(Vr_true,Vl_true,x_true,b,dt);
    [vRoff, vLoff] = SimulateEncoderFault(i,dt);
    x_true = [x_true; V; w; vRoff; vLoff];
%     x_true = [x_true; V; w;];
    hist_state(i+1,:) = x_true';
    
    % PREDICTION: (every time step)
    % Estimate equations:
    Dr = Vr_true*dt + sigma_enc*randn + vRoff*dt; % Measure encoder displacement
    Dl = Vl_true*dt + sigma_enc*randn + vLoff*dt;
    x_est_pre = f_sys(x_est);
    
    Ak = Asys(x_est,dt);
    P_pre     = Ak*P_est*Ak' + Qk;               % Extrapolate error covariance
    
    % ENCODER MEASUREMENT: (every time step)
    H = H_enc;
    Rk = Rk_enc*[Vr_true 0;0 Vl_true];
    Z  = [Dr/dt; Dl/dt];
    Zest  = H*x_est_pre;
    innov = Z - Zest;    % Create the innovation (measurement-prediction)
    
    if meas_enc == 1
        K = P_pre*H'*inv(H*P_pre*H' + Rk_enc);      % Calculate Kalman gain
        x_est_int = x_est_pre + K*(innov);      % State Estimate Update
        P_int     = (eye(length(x_true),length(x_true))-K*H)*P_pre;       % Error Covariance Update
    else
        x_est_int = x_est_pre;
        P_int = P_pre;
    end
    
    % MEASUREMENT: (only when we get a measurement)
    Rk = Rk_gps;  % Create noise matrix
    noise = sqrt(Rk)*randn(length(Rk),1);
    Ztrue = h_gps(x_true);
    Z     = Ztrue + noise;       % Take the measurement, adding simulated noise using randn

    xit = x_est_int;
    for it = 1:iekf_max
%             Zest  = [x_est_int(1) + xarm*cos(x_est_int(3)) - yarm*sin(x_est_int(3));
%                      x_est_int(2) + xarm*sin(x_est_int(3)) + yarm*cos(x_est_int(3))];
        innov  = Z - h_gps(xit(:,it)) - H_gps(xit(:,it))*(xit(:,1)-xit(:,it));    % Create the innovation (measurement-prediction)

        H = H_gps(xit(:,it));
        Rk = Rk_gps*5;
        K = P_int*H'*inv(H*P_int*H' + Rk);      % Calculate Kalman gain
        x_est = x_est_int + K*(innov);  % State Estimate Update
        P_est = (eye(length(x_true),length(x_true))-K*H)*P_int;           % Error Covariance Update
        %         diff = x_est-x_est_pre;
        hist_iter(i+1,1) = it;
        if it > 1
            xdiff = xit(:,it)-xit(:,it-1);
            if sqrt(xdiff'*xdiff) < 0.0005
                break;
            end
        end
        xit = [xit x_est];
    end
    x_est(3) = CoerceAngle(x_est(3));
    
    hist_est(i+1,:) = x_est';
    % % %     hist_est2(i+1,:) = x_est2';
    hist_cov(i+1,:,:) = P_est;
end


%% Evaluation

err_norm = zeros(len+1,1);
err_all = hist_est - hist_state;
for j=1:len
    std_all = diag(squeeze(hist_cov(j,:,:)));
    temp = err_all(j,:)'./(3*std_all);
    err_norm(j,1) = sqrt(temp'*temp/length(temp));
end
filter_diverged = sum(err_norm>20);

err_x   = hist_est(:,1)-hist_state(:,1);
err_y   = hist_est(:,2)-hist_state(:,2);
err_tht = AngleDifference(hist_state(:,3),hist_est(:,3));
err_v = hist_est(:,4)-hist_state(:,4);
err_w = hist_est(:,5)-hist_state(:,5);
err_vRoff = hist_est(:,6)-hist_state(:,6);
err_vLoff = hist_est(:,7)-hist_state(:,7);
std_x   = sqrt(hist_cov(:,1,1));
std_y   = sqrt(hist_cov(:,2,2));
std_tht = sqrt(hist_cov(:,3,3));
std_v = sqrt(hist_cov(:,4,4));
std_w = sqrt(hist_cov(:,5,5));
std_vRoff = sqrt(hist_cov(:,6,6));
std_vLoff = sqrt(hist_cov(:,7,7));

disterr = sqrt(err_x.^2+err_y.^2);
rms_dist = sqrt(mean(disterr.^2));
rmserr_x   = sqrt(mean(err_x.^2));
rmserr_y   = sqrt(mean(err_y.^2));
rmserr_tht = sqrt(mean(err_tht.^2));
rmserr_v   = sqrt(mean(err_v.^2));
rmserr_w   = sqrt(mean(err_w.^2));
rmserr_vRoff = sqrt(mean(err_vRoff.^2));
rmserr_vLoff = sqrt(mean(err_vLoff.^2));


end

