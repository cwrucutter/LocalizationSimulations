
%% Simple Robot Differential Drive Simulation v1.0
% EJ Kreinar

dt = .1;    %Dt
T = 400;     % Sim time
b = .5;     %Track Width

% INITIAL VALUES
x0 = 1;     % Initial x
y0 = 1;     % Initial y
tht0 = 2;   % Initial theta
x_true = [x0; y0; tht0];

% INITIALIZE ESTIMATES
phi = eye(3,3);
x_est = x_true;
P_est = .001*eye(3,3);
% x_est2 = x_true;

% ASSIGN PROCESS NOISE
sigma_x = .01;        % Uncertainty in x
sigma_y = .01;        % Uncertainty in y
sigma_tht = .001;     % Uncertainty in theta
Q = [sigma_x^2 0 0; 0 sigma_y^2 0; 0 0 sigma_tht^2];
Qk = Q*dt;
Vr_sigma = 0;%.05;        % Uncertainty in left wheel
Vl_sigma = 0;%.05;        % Uncertainty in right wheel

% ENCODER MEASUREMENT
sigma_enc = .0002; % make this speed-dependent?

% GPS MEASUREMENT
H_gps = [ 1 0 0 ;
    0 1 0 ;
    0 0 1 ];
sigma_gps = 4;
sigma_head = .1;
V_gps = [ sigma_gps^2 0 0; 0 sigma_gps^2 0; 0 0 sigma_head^2];
timestep = 5;
Vk_gps = V_gps*timestep;

% Initialize the history
len = T/dt;
hist_state = zeros(len+1,3);
hist_est   = zeros(len+1,3);
hist_est2   = zeros(len+1,3);
hist_cov   = zeros(len+1,3,3);
hist_state(1,:) = x_true;
hist_est(1,:)   = x_est;
hist_est2(1,:)   = x_est2;
hist_cov(1,:,:) = P_est;

% GENERATE TRACK
track = zeros(len,2);
track(1:40/dt,1) = 1;     % Demo velocity
track(1:40/dt,2) = 0;    % Demo omega
track(40/dt:50/dt,1) = 1;     % Demo velocity
track(40/dt:50/dt,2) = .1;    % Demo omega
track(50/dt:70/dt,1) = 1;     % Demo velocity
track(50/dt:70/dt,2) = 0;    % Demo omega
track(70/dt:80/dt,1) = .5;     % Demo velocity
track(70/dt:80/dt,2) = -.5;    % Demo omega
track(80/dt:end,1) = 1;     % Demo velocity
track(80/dt:end,2) = 0;    % Demo omega

for i = 1:len
    
    % WHEEL VELOCITIES
    V = track(i,1);
    w = track(i,2);
    Vr = V + b*w/2; %Calculate Vr
    Vl = V - b*w/2; %Calculate Vl
    
    % ACTUATOR NOISE
    % Simulate the imperfect actuator by adding noise to Vl and Vr
    Vr_true = Vr + Vr_sigma*randn;
    Vl_true = Vl + Vl_sigma*randn;
    
    % SIMULATE TRUE ROBOT MOTION
    x_true = SimulateMotion(Vr_true,Vl_true,x_true,b,dt);
    hist_state(i+1,:) = x_true';
    
    % PREDICTION: (every time step)
    % Estimate equations:
    Dr = Vr_true*dt + sigma_enc*randn; % Measure encoder displacement
    Dl = Vl_true*dt + sigma_enc*randn; 
    del_D   = (Dr+Dl)/2;    % Extract distance and dTheta from encoders
    del_Tht = (Dr-Dl)/b;
    tht = x_est(3,1);          % Extract Theta
    x_est_pre(1,1) = x_est(1,1) + del_D*cos(tht+del_Tht/2);    % Predict x_est using the true encoder counts
    x_est_pre(2,1) = x_est(2,1) + del_D*sin(tht+del_Tht/2);
    x_est_pre(3,1) = x_est(3,1) + del_Tht;
    
    tht_mid = tht+del_Tht/2;
    Ak = [1  0   -del_D*sin(tht_mid) ; 
          0  1   -del_D*cos(tht_mid) ;
          0  0         1             ];
    Bk = [.5*cos(tht_mid)-del_D/2/b*sin(tht_mid)   .5*cos(tht_mid)+del_D/2/b*sin(tht_mid) ;
          .5*sin(tht_mid)+del_D/2/b*cos(tht_mid)   .5*sin(tht_mid)-del_D/2/b*cos(tht_mid) ;
                        1/b                                 -1/b                           ];
    P_pre     = Ak*P_est*Ak' + sigma_enc^2*Bk*Bk' + Qk;               % Extrapolate error covariance
    
% % %     % PREDICTION 2: (every time step)
% % %     tht = x_est2(3,1);          % Extract Theta
% % %     Dr_true = Vr_true*dt;
% % %     Dl_true = Vl_true*dt;
% % %     delD_true = (Dr_true+Dl_true)/2;
% % %     delTht_true = (Dr_true-Dl_true)/b;
% % %     B = [.5*cos(tht+delTht_true/2) .5*cos(tht+delTht_true/2) ;
% % %         .5*sin(tht+delTht_true/2) .5*sin(tht+delTht_true/2) ;
% % %         1/b            -1/b    ]; % Determine the input sensitivity
% % %     x_est_pre = x_est + lambda*[Vr;Vl];    % Predict x_est
% % %     x_est2 = x_est2 + B*[Dr_true;Dl_true];    % Predict x_est using the true encoder counts
    
    % MEASUREMENT: (only when we get a measurement)
    if mod(i,timestep/dt) == 0 %GPS Update at 5Hz, for example
        H = [H_gps];    % Create sensitivity matrix
        Rk = [Vk_gps];  % Create noise matrix
        noise = sqrt(Rk)*randn(length(Rk),1);
        Z     = H*x_true + noise;       % Take the measurement, adding simulated noise using randn
        Zest  = H*x_est_pre;
        temp  = Z - Zest;    % Create the innovation (measurement-prediction)
        angle = AngleDifference(Zest(3),Z(3));
        innov = [temp(1); temp(2); angle];
        
        K = P_pre*H'*inv(H*P_pre*H' + Rk);      % Calculate Kalman gain
        x_est = x_est_pre + K*(innov);  % State Estimate Update
        P_est = (eye(3,3)-K*H)*P_pre;           % Error Covariance Update
%         diff = x_est-x_est_pre;

    else
        x_est = x_est_pre;
        P_est = P_pre;
    end
    
    x_est(3) = mod(x_est(3),2*pi);
    
    hist_est(i+1,:) = x_est';
% % %     hist_est2(i+1,:) = x_est2';
    hist_cov(i+1,:,:) = P_est;
end

%% Plots
close all
figure
plot(hist_state(:,1),hist_state(:,2),'b','LineWidth',3)
hold on
plot(hist_est(:,1),hist_est(:,2),'r-x') %,hist_est2(:,1),hist_est2(:,2),'g-x')
title('True track (b) vs Esimated track (r)');

figure
err_x   = hist_est(:,1)-hist_state(:,1);
err_y   = hist_est(:,2)-hist_state(:,2);
err_tht = AngleDifference(hist_state(:,3),hist_est(:,3));
std_x   = sqrt(hist_cov(:,1,1));
std_y   = sqrt(hist_cov(:,2,2));
std_tht = sqrt(hist_cov(:,3,3));
subplot(3,1,1)
t = 0:dt:T;
plot(t,err_x,'b',t,err_x+3*std_x,'r-',t,err_x-3*std_x,'r-');
grid on
title('X Error +- 3*Sigma');

subplot(3,1,2)
plot(t,err_y,'b',t,err_y+3*std_y,'r-',t,err_y-3*std_y,'r-');
grid on
title('Y Error +- 3*Sigma');

subplot(3,1,3)
plot(t,err_tht,'b',t,err_tht+3*std_tht,'r-',t,err_tht-3*std_tht,'r-');
grid on
title('Theta Error +- 3*Sigma');

disterr = sqrt(err_x.^2+err_y.^2);
rms_dist = sqrt(mean(disterr.^2))
rmserr_x   = sqrt(mean(err_x.^2));
rmserr_y   = sqrt(mean(err_y.^2));
rmserr_tht = sqrt(mean(err_tht.^2))*180/pi


figure
plot(t,hist_state(:,3),'b',t,hist_est(:,3),'r')
title('True heading (b) vs Estimated heading (r)');

