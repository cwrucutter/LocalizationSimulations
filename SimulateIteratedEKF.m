
%% Simple Robot Differential Drive Simulation v1.0
% EJ Kreinar
clearvars

% rng(1)

dt = .1;    %Dt
T = 150;     % Sim time
b = .5;     %Track Width
tpmR = 25000;
tpmL = 25000;

% Settings:
%    - System: 
%        1 = 5-state
%        2 = 7-state
%        3 = 9-state
%        4 = 7-state with wheel odometry
%        5 = 5-state with wheel odometry
%        6 = 5-state with wheel odometry, encoders expressed as velocity
%        7 = 5-state with wheel odometry, parameterized encoders as v,w
settings.system = 7;

%% Initializations

initialize;

% Initialize the history
len = T/dt;
hist_state = zeros(len+1,length(x_true));
hist_est   = zeros(len+1,length(x_true));
hist_est2   = zeros(len+1,length(x_true));
hist_cov   = zeros(len+1,length(x_true),length(x_true));
hist_iter = zeros(len+1,1);
hist_movavg = zeros(len+1,3);
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

movAvg = zeros(3,10);

for i = 1:len
    
    % WHEEL VELOCITIES
    if i*dt > tracksum(trackindex,1)
        trackindex = trackindex + 1;
    end
    [V, vaccel] = AccelLimit(track(trackindex,2),V,vdot,dt);
    [w, waccel] = AccelLimit(track(trackindex,3),w,wdot,dt);
    Vr = V + b*1*w/2; %Calculate Vr
    Vl = V - b*1*w/2; %Calculate Vl
    
    % ACTUATOR NOISE
    % Simulate the imperfect actuator by adding noise to Vl and Vr
    Vr_true = Vr ;%+ Vr_sigma*randn;
    Vl_true = Vl ;%+ Vl_sigma*randn;
    
    % SIMULATE TRUE ROBOT MOTION
    x_true = SimulateMotion(Vr_true,Vl_true,x_true,b,dt);
    vRoff = 0; vLoff = 0;
    %     [vRoff, vLoff] = SimulateEncoderVelocityFault2(i,dt);
    
    if settings.system == 1
        x_true = [x_true; V; w;];
    elseif settings.system == 2
        x_true = [x_true; V; w; vRoff; vLoff];
    elseif settings.system == 3
        x_true = [x_true; V; w; vRoff; vLoff; vaccel; waccel];
    elseif settings.system == 4
        x_true = [x_true; V; w; vRoff; vLoff; tpmR; tpmL; b];
    elseif settings.system == 5
        x_true = [x_true; V; w; tpmR; tpmL; b];
    elseif settings.system == 6
        x_true = [x_true; V; w; 1; 1; 1];
    elseif settings.system == 7
        x_true = [x_true; V; w; 1; 1; 1];
    else
        error('Not a valid option for settings.system')
    end
    hist_state(i+1,:) = x_true';
    
    % PREDICTION: (every time step)
    % Estimate equations:
    Dr = Vr_true*dt + vRoff*dt; % + sigma_enc*randn   % Measure encoder displacement
    Dl = Vl_true*dt + vLoff*dt; % + sigma_enc*randn
    x_est_pre = f_sys(x_est);
    
    Ak = Asys(x_est,dt);
    P_pre     = Ak*P_est*Ak' + Qk;               % Extrapolate error covariance
    
    % ENCODER MEASUREMENT: (every time step)
    H = H_enc(x_est_pre,dt);
%     Rk = Rk_enc*[Vr_true 0 0;0 Vl_true 0; 0 0 1];
%     Z  = [Dr/dt; Dl/dt; w+sigma_gyro/10*randn];
    Rk = Rk_enc*[Vr_true 0;0 Vl_true];
    Z  = [Dr/dt*1.1; Dl/dt*0.9];
%     Z  = [Dr*tpmR; Dl*tpmL];
%     Zest  = H*x_est_pre;
    Zest = h_enc(x_est_pre,dt);
    innov = Z - Zest;    % Create the innovation (measurement-prediction)
%     velmeas = [(Dr/dt+Dl/dt)/2; (Dr/dt-Dl/dt)/b; Z(3)];
%     movAvg = [movAvg(:,2:end) velmeas];
%     hist_movavg(i+1,:) = mean(movAvg,2);
    
    if meas_enc == 1
        K = P_pre*H'*inv(H*P_pre*H' + Rk_enc);      % Calculate Kalman gain
        x_est_int = x_est_pre + K*(innov);      % State Estimate Update
        P_int     = (eye(length(x_true),length(x_true))-K*H)*P_pre;       % Error Covariance Update
    else
        x_est_int = x_est_pre;
        P_int = P_pre;
    end
    
    
    if meas_gyro == 1
        Z = x_true(5);
        Zest = x_est_int(5);
        H = [0 0 0 0 1 0 0 0];
        
        K = P_int*H'*inv(H*P_int*H' + 0.001);      % Calculate Kalman gain
        x_est_int = x_est_int + K*(Z-Zest);      % State Estimate Update
        P_int     = (eye(length(x_true),length(x_true))-K*H)*P_int;       % Error Covariance Update
    else
        x_est_int = x_est_int;
        P_int = P_pre;
    end
    
    % MEASUREMENT: (only when we get a measurement)
    if mod(i,timestep/dt) == 0 %GPS Update at 5Hz, for example
        Rk = Rk_gps;  % Create noise matrix
%         noise = sqrt(Rk)*randn(length(Rk),1);
        noise = 0*randn(length(Rk),1);
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
    else
        x_est = x_est_int;
        P_est = P_int;
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
filter_diverged = sum(err_norm>20)

err_x   = hist_est(:,1)-hist_state(:,1);
err_y   = hist_est(:,2)-hist_state(:,2);
err_tht = AngleDifference(hist_state(:,3),hist_est(:,3));
err_v = hist_est(:,4)-hist_state(:,4);
err_w = hist_est(:,5)-hist_state(:,5);
err_vRoff = hist_est(:,6)-hist_state(:,6);
err_vLoff = hist_est(:,7)-hist_state(:,7);
% err_vAcl = hist_est(:,8)-hist_state(:,8);
% err_wAcl = hist_est(:,9)-hist_state(:,9);
std_x   = sqrt(hist_cov(:,1,1));
std_y   = sqrt(hist_cov(:,2,2));
std_tht = sqrt(hist_cov(:,3,3));
std_v = sqrt(hist_cov(:,4,4));
std_w = sqrt(hist_cov(:,5,5));
std_vRoff = sqrt(hist_cov(:,6,6));
std_vLoff = sqrt(hist_cov(:,7,7));
% std_vAcl = sqrt(hist_cov(:,8,8));
% std_wAcl = sqrt(hist_cov(:,9,9));

disterr = sqrt(err_x.^2+err_y.^2);
rms_dist = sqrt(mean(disterr.^2))
rmserr_x   = sqrt(mean(err_x.^2));
rmserr_y   = sqrt(mean(err_y.^2));
rmserr_tht = sqrt(mean(err_tht.^2))
rmserr_v   = sqrt(mean(err_v.^2))
rmserr_w   = sqrt(mean(err_w.^2))
rmserr_vRoff = sqrt(mean(err_vRoff.^2))
rmserr_vLoff = sqrt(mean(err_vLoff.^2))

%% Truth Plots
close all
figure
plot(hist_state(:,1),hist_state(:,2),'b','LineWidth',3)
title('True track (b)');
xlabel('x (m)');
ylabel('y (m)');

figure
t = 0:dt:T;
subplot(2,1,1)
plot(t,hist_state(:,4),'b')
title('True Velocity (b)');
xlabel('Time (s)');
ylabel('Forward Velocity (m/s)');
ylim([-1,2])
subplot(2,1,2)
plot(t,hist_state(:,5),'b')
ylim([-1,1])
title('True Angular Velocity (b)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

%% Plots
close all

names = { 'X Position';
          'Y Position';
          'Heading';
          'Velocity';
          'Angular Velocity';
          'Right Wheel Velocity Err';
          'Left Wheel Velocity Err';
          'Right Wheel Ticks Per Meter';
          'Left Wheel Ticks Per Meter';
          'Track Width'};
if settings.system >= 5
    names = { 'X Position';
              'Y Position';
              'Heading';
              'Velocity';
              'Angular Velocity';
              'Right Wheel Ticks Per Meter';
              'Left Wheel Ticks Per Meter';
              'Track Width'};
end
     
plotStart = 1;
RMSstate = zeros(length(names),1);
for ii = 1:length(x_true)
    figure(plotStart+ii)
    RMSstate(ii,1) = plot_filter_state(ii, t, hist_state, hist_est, hist_cov);
    subplot(2,1,1)
    title(names{ii})
    subplot(2,1,2)
    title([names{ii} ' Error']);
    ylim([-5 5]);
end

%%
figure
plot(hist_state(:,1),hist_state(:,2),'b','LineWidth',3)
hold on
plot(hist_est(:,1),hist_est(:,2),'r-x') %,hist_est2(:,1),hist_est2(:,2),'g-x')
title('True track (b) vs Esimated track (r)');

figure
subplot(3,1,1)
t = 0:dt:T;
trim = 30;
plot(t(trim:end),err_x(trim:end),'b',t(trim:end),3*std_x(trim:end),'r-',t(trim:end),-3*std_x(trim:end),'r-');
grid on
title('X Error +- 3*Sigma');

subplot(3,1,2)
plot(t(trim:end),err_y(trim:end),'b',t(trim:end),3*std_y(trim:end),'r-',t(trim:end),-3*std_y(trim:end),'r-');
grid on
title('Y Error +- 3*Sigma');

subplot(3,1,3)
plot(t(trim:end),err_tht(trim:end),'b',t(trim:end),3*std_tht(trim:end),'r-',t(trim:end),-3*std_tht(trim:end),'r-');
grid on
title('Theta Error +- 3*Sigma');

figure
plot(t,hist_state(:,3),'b',t,hist_est(:,3),'r')
title('True heading (b) vs Estimated heading (r)');

figure
subplot(2,1,1)
plot(t,hist_state(:,4),'b',t,hist_est(:,4),'r')
title('True Velocity (b) vs Estimated Velocity (r)');
subplot(2,1,2)
plot(t,hist_state(:,5),'b',t,hist_est(:,5),'r')
title('True Omega (b) vs Estimated Omega (r)');

figure
subplot(2,1,1)
trim = 10;
plot(t(trim:end),err_v(trim:end),'b',t(trim:end),3*std_v(trim:end),'r-',t(trim:end),-3*std_v(trim:end),'r-');
title('Velocity Error +- 3*sigma');
subplot(2,1,2)
plot(t(trim:end),err_w(trim:end),'b',t(trim:end),3*std_w(trim:end),'r-',t(trim:end),-3*std_w(trim:end),'r-');
title('Omega Error +- 3*sigma');


figure
subplot(2,1,1)
trim = 10;
plot(t(trim:end),err_vRoff(trim:end),'b',t(trim:end),3*std_vRoff(trim:end),'r-',t(trim:end),-3*std_vRoff(trim:end),'r-');
title('Right Wheel Offset Error +- 3*sigma');
subplot(2,1,2)
plot(t(trim:end),err_vLoff(trim:end),'b',t(trim:end),3*std_vLoff(trim:end),'r-',t(trim:end),-3*std_vLoff(trim:end),'r-');
title('Left Wheel Offset Error +- 3*sigma');


% figure
% plot(t(hist_iter(:,1)>0),hist_iter(hist_iter(:,1)>0,1))
% ylim([0 max(hist_iter(:,1)+5)])
% title('Number of iterations run for GPS measurement');

figure
plot(t,err_norm)
title('normalized error in all states')

% figure
% subplot(2,1,1)
% plot(t,hist_state(:,8),'b',t,hist_est(:,8),'r')
% title('True V accel (b) vs Estimated V accel (r)');
% subplot(2,1,2)
% plot(t,hist_state(:,9),'b',t,hist_est(:,9),'r')
% title('True W accel (b) vs Estimated W accel (r)');
% 
% figure
% subplot(2,1,1)
% trim = 10;
% plot(t(trim:end),err_vAcl(trim:end),'b',t(trim:end),err_vAcl(trim:end)+3*std_vAcl(trim:end),'r-',t(trim:end),err_vAcl(trim:end)-3*std_vAcl(trim:end),'r-');
% title('V accel Error +- 3*sigma');
% subplot(2,1,2)
% plot(t(trim:end),err_wAcl(trim:end),'b',t(trim:end),err_wAcl(trim:end)+3*std_wAcl(trim:end),'r-',t(trim:end),err_wAcl(trim:end)-3*std_wAcl(trim:end),'r-');
% title('W accel Error +- 3*sigma');


% figure
% plot(t,hist_movavg(:,2),'b',t,hist_movavg(:,3),'r');
% title('Omega Moving Average: Encoders (b), Gyro (r)');


