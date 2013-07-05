%% Setup Variables
numTests = 10;
numOutputs = 7;

verr1_true = zeros(length(t),1);
werr1_true = zeros(length(t),1);
verr1_true(t>70 & t<80)=0.1/2;
werr1_true(t>70 & t<80)=0.1/.5;

verr2_true = zeros(length(t),1);
werr2_true = zeros(length(t),1);
verr2_true(t>20 & t<30)=.3;
werr2_true(t>10 & t<30)=0;

results5States_Err0 = zeros(numTests,numOutputs);
results7States_Err0 = zeros(numTests,numOutputs);
results9States_Err0 = zeros(numTests,numOutputs);
procNoise = zeros(numTests,9);
measNoise = zeros(numTests,3);

% sigma_x = .01;        % Uncertainty in x
% sigma_y = .01;        % Uncertainty in y
% sigma_tht = .001;     % Uncertainty in theta
% sigma_v = .1;         % Uncertainty in velocity
% sigma_w = .1;         % Uncertainty in omega
% sigma_vRerr = 0.05;
% sigma_vLerr = 0.05;
% sigma_vdot = .5;
% sigma_wdot = .5;
sigma_x = .01;        % Uncertainty in x
sigma_y = .01;        % Uncertainty in y
sigma_tht = .001;     % Uncertainty in theta
sigma_v = .3;         % Uncertainty in velocity
sigma_w = .3;         % Uncertainty in omega
sigma_vRerr = 0.2;
sigma_vLerr = 0.2;
sigma_vdot = .1;
sigma_wdot = .1;
procNoise = repmat([sigma_x sigma_y sigma_tht sigma_v sigma_w sigma_vRerr sigma_vLerr sigma_vdot sigma_wdot],numTests,1);
sigma_vr = 0.05;
sigma_vl = 0.05;
sigma_gps = 0.05;
measNoise = repmat([sigma_vr sigma_vl sigma_gps],numTests,1);


%% Tests
i = 1;
[ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff, hist_est_5, err_norm_5, hist_state ] = TestFilter5States(@(i,dt) SimulateEncoderVelocityFault1(i,dt), procNoise(i,:), measNoise(i,:));
[ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff, hist_est_7, err_norm_7, hist_state ] = TestFilter7States(@(i,dt) SimulateEncoderVelocityFault1(i,dt), procNoise(i,:), measNoise(i,:));
[ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff, hist_est_9, err_norm_9, hist_state ] = TestFilter9States(@(i,dt) SimulateEncoderVelocityFault1(i,dt), procNoise(i,:), measNoise(i,:));

%% Plots

close all
dt = 0.1;
T = 100;
t = 0:dt:T;

% figure
% plot(hist_state(:,1),hist_state(:,2),'b','LineWidth',3)
% hold on
% plot(hist_est(:,1),hist_est(:,2),'r-x') %,hist_est2(:,1),hist_est2(:,2),'g-x')
% title('True track (b) vs Esimated track (r)');
% 
% figure
% subplot(3,1,1)
% t = 0:dt:T;
% trim = 30;
% plot(t(trim:end),err_x(trim:end),'b',t(trim:end),err_x(trim:end)+3*std_x(trim:end),'r-',t(trim:end),err_x(trim:end)-3*std_x(trim:end),'r-');
% grid on
% title('X Error +- 3*Sigma');
% 
% subplot(3,1,2)
% plot(t(trim:end),err_y(trim:end),'b',t(trim:end),err_y(trim:end)+3*std_y(trim:end),'r-',t(trim:end),err_y(trim:end)-3*std_y(trim:end),'r-');
% grid on
% title('Y Error +- 3*Sigma');
% 
% subplot(3,1,3)
% plot(t(trim:end),err_tht(trim:end),'b',t(trim:end),err_tht(trim:end)+3*std_tht(trim:end),'r-',t(trim:end),err_tht(trim:end)-3*std_tht(trim:end),'r-');
% grid on
% title('Theta Error +- 3*Sigma');
% 
% figure
% plot(t,hist_state(:,3),'b',t,hist_est(:,3),'r')
% title('True heading (b) vs Estimated heading (r)');

% figure
% subplot(2,1,1)
% plot(t,hist_est_5(:,4),'r',t,hist_est_7(:,4),'g',t,hist_est_9(:,4),'k')
% hold on;
% plot(t,hist_state(:,4),'b','LineWidth',2);
% title('True Velocity (b) vs Estimated 5-State (r) 7-State (g) 9-State (k)');
% subplot(2,1,2)
% plot(t,hist_est_5(:,5),'r',t,hist_est_7(:,5),'g',t,hist_est_9(:,5),'k')
% hold on;
% plot(t,hist_state(:,5),'b','LineWidth',2)
% title('True Angular Velocity (b) vs Estimated 5-State (r) 7-State (g) 9-State (k)');

figure
subplot(2,1,1)
trim = 10;
verr_5 = hist_est_5(:,4)-hist_state(:,4);
verr_7 = hist_est_7(:,4)-hist_state(:,4);
verr_9 = hist_est_9(:,4)-hist_state(:,4);
werr_5 = hist_est_5(:,5)-hist_state(:,5);
werr_7 = hist_est_7(:,5)-hist_state(:,5);
werr_9 = hist_est_9(:,5)-hist_state(:,5);
% plot(t(trim:end),verr_9(trim:end),'g',t(trim:end),verr_5(trim:end),'b',t(trim:end),verr_7(trim:end),'r');
% plot(t(trim:end),[verr_7(trim:end) verr_5(trim:end) ])% ]);
plot(t(trim:end), verr_7(trim:end), 'b'); hold on;
plot(t(trim:end), verr_5(trim:end), 'r'); hold off;
title('Fault 1: Simulated True Forward Velocity Error');
xlabel('Time (s)')
ylabel('Forward Velocity (m/s)')
% legend('AKF','AKF with Accel','5-State EKF')
legend('AKF','5-State EKF')
ylim([-.1,.4])
subplot(2,1,2)
% plot(t(trim:end),[werr_7(trim:end) werr_9(trim:end) werr_5(trim:end) ]);
% plot(t(trim:end),[werr_7(trim:end) werr_5(trim:end) ]);
plot(t(trim:end), werr_7(trim:end), 'b'); hold on;
plot(t(trim:end), werr_5(trim:end), 'r'); hold off;
title('Fault 1: Simulated True Angular Velocity Error');
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
ylim([-.4,.4])


% figure
% plot(t,err_norm_5,'r',t,err_norm_7,'g',t,err_norm_9,'k')
% title('Normalized Error: 5-State (r) 7-State (g) 9-State (k)')

