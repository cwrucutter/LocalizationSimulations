
%% Robot Differential Drive Particle Filter v1.3 w Lever Arm + Lidar Landmarks + Federated KF

% EJ Kreinar
clear all
close all

dt = .1;    %Dt
T = 60;    % Sim time
b = .5;     %Track Width

% INITIAL VALUES
x0 = 0;     % Initial x
y0 = 0;     % Initial y
tht0 = 0;   % Initial theta
x_true = [x0; y0; tht0];

% INITIALIZE ESTIMATES
x_odom = [0; 0; 0];
x_odom_old = x_odom;
n_part = 1000; % number of particles
% particles = repmat([x_true; 1/n_part], 1, n_part);
particles = [2*rand(1,n_part);2*rand(1,n_part);2*pi*rand(1,n_part)]-repmat([1;1;pi],1,n_part);

% ASSIGN PROCESS NOISE
Vr_sigma = 0; % True Actuator Noise on the encoder left
Vl_sigma = 0; % True Actuator Noise on the encoder right

% ENCODER MEASUREMENT
sigma_enc = .01; % make this speed-dependent?
sigma_gyro1 = 0.1;
sigma_vel1 = 0.1;
sigma_gyro2 = 0.05;
sigma_vel2 = 0.05;

% GPS MEASUREMENT
H_gps = [ 1 0 0 ;
    0 1 0 ;
    0 0 1 ];
sigma_gps = .1;
sigma_head = .1;
V_gps = [ sigma_gps^2 0 0; 0 sigma_gps^2 0; 0 0 sigma_head^2];
timestep = 1;
Vk_gps = V_gps*timestep;

% LANDMARK MEASUREMENT
map = [ [20;40] [-20;40] [0;80]];
[dim nland] = size(map);
sigma_r = 5;
sigma_psi = 1;
sigma_r = sigma_r*timestep;
sigma_psi = sigma_psi*timestep;

% Initialize the Velocity Kalman Pre-Filter
Xtot = [0;0;0;0]; Ptot = zeros(4,4);
% CKF = VelocityKF(dt);
CKF = VelocityErrorKF(dt);
CKF.setTrackWidth(b);
kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .05; kf_sigma_wdot = .05;
kf_sigma_vRoff = 0.1; kf_sigma_vLoff = 0.1;
kf_sigma_v_enc = .5; kf_sigma_w_enc = 1.5;
kf_sigma_w_gyro1 = .8; kf_sigma_v_obs1 = .8;
kf_sigma_w_gyro2 = .4; kf_sigma_v_obs2 = .4;
CKF.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot, kf_sigma_vRoff, kf_sigma_vLoff);
CKF.setMeasurementNoiseEncoder(kf_sigma_v_enc, kf_sigma_w_enc);
CKF.setMeasurementNoiseGyro(kf_sigma_w_gyro1);
CKF.setMeasurementNoiseVel(kf_sigma_v_obs1);
% FKF1 = VelocityKF(dt);
% FKF2 = VelocityKF(dt);
% FKF3 = VelocityKF(dt);
% kf_sigma_v_enc = .5; kf_sigma_w_enc = 1.5;
% kf_sigma_w_gyro1 = .3; kf_sigma_v_obs1 = .3;
% kf_sigma_w_gyro2 = .3; kf_sigma_v_obs2 = .3;
% kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .02; kf_sigma_wdot = .02;
% FKF1.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
% kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .02; kf_sigma_wdot = .02;
% FKF2.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
% FKF3.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
% FKF1.setMeasurementNoiseEncoder(kf_sigma_v_enc, kf_sigma_w_enc);
% FKF2.setMeasurementNoiseGyro(kf_sigma_w_gyro1);
% FKF2.setMeasurementNoiseVel(kf_sigma_v_obs1);
% FKF3.setMeasurementNoiseGyro(kf_sigma_w_gyro2);
% FKF3.setMeasurementNoiseVel(kf_sigma_v_obs2);

% Initialize the history
len = T/dt;
hist_ckf = zeros(len+1,6);
hist_ckf_p = zeros(len+1,6,6);
hist_state = zeros(len+1,3);
hist_wheelvel = zeros(len+1,4);
hist_vel = zeros(len+1,8);
hist_odom = zeros(len+1,3);
hist_part  = zeros(len+1,3,n_part);
hist_state(1,:) = x_true;
hist_odom(1,:)  = x_odom;

% GENERATE TRACK
track = zeros(len,2);
track(1:end,1) = 1;
track(1:end,2) = .15;
% track(1:end,1) = .1;
% track(1:end,2) = .1;
% % % track(1:end,1) = 1;
% % % track(1:end,2) = .1;
% % % track(500:end,1) = .1;
% % % track(500:end,2) = .1;
% track(1:40/dt,1) = 1;     % Demo velocity
% track(1:40/dt,2) = 0;    % Demo omega
% track(40/dt:50/dt,1) = 1;     % Demo velocity
% track(40/dt:50/dt,2) = .1;    % Demo omega
% track(50/dt:70/dt,1) = 1;     % Demo velocity
% track(50/dt:70/dt,2) = 0;    % Demo omega
% track(70/dt:80/dt,1) = .5;     % Demo velocity
% track(70/dt:80/dt,2) = -.5;    % Demo omega
% track(80/dt:end,1) = 1;     % Demo velocity
% track(80/dt:end,2) = 0;    % Demo omega

U1 = 0; U2 = 0; U3 = 0;
Ualpha = 0.9;
Cavg = 0;
Cnum = 0;
tic
for i = 1:len
    
%     if i == 20/dt
%         sigma_enc = sigma_enc*5;
%     end
    
    % WHEEL VELOCITIES
    V = track(i,1);
    w = track(i,2);
    Vr = V + b*w/2; %Calculate Vr
    Vl = V - b*w/2; %Calculate Vl
    
    % ACTUATOR NOISE
    % Simulate the imperfect actuator by adding noise to Vl and Vr
    Vr_true = Vr + Vr_sigma*randn;
    Vl_true = Vl + Vl_sigma*randn;
    v_true = (Vr_true + Vl_true)/2;
    w_true = (Vr_true - Vl_true)/b;
    
    % SIMULATE TRUE ROBOT MOTION
    x_true = SimulateMotion(Vr_true,Vl_true,x_true,b,dt);
    hist_state(i+1,:) = x_true';
    
    % PREDICTION: (every time step)
    % Update odometry map
    Dr = Vr_true*dt + sigma_enc*randn; % Measure encoder displacement
    Dl = Vl_true*dt + sigma_enc*randn;
    % Add an encoder fault
    [Dr, Dl] = SimulateEncoderFault(Dr,Dl,i,dt);
    
    % Perform Velocity Measurement Update
    v_enc = (Dr/dt + Dl/dt)/2;
    w_enc = (Dr/dt - Dl/dt)/b;
    w_gyro1 = (Vr-Vl)/b + sigma_gyro1*randn;
    v_obs1 = (Vr+Vl)/2 + sigma_vel1*randn;
    w_gyro2 = (Vr-Vl)/b + sigma_gyro2*randn;
    v_obs2 = (Vr+Vl)/2 + sigma_vel2*randn;
    CKF.update();
    CKF.measureEncoder(v_enc,w_enc);
    CKF.setMeasurementNoiseGyro(kf_sigma_w_gyro1);
    CKF.setMeasurementNoiseVel(kf_sigma_v_obs1);
%     CKF.measureVel(v_obs1);
    CKF.measureGyro(w_gyro1);
%     CKF.setMeasurementNoiseGyro(kf_sigma_w_gyro2);
%     CKF.setMeasurementNoiseVel(kf_sigma_v_obs2);
%     CKF.measureVelGyro(v_obs2,w_gyro2);
    
    hist_vel(i+1,:) = [v_true;w_true;v_enc;w_enc;v_obs1;w_gyro1;v_obs1;w_gyro1];
    hist_wheelvel(i+1,:) = [Vr_true;Vl_true;Dr/dt;Dl/dt];
    hist_ckf(i+1,:) = CKF.x';
    hist_ckf_p(i+1,:,:) = CKF.p;
    
    % Integrate velocity to find odometry data
    kf_v = CKF.x(1);
    kf_w = CKF.x(2);
%     kf_v = Xtot(1);
%     kf_w = Xtot(2);
%     x_odom = SimulateMotion(Dr/dt,Dl/dt,x_odom,b,dt);
    x_odom = SimulateMotion(kf_v+b*kf_w/2,kf_v-b*kf_w/2,x_odom,b,dt);
    hist_odom(i+1,:) = x_odom';
        
    % Update the particle filter every 1 Hz (for now)
    if mod(i,1/dt) == 0
        % Create the GPS Measurement (of GPS LEVER ARM)
        alpha = .4;
        GPS(1) = x_true(1) + alpha*cos(x_true(3)) + randn(1)*sigma_gps;
        GPS(2) = x_true(2) + alpha*sin(x_true(3)) + randn(1)*sigma_gps;
        
        % Initialize weights
        particles(4,:) = 1;
        
        % Update the particle filter
        for p_index = 1:n_part
            particles(1:3,p_index) = sample_motion_model_odometry([x_odom_old;x_odom],particles(:,p_index));
            % gps measurements
            particles(4,p_index) = particles(4,p_index) * measurement_model_gps_leverarm(GPS,particles(:,p_index),sigma_gps,alpha);
        end
        
        % Resample
        particles(4,:) = particles(4,:)/sum(particles(4,:));
        new_particles = resample_low_variance(particles);
        
        % Diplay histograms
%         figure(2)
%         hist(particles(4,:),20);
%         figure(3)
%         hist(new_particles(4,:),20);

        % Store the particles
        particles = new_particles;
        
        x_odom_old = x_odom;
    end
    
    % DISPLAY
    if mod(i,5/dt) == 0
        figure(1)
        clf; hold on;
        axis([-100 100 -100 100])
%         axis([-30 30 -30 30])
        plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3);
        plot(hist_odom(1:i+1,1),hist_odom(1:i+1,2),'r-x');
        plot(particles(1,:),particles(2,:),'xm','LineWidth',1);
        plot(map(1,1:2),map(2,1:2),'xk','LineWidth',5);
        title('True Path (blue), Odometry (red), Particles (magenta)');
    end
    
    if (mod(i,20/dt) == 0)
%         pause
    end
end
toc


%% Evaluation
v_true = hist_vel(:,1);
w_true = hist_vel(:,2);
v_ckf  = hist_ckf(:,1);
w_ckf  = hist_ckf(:,2);
v_rms_ckf = sqrt((v_true-v_ckf)'*(v_true-v_ckf)/length(v_true))
w_rms_ckf = sqrt((w_true-w_ckf)'*(w_true-w_ckf)/length(w_true))

%% Plots
close all
figure
clf; hold on;
axis([-100 100 -100 100])
plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3)
plot(hist_odom(1:i+1,1),hist_odom(1:i+1,2),'r-x')
plot(particles(1,:),particles(2,:),'xm','LineWidth',1)
title('True Path (blue), Odometry (red), Particles (magenta)');

% Display CKF
figure
subplot(2,1,1)
t = 0:dt:T;
v_err = 3*sqrt(hist_ckf_p(:,1,1));
plot(t,hist_ckf(:,1),'r')%,t,hist_vel(:,6),'g',t,hist_vel(:,3),'b');
hold on
plot(t,hist_ckf(:,1)+v_err,'b-',t,hist_ckf(:,1)-v_err,'b-')
% plot(t,hist_vel(:,1),'m');
plot(t,hist_vel(:,3),'g');
title('CKF Velocity (r) and Encoder Velocity (g)');

subplot(2,1,2)
w_err = 3*sqrt(hist_ckf_p(:,2,2));
plot(t,hist_ckf(:,2),'r')%,t,hist_vel(:,4),'b',t,hist_vel(:,5),'g');
hold on
plot(t,hist_ckf(:,2)+w_err,'b-',t,hist_ckf(:,2)-w_err,'b-')
% plot(t,hist_vel(:,2),'m');
plot(t,hist_vel(:,4),'g');
title('CKF Omega and Encoder Omega (g)')


% Display CKF's Encoder Error States
figure
subplot(2,1,1)
t = 0:dt:T;
voff_err = 3*sqrt(hist_ckf_p(:,5,5));
plot(t,hist_ckf(:,5),'r')%,t,hist_vel(:,6),'g',t,hist_vel(:,3),'b');
hold on
plot(t,hist_ckf(:,5)+voff_err,'b-',t,hist_ckf(:,5)-voff_err,'b-')
plot(t,hist_wheelvel(:,3)-hist_wheelvel(:,1),'g');
title('Estimated Right-Wheel Velocity Error (r) and Actual Error (g)');

subplot(2,1,2)
woff_err = 3*sqrt(hist_ckf_p(:,6,6));
plot(t,hist_ckf(:,6),'r')%,t,hist_vel(:,4),'b',t,hist_vel(:,5),'g');
hold on
plot(t,hist_ckf(:,6)+woff_err,'b-',t,hist_ckf(:,6)-woff_err,'b-')
plot(t,hist_wheelvel(:,4)-hist_wheelvel(:,2),'g');
title('Estimated Left-Wheel Velocity Error (r) and Actual Error (g)')


% % Adaptive R estimates
% figure
% subplot(2,1,1)
% plot(t,hist_Rest(:,1))
% title('Experimental Measurement noise: Encoder Velocity')
% subplot(2,1,2)
% plot(t,hist_Rest(:,2))
% title('Experimental Measurement noise: Encoder Omega')

% % % Display FKF 1
% % figure
% % subplot(2,1,1)
% % t = 0:dt:T;
% % v_err = 3*sqrt(hist_fkf1_p(:,1,1));
% % plot(t,hist_fkf1(:,1),'r')%,t,hist_vel(:,6),'g',t,hist_vel(:,3),'b');
% % hold on
% % plot(t,hist_fkf1(:,1)+v_err,'b-',t,hist_fkf1(:,1)-v_err,'b-')
% % plot(t,hist_vel(:,1),'m');
% % title('FKF1 (odom filter) Velocity');
% % 
% % subplot(2,1,2)
% % w_err = 3*sqrt(hist_fkf1_p(:,1,1));
% % plot(t,hist_fkf1(:,2),'r')%,t,hist_vel(:,4),'b',t,hist_vel(:,5),'g');
% % hold on
% % plot(t,hist_fkf1(:,2)+w_err,'b-',t,hist_fkf1(:,2)-w_err,'b-')
% % plot(t,hist_vel(:,2),'m');
% % title('FKF1 (odom filter) Omega')
% % 
% % % Display FKF 2
% % figure
% % subplot(2,1,1)
% % t = 0:dt:T;
% % v_err = 3*sqrt(hist_fkf2_p(:,1,1));
% % plot(t,hist_fkf2(:,1),'r')%,t,hist_vel(:,6),'g',t,hist_vel(:,3),'b');
% % hold on
% % plot(t,hist_fkf2(:,1)+v_err,'b-',t,hist_fkf2(:,1)-v_err,'b-')
% % plot(t,hist_vel(:,1),'m');
% % title('FKF2 (aux filter) Velocity');
% % 
% % subplot(2,1,2)
% % w_err = 3*sqrt(hist_fkf2_p(:,1,1));
% % plot(t,hist_fkf2(:,2),'r')%,t,hist_vel(:,4),'b',t,hist_vel(:,5),'g');
% % hold on
% % plot(t,hist_fkf2(:,2)+w_err,'b-',t,hist_fkf2(:,2)-w_err,'b-')
% % plot(t,hist_vel(:,2),'m');
% % title('FKF2 (aux filter) Omega')
% % 
% % % Display FKF 3
% % figure
% % subplot(2,1,1)
% % t = 0:dt:T;
% % v_err = 3*sqrt(hist_fkf3_p(:,1,1));
% % plot(t,hist_fkf3(:,1),'r')%,t,hist_vel(:,4),'g',t,hist_vel(:,1),'b');
% % hold on
% % plot(t,hist_fkf3(:,1)+v_err,'b-',t,hist_fkf3(:,1)-v_err,'b-')
% % plot(t,hist_vel(:,1),'m');
% % title('FKF3 (aux filter) Velocity');
% % 
% % subplot(2,1,2)
% % w_err = 3*sqrt(hist_fkf3_p(:,1,1));
% % plot(t,hist_fkf3(:,2),'r')%,t,hist_vel(:,2),'b',t,hist_vel(:,3),'g');
% % hold on
% % plot(t,hist_fkf3(:,2)+w_err,'b-',t,hist_fkf3(:,2)-w_err,'b-')
% % plot(t,hist_vel(:,2),'m');
% % title('FKF3 (aux filter) Omega')
% % 
% % % Display Fused FKF
% % figure
% % subplot(2,1,1)
% % t = 0:dt:T;
% % v_err = 3*sqrt(hist_tot_p(:,1,1));
% % plot(t,hist_tot(:,1),'r')%,t,hist_vel(:,4),'g',t,hist_vel(:,1),'b');
% % hold on
% % plot(t,hist_tot(:,1)+v_err,'b-',t,hist_tot(:,1)-v_err,'b-')
% % plot(t,hist_vel(:,1),'m');
% % title('Total Velocity');
% % 
% % subplot(2,1,2)
% % w_err = 3*sqrt(hist_tot_p(:,1,1));
% % plot(t,hist_tot(:,2),'r')%,t,hist_vel(:,2),'b',t,hist_vel(:,3),'g');
% % hold on
% % plot(t,hist_tot(:,2)+w_err,'b-',t,hist_tot(:,2)-w_err,'b-')
% % plot(t,hist_vel(:,2),'m');
% % title('Total Omega')
% % 
% % % Display Track quality
% % figure
% % subplot(2,1,1)
% % t = 0:dt:T;
% % % plot(t,hist_quality(:,3),t,hist_quality(:,4))%,t,hist_quality(:,3))
% % plot(t,hist_quality(:,1),t,hist_quality(:,2))%,t,hist_quality(:,3))
% % title('Quality Factors: FKF1 (b), FKF2 (g)')%, FKF3 (r)')
% % 
% % subplot(2,1,2)
% % plot(t,hist_fusion(:,1));
% % ylim([-1 2])
% % title('Encoders Fused in Master Filter?')



