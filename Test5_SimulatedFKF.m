
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
sigma_gyro1 = 0.05;
sigma_vel1 = 0.05;
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
% vKF = VelocityKF(dt);
% kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .05; kf_sigma_wdot = .05;
% kf_sigma_v_enc = .1; kf_sigma_w_enc = 1;
% kf_sigma_w_gyro = .1;
% kf_sigma_v_obs = .1;
% vKF.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
% vKF.setMeasurementNoiseEncoder(kf_sigma_v_enc, kf_sigma_w_enc);
% vKF.setMeasurementNoiseGyro(kf_sigma_w_gyro);
% vKF.setMeasurementNoiseVel(kf_sigma_v_obs);
Xtot = [0;0;0;0]; Ptot = zeros(4,4);
FKF1 = VelocityKF(dt);
FKF2 = VelocityKF(dt);
FKF3 = VelocityKF(dt);
kf_sigma_v_enc = .5; kf_sigma_w_enc = 1;
kf_sigma_w_gyro1 = .1; kf_sigma_v_obs1 = .1;
kf_sigma_w_gyro2 = .1; kf_sigma_v_obs2 = .1;
kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .02; kf_sigma_wdot = .05;
FKF1.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .02; kf_sigma_wdot = .02;
FKF2.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
FKF3.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
FKF1.setMeasurementNoiseEncoder(kf_sigma_v_enc, kf_sigma_w_enc);
FKF2.setMeasurementNoiseGyro(kf_sigma_w_gyro1);
FKF2.setMeasurementNoiseVel(kf_sigma_v_obs1);
FKF3.setMeasurementNoiseGyro(kf_sigma_w_gyro2);
FKF3.setMeasurementNoiseVel(kf_sigma_v_obs2);

% Initialize the history
len = T/dt;
hist_fkf1 = zeros(len+1,4);
hist_fkf1_p = zeros(len+1,4,4);
hist_fkf2 = zeros(len+1,4);
hist_fkf2_p = zeros(len+1,4,4);
hist_fkf3 = zeros(len+1,4);
hist_fkf3_p = zeros(len+1,4,4);
hist_tot = zeros(len+1,4);
hist_tot_p = zeros(len+1,4,4);
hist_quality = zeros(len+1,4);
hist_state = zeros(len+1,3);
hist_vel = zeros(len+1,8);
hist_odom = zeros(len+1,3);
hist_part  = zeros(len+1,3,n_part);
hist_state(1,:) = x_true;
hist_odom(1,:)  = x_odom;

% GENERATE TRACK
track = zeros(len,2);
track(1:end,1) = 1;
track(1:end,2) = .1;
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
tic
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
    v_true = (Vr_true + Vl_true)/2;
    w_true = (Vr_true - Vl_true)/b;
    
    % SIMULATE TRUE ROBOT MOTION
    x_true = SimulateMotion(Vr_true,Vl_true,x_true,b,dt);
    hist_state(i+1,:) = x_true';
    
    % PREDICTION: (every time step)
    % Update odometry map
    [vRoff, vLoff] = SimulateEncoderVelocityFault2(i,dt);
    Dr = Vr_true*dt + sigma_enc*randn + vRoff*dt; % Measure encoder displacement
    Dl = Vl_true*dt + sigma_enc*randn + vLoff*dt;
    % Add an encoder fault
%     [Dr, Dl] = SimulateEncoderFault(Dr,Dl,i,dt);
    
    % Perform Velocity Measurement Update
    v_enc = (Dr/dt + Dl/dt)/2;
    w_enc = (Dr/dt - Dl/dt)/b;
    w_gyro1 = (Vr-Vl)/b + sigma_gyro1*randn;
    v_obs1 = (Vr+Vl)/2 + sigma_vel1*randn;
    w_gyro2 = (Vr-Vl)/b + sigma_gyro2*randn;
    v_obs2 = (Vr+Vl)/2 + sigma_vel2*randn;
    FKF1.update();
    FKF2.update();
    FKF3.update();
    d1 = 0; d2 = 0; d3 = 0;
    FKF1.measureEncoder(v_enc,w_enc);
    S1 = [1 0 0 0; 0 1 0 0]*FKF1.p*[1 0 0 0; 0 1 0 0]' + FKF1.Rk_enc;
%     d1 = d1 + ([FKF1.x(1);FKF1.x(2)] - [Xtot(1);Xtot(2)])'*inv(S1)*([FKF1.x(1);FKF1.x(2)] - [Xtot(1);Xtot(2)]);
    FKF2.measureGyro(w_gyro1);
    FKF2.measureVel(v_obs1);
    S2 = [1 0 0 0; 0 1 0 0]*FKF2.p*[1 0 0 0; 0 1 0 0]' + [FKF2.Rk_vel 0; 0 FKF2.Rk_gyro];
%     d2 = d2 + ([FKF2.x(1);FKF2.x(2)] - [Xtot(1);Xtot(2)])'*inv(S2)*([FKF2.x(1);FKF2.x(2)] - [Xtot(1);Xtot(2)]);
    FKF3.measureGyro(w_gyro2);
    FKF3.measureVel(v_obs2);
    S3 = [1 0 0 0; 0 1 0 0]*FKF3.p*[1 0 0 0; 0 1 0 0]' + [FKF3.Rk_vel 0; 0 FKF3.Rk_gyro];
%     d3 = d3 + ([FKF3.x(1);FKF3.x(2)] - [Xtot(1);Xtot(2)])'*inv(S3)*([FKF3.x(1);FKF3.x(2)] - [Xtot(1);Xtot(2)]);
    d1 = d1 + ([FKF1.x(1);FKF1.x(2)] - [FKF2.x(1);FKF2.x(2)])'*inv(S1+S2)*([FKF1.x(1);FKF1.x(2)] - [FKF2.x(1);FKF2.x(2)]);
    d2 = d2 + ([FKF1.x(1);FKF1.x(2)] - [FKF3.x(1);FKF3.x(2)])'*inv(S1+S3)*([FKF1.x(1);FKF1.x(2)] - [FKF3.x(1);FKF3.x(2)]);
    U1 = Ualpha*d1 + (1-Ualpha)*U1;
    U2 = Ualpha*d2 + (1-Ualpha)*U2;
%     U3 = Ualpha*d3 + (1-Ualpha)*U3;
    Umin = min(U1,min(U2,U3));
    Q1 = U1/Umin;
    Q2 = U2/Umin;
%     Q3 = U3/Umin;
    hist_quality(i+1,:)=[Q1;Q2;U1;U2];%;Q3];
    
    if (U1 > 2)
        Ptot = inv(inv(FKF2.p)+inv(FKF3.p));
        Xtot = Ptot*(FKF2.p\FKF2.x + FKF3.p\FKF3.x);
    else
        Ptot = inv(inv(FKF1.p)+inv(FKF2.p)+inv(FKF3.p));
        Xtot = Ptot*(FKF1.p\FKF1.x + FKF2.p\FKF2.x + FKF3.p\FKF3.x);
    end
    hist_vel(i+1,:) = [v_true;w_true;v_enc;w_enc;v_obs1;w_gyro1;v_obs1;w_gyro1];
    hist_fkf1(i+1,:) = FKF1.x';
    hist_fkf1_p(i+1,:,:) = FKF1.p;
    hist_fkf2(i+1,:) = FKF2.x';
    hist_fkf2_p(i+1,:,:) = FKF2.p;
    hist_fkf3(i+1,:) = FKF3.x';
    hist_fkf3_p(i+1,:,:) = FKF3.p;
    hist_tot(i+1,:) = Xtot;
    hist_tot_p(i+1,:,:) = Ptot;
    
    % Integrate velocity to find odometry data
%     kf_v = FKF1.x(1);
%     kf_w = FKF1.x(2);
    kf_v = Xtot(1);
    kf_w = Xtot(2);
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
v_fkf1 = hist_fkf1(:,1);
w_fkf1 = hist_fkf1(:,2);
v_fkf2 = hist_fkf2(:,1);
w_fkf2 = hist_fkf2(:,2);
v_fkf3 = hist_fkf3(:,1);
w_fkf3 = hist_fkf3(:,2);
v_tot = hist_tot(:,1);
w_tot = hist_tot(:,2);
v_rms_fkf1 = sqrt((v_true-v_fkf1)'*(v_true-v_fkf1)/length(v_true))
v_rms_fkf2 = sqrt((v_true-v_fkf2)'*(v_true-v_fkf2)/length(v_true))
v_rms_fkf3 = sqrt((v_true-v_fkf3)'*(v_true-v_fkf3)/length(v_true))
v_rms_tot = sqrt((v_true-v_tot)'*(v_true-v_tot)/length(v_true))

%% Plots
close all
figure
clf; hold on;
axis([-100 100 -100 100])
plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3)
plot(hist_odom(1:i+1,1),hist_odom(1:i+1,2),'r-x')
plot(particles(1,:),particles(2,:),'xm','LineWidth',1)
title('True Path (blue), Odometry (red), Particles (magenta)');

% Display FKF 1
figure
subplot(2,1,1)
t = 0:dt:T;
v_err = 3*sqrt(hist_fkf1_p(:,1,1));
plot(t,hist_fkf1(:,1),'r')%,t,hist_vel(:,6),'g',t,hist_vel(:,3),'b');
hold on
plot(t,hist_fkf1(:,1)+v_err,'b-',t,hist_fkf1(:,1)-v_err,'b-')
plot(t,hist_vel(:,1),'m');
title('FKF1 (odom filter) Velocity');

subplot(2,1,2)
w_err = 3*sqrt(hist_fkf1_p(:,1,1));
plot(t,hist_fkf1(:,2),'r')%,t,hist_vel(:,4),'b',t,hist_vel(:,5),'g');
hold on
plot(t,hist_fkf1(:,2)+w_err,'b-',t,hist_fkf1(:,2)-w_err,'b-')
plot(t,hist_vel(:,2),'m');
title('FKF1 (odom filter) Omega')

% Display FKF 2
figure
subplot(2,1,1)
t = 0:dt:T;
v_err = 3*sqrt(hist_fkf2_p(:,1,1));
plot(t,hist_fkf2(:,1),'r')%,t,hist_vel(:,6),'g',t,hist_vel(:,3),'b');
hold on
plot(t,hist_fkf2(:,1)+v_err,'b-',t,hist_fkf2(:,1)-v_err,'b-')
plot(t,hist_vel(:,1),'m');
title('FKF2 (aux filter) Velocity');

subplot(2,1,2)
w_err = 3*sqrt(hist_fkf2_p(:,1,1));
plot(t,hist_fkf2(:,2),'r')%,t,hist_vel(:,4),'b',t,hist_vel(:,5),'g');
hold on
plot(t,hist_fkf2(:,2)+w_err,'b-',t,hist_fkf2(:,2)-w_err,'b-')
plot(t,hist_vel(:,2),'m');
title('FKF2 (aux filter) Omega')

% Display FKF 3
figure
subplot(2,1,1)
t = 0:dt:T;
v_err = 3*sqrt(hist_fkf3_p(:,1,1));
plot(t,hist_fkf3(:,1),'r')%,t,hist_vel(:,4),'g',t,hist_vel(:,1),'b');
hold on
plot(t,hist_fkf3(:,1)+v_err,'b-',t,hist_fkf3(:,1)-v_err,'b-')
plot(t,hist_vel(:,1),'m');
title('FKF3 (aux filter) Velocity');

subplot(2,1,2)
w_err = 3*sqrt(hist_fkf3_p(:,1,1));
plot(t,hist_fkf3(:,2),'r')%,t,hist_vel(:,2),'b',t,hist_vel(:,3),'g');
hold on
plot(t,hist_fkf3(:,2)+w_err,'b-',t,hist_fkf3(:,2)-w_err,'b-')
plot(t,hist_vel(:,2),'m');
title('FKF3 (aux filter) Omega')

% Display Fused FKF
figure
subplot(2,1,1)
t = 0:dt:T;
v_err = 3*sqrt(hist_tot_p(:,1,1));
plot(t,hist_tot(:,1),'r')%,t,hist_vel(:,4),'g',t,hist_vel(:,1),'b');
hold on
plot(t,hist_tot(:,1)+v_err,'b-',t,hist_tot(:,1)-v_err,'b-')
plot(t,hist_vel(:,1),'m');
title('Total Velocity');

subplot(2,1,2)
w_err = 3*sqrt(hist_tot_p(:,1,1));
plot(t,hist_tot(:,2),'r')%,t,hist_vel(:,2),'b',t,hist_vel(:,3),'g');
hold on
plot(t,hist_tot(:,2)+w_err,'b-',t,hist_tot(:,2)-w_err,'b-')
plot(t,hist_vel(:,2),'m');
title('Total Omega')

% Display Track quality
figure
subplot(2,1,1)
t = 0:dt:T;
plot(t,hist_quality(:,3),t,hist_quality(:,4))%,t,hist_quality(:,3))
title('Quality Factors: FKF1 (b), FKF2 (g), FKF3 (r)')

subplot(2,1,2)
plot(t,hist_quality(:,3)>2);
ylim([-1 2])
title('Encoders Fused in Master Filter?')



