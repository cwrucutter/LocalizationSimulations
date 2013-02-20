
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
sigma_enc = .005; % make this speed-dependent?
sigma_gyro = 0.01;
sigma_vel = 0.01;

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

% Initialize the history
len = T/dt;
hist_state = zeros(len+1,3);
hist_vkf = zeros(len+1,4);
hist_vkf_p = zeros(len+1,4,4);
hist_vel = zeros(len+1,4);
hist_odom = zeros(len+1,3);
hist_part  = zeros(len+1,3,n_part);
hist_state(1,:) = x_true;
hist_odom(1,:)  = x_odom;

% Initialize the Velocity Kalman Pre-Filter
vKF = VelocityKF(dt);
kf_sigma_v = 0.001; kf_sigma_w = 0.001; kf_sigma_vdot = .05; kf_sigma_wdot = .01;
kf_sigma_v_enc = .1; kf_sigma_w_enc = .1;
kf_sigma_w_gyro = .01;
kf_sigma_v_obs = .05;
vKF.setProcessNoise(kf_sigma_v, kf_sigma_w, kf_sigma_vdot, kf_sigma_wdot);
vKF.setMeasurementNoiseEncoder(kf_sigma_v_enc, kf_sigma_w_enc);
vKF.setMeasurementNoiseGyro(kf_sigma_w_gyro);
vKF.setMeasurementNoiseVel(kf_sigma_v_obs);

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
    w_gyro = (Vr-Vl)/b + sigma_gyro*randn;
    v_obs = (Vr+Vl)/2 + sigma_vel*randn;
    vKF.update();
    vKF.measureEncoder(v_enc,w_enc);
    vKF.measureGyro(w_gyro);
    vKF.measureVel(v_obs);
    hist_vkf(i+1,:) = vKF.x';
    hist_vel(i+1,:) = [v_enc;w_enc;w_gyro;v_obs];
    hist_vkf_p(i+1,:,:) = vKF.p;
    
    % Integrate velocity to find odometry data
    kf_v = vKF.x(1);
    kf_w = vKF.x(2);
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

%% Plots
close all
figure
clf; hold on;
axis([-100 100 -100 100])
plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3)
plot(hist_odom(1:i+1,1),hist_odom(1:i+1,2),'r-x')
plot(particles(1,:),particles(2,:),'xm','LineWidth',1)
title('True Path (blue), Odometry (red), Particles (magenta)');

figure
t = 0:dt:T;
v_err = 3*sqrt(hist_vkf_p(:,1,1));
plot(t,hist_vkf(:,1),'r')%,t,hist_vel(:,4),'g',t,hist_vel(:,1),'b');
hold on
plot(t,hist_vkf(:,1)+v_err,'b-',t,hist_vkf(:,1)-v_err,'b-')
title('Encoder Velocity (blue), Observed Velocity (green), KF Velocity (red)');

figure
t = 0:dt:T;
w_err = 3*sqrt(hist_vkf_p(:,1,1));
plot(t,hist_vkf(:,2),'r')%,t,hist_vel(:,2),'b',t,hist_vel(:,3),'g');
hold on
plot(t,hist_vkf(:,2)+w_err,'b-',t,hist_vkf(:,2)-w_err,'b-')
title('Encoder Omega (blue), Gyro Omega (green), KF Omega(red)')

