
%% Robot Differential Drive Particle Filter v1.0
% EJ Kreinar
clear all
close all

dt = .1;    %Dt
T = 300;     % Sim time
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
Vr_sigma = 0; % Actuator Noise on the encoder left
Vl_sigma = 0; % Actuator Noise on the encoder right

% ENCODER MEASUREMENT
sigma_enc = .0004; % make this speed-dependent?

% GPS MEASUREMENT
H_gps = [ 1 0 0 ;
    0 1 0 ;
    0 0 1 ];
sigma_gps = .05;
sigma_head = .1;
V_gps = [ sigma_gps^2 0 0; 0 sigma_gps^2 0; 0 0 sigma_head^2];
timestep = 1;
Vk_gps = V_gps*timestep;

% LANDMARK MEASUREMENT
map = [ [20;40] [-20;40] [0;80]];
[dim nland] = size(map);
sigma_r = .5;
sigma_psi = .1;

% Initialize the history
len = T/dt;
hist_state = zeros(len+1,3);
hist_odom = zeros(len+1,3);
hist_part  = zeros(len+1,3,n_part);
hist_state(1,:) = x_true;
hist_odom(1,:)  = x_odom;

% GENERATE TRACK
track = zeros(len,2);
track(1:end,1) = 1;
track(1:end,2) = .02;
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
    x_odom = SimulateMotion(Dr/dt,Dl/dt,x_odom,b,dt);
    hist_odom(i+1,:) = x_odom';
        
    % Update the particle filter every 1 Hz (for now)
    if mod(i,1/dt) == 0
        % Create the GPS Measurement
        H = [H_gps];    % Create sensitivity matrix
        Rk = [Vk_gps];  % Create noise matrix
        noise = sqrt(Rk)*randn(length(Rk),1);
        Z     = H*x_true + noise;       % Take the measurement, adding simulated noise using randn
        
        % Create the Landmark measurements
        f1(1) = sqrt((map(1,1)-x_true(1))^2+(map(2,1)-x_true(2))^2)    + randn(1)*sigma_r;
        f1(2) = CoerceAngle(atan2(map(2,1)-x_true(2),map(1,1)-x_true(1))-x_true(3) + randn(1)*sigma_psi);
        f1(3) = 1;
        
        f2(1) = sqrt((map(1,2)-x_true(1))^2+(map(2,2)-x_true(2))^2)    + randn(1)*sigma_r;
        f2(2) = CoerceAngle(atan2(map(2,2)-x_true(2),map(1,2)-x_true(1))-x_true(3) + randn(1)*sigma_psi);
        f2(3) = 2;
        
        f3(1) = sqrt((map(1,3)-x_true(1))^2+(map(2,3)-x_true(2))^2)    + randn(1)*sigma_r;
        f3(2) = CoerceAngle(atan2(map(2,3)-x_true(2),map(1,3)-x_true(1))-x_true(3) + randn(1)*sigma_psi);
        f3(3) = 3;
        
        % Initialize weights
        particles(4,:) = 1;
        
        % Update the particle filter
        for p_index = 1:n_part
            particles(1:3,p_index) = sample_motion_model_odometry([x_odom_old;x_odom],particles(:,p_index));
            % gps measurements
            particles(4,p_index) = particles(4,p_index) * measurement_model_gps(Z,particles(:,p_index),sigma_gps);
            % Landmark measurements
            particles(4,p_index) = particles(4,p_index) * measurement_model_landmark(f1,1,particles(:,p_index),map,sigma_r,sigma_psi);
            particles(4,p_index) = particles(4,p_index) * measurement_model_landmark(f2,2,particles(:,p_index),map,sigma_r,sigma_psi);
            particles(4,p_index) = particles(4,p_index) * measurement_model_landmark(f3,3,particles(:,p_index),map,sigma_r,sigma_psi);
        end
        
        % Resample
        particles(4,:) = particles(4,:)/sum(particles(4,:));
        new_particles = resample_low_variance(particles);
        
        % Diplay histograms
        figure(2)
        hist(particles(4,:),20);
        figure(3)
        hist(new_particles(4,:),20);
        particles = new_particles;
        
        x_odom_old = x_odom;
    end
    
    % DISPLAY
    if mod(i,1/dt) == 0
        figure(1)
        clf; hold on;
        axis([-100 100 -100 100])
        plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3)
        plot(hist_odom(1:i+1,1),hist_odom(1:i+1,2),'r-x')
        plot(particles(1,:),particles(2,:),'xm','LineWidth',1)
    end
end
toc

%% Plots
close all
figure(1)
clf; hold on;
axis([-100 100 -100 100])
plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3)
plot(hist_odom(1:i+1,1),hist_odom(1:i+1,2),'r-x')
plot(particles(1,:),particles(2,:),'xm','LineWidth',1)
title('True track (b) vs Esimated track (r)');

