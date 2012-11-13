%% Basic Simulation

% INITIALIZE TIMING
dt = .1;    %Dt
T = 10;     % Sim time
b = .5;     %Track Width

% INITIAL VALUES for the robot state
x0 = 0;     % Initial x
y0 = 0;     % Initial y
tht0 = .1;   % Initial theta
x_true = [x0; y0; tht0];

% INITIALIZE PATH
waypoints = [ [0;0] [10;0] [10;10] ]

% Initialize the history
len = T/dt;
hist_state = zeros(len+1,3);
hist_state(1,:) = x_true;

for i=1:len
    
    % Choose values for the new Vr and Vl
    Vr = 1;
    Vl = 1;
    
    % Simulates the motion here- save the new state
    x_true = SimulateMotion(Vr,Vl,x_true,b,dt)
    hist_state(i+1,:) = x_true;
    
end


%% Plots
close all
figure(1)
clf; hold on;
plot(hist_state(1:i+1,1),hist_state(1:i+1,2),'b','LineWidth',3)
plot(waypoints(1,:),waypoints(2,:),'r')
title('True Path (blue)');
