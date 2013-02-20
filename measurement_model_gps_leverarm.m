function [ q ] = measurement_model_gps_leverarm( zk, xk, gps_std, alpha )
%MEASUREMENT_MODEL_GPS Generates the probability of receiving the 
%gps measurement zk given the state xk (gaussian distribution)
%   zk: gps measurement
%   xk: state
%   gps_std: gps standard deviation
%   alpha: gps lever arm offset

% use a gaussian PDF to find the probability of receiving the measurement
% zk assuming the actual state is xk
x = zk(1);
y = zk(2);
x_hat = xk(1) + alpha*cos(xk(3));
y_hat = xk(2) + alpha*sin(xk(3));

pz = 0;
z1 = x-x_hat;
z2 = y-y_hat;
pz = pz + exp(-(z1*z1)/(2*gps_std*gps_std));
pz = pz + exp(-(z2*z2)/(2*gps_std*gps_std));
q = pz;

% q = normpdf(x-x_hat,0,gps_std)*normpdf(y-y_hat,0,gps_std);

end

