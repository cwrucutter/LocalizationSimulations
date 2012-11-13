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

q = normpdf(x-x_hat,0,gps_std)*normpdf(y-y_hat,0,gps_std);

end

