function [ q ] = measurement_model_gps( zk, xk, gps_std )
%MEASUREMENT_MODEL_GPS Generates the probability of receiving the
%measurement zk given the state xk (gaussian distribution)
%   zk: measurement
%   xk: state
%   gps_std: gps standard deviation

% use a gaussian PDF to find the probability of receiving the measurement
% zk assuming the actual state is xk
q = normpdf(zk(1)-xk(1),0,gps_std)*normpdf(zk(2)-xk(2),0,gps_std);
% q = normpdf(sqrt((zk(2)-xk(2))^2+(zk(1)-xk(1))^2),0,gps_std);

end

