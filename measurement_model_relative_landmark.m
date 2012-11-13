function [ q ] = measurement_model_relative_landmark(f,c,x,m,sigma_r,sigma_psi)
%MEASUREMENT_MODEL_LANDMARD generates the probability that a landmark would
%produce the given reading to the state
%   f: measurement [r psi s]
%   c: landmark identity (index value)
%   x: landmark particle state [x y]
%   m: map with all landmarks [ [x;y] [x;y] ... [x;y] ]
%   sigma_r: standard deviation in range
%   sigma_psi: standard deviation in angle

r   = f(1);
psi = f(2);

r_hat   = sqrt(x(1)^2+x(2)^2);
psi_hat = CoerceAngle(atan2(x(2),x(1))-pi/2);

% q = normpdf(AngleDifference(psi,psi_hat),0,sigma_psi);
% q = normpdf(r-r_hat,0,sigma_r)*normpdf(AngleDifference(psi,psi_hat),0,sigma_psi);
q = normpdf(r-r_hat,0,sigma_r);

end

