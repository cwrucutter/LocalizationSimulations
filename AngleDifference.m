function [ angle ] = AngleDifference(a,b)
%ANGLEDIFFERENCE Finds the difference between the two angles a and b
% Right hand orientation representation of b-a. Angle to go FROM a to b. 
%   a- angle 1 in radians (0,2pi)
%   b- angle 2 in radians (0,2pi)

a = mod(a,2*pi); % Coerce between 0 - 2pi
b = mod(b,2*pi); % Coerce between 0 - 2pi

a1 = b-a;                           % Find the angle difference
a2 = (2*pi-abs(a1)).*sign(a1)*-1;      % Find the complementary angle difference

% Use the angle with the smaller magnitude
angle = a1;
index = abs(a2)<abs(a1);
angle(index) = a2(index);

end

