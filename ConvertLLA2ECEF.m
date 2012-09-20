
function [ecef] = ConvertLLA2ECEF(lla)
%ConvertLLA2ECEF converts a single lat, lon, alt point to ECEF
%  lla(1)- latitude in radians
%  lla(2)- longitude in radians
%  lla(3)- height in meters

lat = lla(1);
lon = lla(2);
alt = lla(3);

a = 6378137;        %meters, Semi Major Axis
b = 6356752.3142;   %meters, Semi Minor Axis
f = (a-b)/a;        %Flattening
e2 = 2*f-f*f;       %First eccentricity SQUARED
Norm = a / sqrt(1 - e2*sin(lat)^2);

x = (Norm + alt)*cos(lat)*cos(lon);
y = (Norm + alt)*cos(lat)*sin(lon);
z = (Norm*(1-e2)+alt)*sin(lat);

ecef = [x,y,z];
