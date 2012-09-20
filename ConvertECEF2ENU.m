
function P_enu = ConvertECEF2ENU(P_ecef,R_ecef,R_lla)
%ConvertECEF2ENU converts a point to enu with respect to a reference pt
%  P_ecef- (x,y,z) in meters of the desired point to convert 
%  R_ecef- (x,y,z) in meters of the reference point
%  R_lla - (lat,lon,alt) in (rad, rad, m) of the reference point

lat = R_lla(1);
lon = R_lla(2);

A_ecef2enu = [     -sin(lon)    ,      cos(lon)     ,    0     ;
              -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat) ;
              cos(lat)*cos(lon) , cos(lat)*sin(lon) , sin(lat) ];
          
P_enu = A_ecef2enu * (P_ecef' - R_ecef');