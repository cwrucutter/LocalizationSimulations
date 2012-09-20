lat = gps(1,1)*pi/180;
lon = gps(1,2)*pi/180;
alt = gps(1,3);

R_lla = [lat,lon,alt];
R_ecef = ConvertLLA2ECEF(R_lla);

[r,c] = size(gps);
track = zeros(r,3);
for i = 2:r
    P_lla = [gps(i,1)*pi/180, gps(i,2)*pi/180, gps(i,3)];
    P_ecef = ConvertLLA2ECEF(P_lla);
    P_enu = ConvertECEF2ENU(P_ecef,R_ecef,R_lla);    
    
    track(i,:) = P_enu;
end

plot(track(:,1),track(:,2),'bx-');