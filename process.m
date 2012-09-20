gps_all = [m(:,23:25) m(:,28:30)];
gps = [];
for i = 1:6
   gps =  [gps gps_all(gps_all(:,i)~=0,i)];
end
gps(:,1:2) = gps(:,1:2)/10000000;
gps(:,6) = gps(:,6)/100;