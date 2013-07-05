
t = 0:.1:100;

verr1 = zeros(length(t),1);
werr1 = zeros(length(t),1);
verr1(t>70 & t<80)=0.1/2;
werr1(t>70 & t<80)=0.1/.5;

verr2 = zeros(length(t),1);
werr2 = zeros(length(t),1);
verr2(t>20 & t<30)=.3;
werr2(t>10 & t<30)=0;


figure(1)
subplot(2,1,1)
plot(t,verr1,'m','Linewidth',2);
title('Fault 1: Forward Velocity Error')
xlabel('Time (s)')
ylabel('Forward Velocity (m/s)')
ylim([-0.1 0.4])
subplot(2,1,2)
plot(t,werr1,'m','Linewidth',2);
title('Fault 1: Angular Velocity Error')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
ylim([-0.4 0.4])


figure(2)
subplot(2,1,1)
plot(t,verr2,'m','Linewidth',2);
title('Fault 2: Forward Velocity Error')
xlabel('Time (s)')
ylabel('Forward Velocity (m/s)')
ylim([-0.1 0.4])
subplot(2,1,2)
plot(t,werr2,'m','Linewidth',2);
title('Fault 2: Angular Velocity Error')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
ylim([-0.4 0.4])