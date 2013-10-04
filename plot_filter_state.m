function RMSerr = plot_filter_state(param, time, true, est, cov)
subplot(2,1,1)
plot(time, est(:,param), 'b', time, true(:,param), 'r');
subplot(2,1,2)
err = true(:,param)-est(:,param);
errTemp = err(1000:end);
RMSerr = sqrt((errTemp'*errTemp)/length(errTemp));
conf = 3*sqrt(cov(:,param,param));
plot(time, true(:,param)-est(:,param), 'b', time, conf, 'r', time, -conf, 'r');
end