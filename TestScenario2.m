

numTests = 50;
numOutputs = 7;

procNoise = zeros(numTests,9);
measNoise = zeros(numTests,3);

% sigma_x = .01;        % Uncertainty in x
% sigma_y = .01;        % Uncertainty in y
% sigma_tht = .001;     % Uncertainty in theta
% sigma_v = .1;         % Uncertainty in velocity
% sigma_w = .1;         % Uncertainty in omega
% sigma_vRerr = 0.05;
% sigma_vLerr = 0.05;
% sigma_vdot = .5;
% sigma_wdot = .5;
sigma_x = .01;        % Uncertainty in x
sigma_y = .01;        % Uncertainty in y
sigma_tht = .001;     % Uncertainty in theta
sigma_v = .3;         % Uncertainty in velocity
sigma_w = .3;         % Uncertainty in omega
sigma_vRerr = 0.05;
sigma_vLerr = 0.05;
sigma_vdot = .5;
sigma_wdot = .5;
procNoise = repmat([sigma_x sigma_y sigma_tht sigma_v sigma_w sigma_vRerr sigma_vLerr sigma_vdot sigma_wdot],numTests,1);
% sigma_vr = 0.05;
% sigma_vl = 0.05;
% sigma_gps = 0.05;
sigma_vr = 0.05;
sigma_vl = 0.05;
sigma_gps = 0.05;
measNoise = repmat([sigma_vr sigma_vl sigma_gps],numTests,1);

 
% % Initialize process and measurement noise
% for i=1:numTests
%     rng(10000+i);
%     
%     sigma_v     = 0.4*rand + 0.005;         % Uncertainty in velocity
%     sigma_w     = 0.4*rand + 0.005;         % Uncertainty in omega
%     verr        = 1.0*rand + 0.01;
%     accel       = 1.0*rand + 0.01;
%     enc_v       = 0.1*rand + 0.005;
%     sigma_gps   = 0.1*rand + 0.005;
%     
%     sigma_x = .01;        % Uncertainty in x
%     sigma_y = .01;        % Uncertainty in y
%     sigma_tht = .001;     % Uncertainty in theta
%     sigma_v = sigma_v;
%     sigma_w = sigma_w;
%     sigma_vRerr = verr;
%     sigma_vLerr = verr;
%     sigma_vdot = accel;
%     sigma_wdot = accel;
%     procNoise(i,:) = [sigma_x; sigma_y; sigma_tht; sigma_v; sigma_w; sigma_vRerr; sigma_vLerr; sigma_vdot; sigma_wdot];
% 
%  
%     sigma_vr = enc_v;
%     sigma_vl = enc_v;
%     measNoise(i,:) = [sigma_vr; sigma_vl; sigma_gps];
%     
% end


resultsArchEKF_Err0 = zeros(numTests,numOutputs);
resultsArchUKF_Err0 = zeros(numTests,numOutputs);
resultsArchIterEKF_Err0 = zeros(numTests,numOutputs);
for i=1:numTests    
    display('NoError')
    i
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchEKF(@(i,dt) SimulateEncoderVelocityFault0(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchEKF_Err0(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchUKF(@(i,dt) SimulateEncoderVelocityFault0(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchUKF_Err0(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchIterEKF(@(i,dt) SimulateEncoderVelocityFault0(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchIterEKF_Err0(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
%     rng(i)
%     [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestFilter9States(@(i,dt) SimulateEncoderVelocityFault0(i,dt), procNoise(i,:), measNoise(i,:));
%     results9States_Err0(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];

end

resultsArchEKF_Err1 = zeros(numTests,numOutputs);
resultsArchUKF_Err1 = zeros(numTests,numOutputs);
resultsArchIterEKF_Err1 = zeros(numTests,numOutputs);
for i=1:numTests
    display('Error1')
    i
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchEKF(@(i,dt) SimulateEncoderVelocityFault1(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchEKF_Err1(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchUKF(@(i,dt) SimulateEncoderVelocityFault1(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchUKF_Err1(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchIterEKF(@(i,dt) SimulateEncoderVelocityFault1(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchIterEKF_Err1(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
end


resultsArchEKF_Err2 = zeros(numTests,numOutputs);
resultsArchUKF_Err2 = zeros(numTests,numOutputs);
resultsArchIterEKF_Err2 = zeros(numTests,numOutputs);
for i=1:numTests
    display('Error2')
    i
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchEKF(@(i,dt) SimulateEncoderVelocityFault2(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchEKF_Err2(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
    
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchUKF(@(i,dt) SimulateEncoderVelocityFault2(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchUKF_Err2(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
   
    rng(i)
    [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ] = TestArchIterEKF(@(i,dt) SimulateEncoderVelocityFault2(i,dt), procNoise(i,:), measNoise(i,:));
    resultsArchIterEKF_Err2(i,:) = [ filter_diverged, rms_dist, rmserr_tht, rmserr_v, rmserr_w, rmserr_vRoff, rmserr_vLoff ];
end

