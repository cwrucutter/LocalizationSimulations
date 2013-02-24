classdef VelocityErrorKF < handle
    %VelocityErrorKF Kalman filter on v and w for mobile robot
    % with augmented states for wheel slip
    
    properties
        x = []
        p = []
        
        A = []
        Phi = []
        Qk = []
        Rk_enc = []
        Rk_gyro = []
        Rk_vel = []
        
        y = []
        S = []
        
        dt = 1
        
        b = 1
    end
    
    methods (Access = public)
        function KF = VelocityErrorKF(dt)
            % Initialize dt
            KF.dt = dt;
            
            % Initialize state and covariance
            KF.x = [0; 0; 0; 0; 0; 0]; %[v; w; vdot; wdot; vRoff; vLoff]
            KF.p = zeros(length(KF.x),length(KF.x));
            y = [];
            
            % Initialize the diagonal noise
            pinit = 100;
            for i=1:length(KF.x)
                KF.p(i,i) = pinit;
            end
            KF.p(5,5) = 0.1;
            KF.p(6,6) = 0.1;
            
            % Initialize system model
            KF.A = zeros(length(KF.x),length(KF.x));
            KF.A(1,3) = 1;
            KF.A(2,4) = 1;
            KF.Phi = expm(KF.A * KF.dt); %Discretize by DT
            
            % Initialize Process noise values
            sigma_v = 0.1;
            sigma_w = 0.1;
            sigma_vdot = 1;
            sigma_wdot = 1;
            sigma_vRerr = 0.1;
            sigma_vLerr = 0.1;
            Q = zeros(length(KF.x),length(KF.x));
            Q(1,1) = sigma_v^2;
            Q(2,2) = sigma_w^2;
            Q(3,3) = sigma_vdot^2;
            Q(4,4) = sigma_wdot^2;
            Q(5,5) = sigma_vRerr^2;
            Q(6,6) = sigma_vLerr^2;
            KF.Qk = Q*KF.dt;
            
            % Initialize Measurement noise values (Encoders)
            sigma_v_enc = 0.01;
            sigma_w_enc = 0.01;
            R_enc = [sigma_v_enc^2 0; 0 sigma_w_enc^2];
            KF.Rk_enc = R_enc*KF.dt;
            
            % Initialize Measurement noise values (gyro)
            sigma_w_gyro = 0.01;
            R_gyro = [sigma_w_gyro^2];
            KF.Rk_gyro = R_gyro*KF.dt;
            
            % Initialize Measurement noise values (velocity measure)
            sigma_v_obs = 0.01;
            R_vel = [sigma_v_obs^2];
            KF.Rk_vel = R_vel*KF.dt;
        end
        
        function KF = setProcessNoise(KF, sigma_v, sigma_w, sigma_vdot, sigma_wdot, sigma_vRoff, sigma_vLoff)
            Q = zeros(length(KF.x),length(KF.x));
            Q(1,1) = sigma_v^2;
            Q(2,2) = sigma_w^2;
            Q(3,3) = sigma_vdot^2;
            Q(4,4) = sigma_wdot^2;
            Q(5,5) = sigma_vRoff^2;
            Q(6,6) = sigma_vLoff^2;
            KF.Qk = Q*KF.dt;
        end
        
        function KF = setMeasurementNoiseEncoder(KF, sigma_v_enc, sigma_w_enc)
            R_enc = [sigma_v_enc^2 0; 0 sigma_w_enc^2];
            KF.Rk_enc = R_enc*KF.dt;
        end
        
        function KF = setMeasurementNoiseGyro(KF, sigma_w_gyro)
            R_gyro = [sigma_w_gyro^2];
            KF.Rk_gyro = R_gyro*KF.dt;
        end
        
        function KF = setMeasurementNoiseVel(KF, sigma_v_obs)
            R_vel = [sigma_v_obs^2];
            KF.Rk_vel = R_vel*KF.dt;
        end
        
        function KF = setTrackWidth(KF, b)
            KF.b = b;
        end
        
        function KF = update(KF)
            % Update state:
            % x_k = Phi*x_k-1 + B*u
            %   where u = zeros
            KF.x = KF.Phi * KF.x;
            
            % Update covariance
            % p_pre = Phi * P * Phi + Qk
            KF.p = KF.Phi * KF.p * KF.Phi' + KF.Qk;
        end
        
        function KF = measureEncoderNoError(KF, v, w)
            % Encoders measure v and w
            H = [1 0 0 0 0 0;  %0.5    0.5
                 0 1 0 0 0 0]; %1/KF.b -1/KF.b];
            z = [v;w];
            Rk = KF.Rk_enc;
            
            % Measurement Update
            KF.measurementUpdate(H, z, Rk);
        end
        
        function KF = measureEncoder(KF, v, w)
            % Encoders measure v and w
            H = [1 0 0 0 0.5    0.5
                 0 1 0 0 1/KF.b -1/KF.b];
            z = [v;w];
            Rk = KF.Rk_enc;
            
            % Measurement Update
            KF.measurementUpdate(H, z, Rk);
        end
        
        function KF = measureGyro(KF, w)
            % Gyro measures w
            H = [0 1 0 0 0 0];
            z = [w];
            Rk = KF.Rk_gyro;
            
            % Measurement Update
            KF.measurementUpdate(H, z, Rk);           
        end
        
        function KF = measureVel(KF, v)
            % Gyro measures w
            H = [1 0 0 0 0 0];
            z = [v];
            Rk = KF.Rk_vel;
            
            % Measurement Update
            KF.measurementUpdate(H, z, Rk);           
        end
        
        function KF = measureVelGyro(KF, v, w)
            % Gyro measures w
            H = [1 0 0 0 0 0;
                 0 1 0 0 0 0];
            z = [v;w];
            Rk = [KF.Rk_vel 0; 0 KF.Rk_gyro];
            
            % Measurement Update
            KF.measurementUpdate(H, z, Rk);           
        end
    end
    
    
    methods (Access = private)
        function KF = measurementUpdate(KF, H, z, Rk)
            % Store innovation and innov. Covariance
            KF.y = z - H*KF.x;
            KF.S = H*KF.p*H' + Rk;
            
            % Calculate P_post
            KF.p = KF.p - KF.p*H'/(H*KF.p*H' + Rk)*H*KF.p;
            
            % Kalman gain
            K = KF.p*H'/(Rk);
            
            % Measurement update
            KF.x = KF.x + K*(z - H*KF.x); 
        end 
    end
    
end
