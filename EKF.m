classdef EKF < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        sigma_omega double % Standard deviation of angular velocity (3 degrees/s)
        sigma_v double % Standard deviation of speed (0.05 m/s)
        sigma_d double % Amplitude of white noise for distance measurements (0.15 m)
        cov_w double % Relative uncertainty in angular velocity measurements
        cov_v double % Relative uncertainty in speed measurements
        cov_l double % Relative uncertainty in lidar ranges
        x_init (3,1) double % Beginning pos
        P_init (3,3) double % Beginning Covariance Matrix
    end
    properties (Access = public)
        P (3,3) double
        xpos (3, 1) double
        initialised single
        
        
    end
    
    methods
        function obj = EKF(x0, sigma_omega, sigma_v, sigma_d, cov_w, cov_v, cov_l)
            arguments
                x0 (3,1) double = [0;0;0];
                sigma_omega double = deg2rad(3)
                sigma_v double = 0.05
                sigma_d double = 0.15
                cov_w double = 0
                cov_v double = 0
                cov_l double = 0
                
            end
            obj.sigma_omega = sigma_omega;
            obj.sigma_v = sigma_v;
            obj.sigma_d = sigma_d;
            obj.cov_w = cov_w;
            obj.cov_v = cov_v;
            obj.cov_l = cov_l;
            
            obj.P_init = diag([cov_v, cov_l, cov_w]);
            obj.P = zeros(3,3); % <- initial known exactly
            obj.initialised = false;
            
        end
        
        function configure(obj, params)
            arguments
                obj
                params.x0 (3,:) double = double.empty(3,0)
                params.sigma_omega double = -1
                params.sigma_v double = -1
                params.sigma_d double = -1
                params.cov_w double = -1
                params.cov_v double = -1
                params.cov_l double = -1
            end
            
            if ~isempty(params.x0)
                obj.x_init = params.x0;
                obj.xpos = params.x0;
            end
            if (params.sigma_omega >= 0)
                obj.sigma_omega = params.sigma_omega;
            end
            if (params.sigma_v >= 0)
                obj.sigma_v = params.sigma_v;
            end
            if (params.sigma_d >= 0)
                obj.sigma_d = params.sigma_d;
            end
            if (params.cov_w >= 0)
                obj.cov_w = params.cov_w;
            end
            if (params.cov_v >= 0)
                obj.cov_v = params.cov_v;
            end
            if (params.cov_l >= 0)
                obj.cov_l = params.cov_l;
            end
            
            
        end
        
        function reset(obj)
            obj.xpos = obj.x_init;
            obj.P = obj.P_init;
        end
        
        function est = estimate(obj, imu, t)
            arguments
                obj
                imu (2,1) double;
                t (2,1) uint32;
            end
            t = double(t);
            delta_t = (t(2)-t(1)) * 1e-4;
            obj.initialised = true;
            
            v = imu(1);
            omega = imu(2);
            curr = obj.xpos(1:3);
            
            % Prediction step
            obj.xpos = curr + [v * cos(curr(3)) * delta_t;
                v * sin(curr(3)) * delta_t;
                omega * delta_t];
            
            Q = diag([(obj.sigma_omega * 2 * delta_t)^2, ...
                (obj.sigma_omega * 2 *delta_t)^2, (obj.sigma_v * delta_t)^2]);
            
            F = [1, 0, -v * sin(curr(3)) * delta_t;
                0, 1,  v * cos(curr(3)) * delta_t;
                0, 0,  1];
            
            obj.P = F * obj.P * F' + Q;
            
            est = [obj.xpos; t(2)*1e-4];
            
        end
        
        
        function [x, P] = update(obj, ooi_ref, ooi_meas)
            arguments
                obj
                ooi_ref (2, :) double
                ooi_meas (2, :) double
            end
            
            for j = 1:5
                
                R = obj.sigma_d^2*4; % artificially increase std.dev
                
                % SONAR distance to OOIs (measured)
                z_meas = sqrt((ooi_meas(1,:) - obj.xpos(1)).^2 + (ooi_meas(2,:) - obj.xpos(2)).^2);
                
                
                for i = 1:size(ooi_meas,2)
                    d_exp = sqrt((ooi_ref(1,i) - obj.xpos(1))^2 + (ooi_ref(2,i) - obj.xpos(2))^2);
                    
                    % Update step
                    H = [(obj.xpos(1) - ooi_ref(1,i))/d_exp, (obj.xpos(2) - ooi_ref(2,i))/d_exp, 0];
                    
                    K = obj.P * H' * inv(H * obj.P * H' + R); % Kalman gain
                    obj.xpos = obj.xpos + K * (z_meas(i)-d_exp); % Update state estimate
                    obj.P = (eye(3) - K * H) * obj.P; % Update covariance matrix
                end
            end
            
            x = obj.xpos;
            P = obj.P;
        end
    end
    methods (Static)
        function [rho, theta] = polarOOIs(pos, knownOOIs)
            % Gets the distance
            arguments
                pos (3,1) double
                knownOOIs (2,:) double
            end
            [rho, theta] = cart2pol(knownOOIs(1,:) - pos(1), knownOOIs(2,:) - pos(2));
        end
    end
end