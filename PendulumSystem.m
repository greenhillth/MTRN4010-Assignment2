classdef PendulumSystem < handle
    
    properties
        a double
        b double
        c double
        dt double % timestep
        tspan (1,2) double
        initcons (2,1) double
        t (1,:) double
        y (2,:) double
    end
    properties (Access = private)
        sensor struct
        step uint32
        u function_handle
        x_est (2,1) double
        P_diag (2,:) double
        P (2,2) double
        R double
        
        trueRegister(3,:) double
        estimateRegister(3,:) double
        varRegister(3,:) double
        errRegister (3,:) double
        trueIndex uint32
        estIndex uint32
        errIndex uint32
    end
    
    methods
        function obj = PendulumSystem(params)
            arguments
                params.a double
                params.b double
                params.c double
                params.T double
                params.timestep double
                params.x0 (2,1) double
                params.sensor struct
                params.signal struct
            end
            obj.a = params.a;
            obj.b = params.b;
            obj.c = params.c;
            obj.dt = params.timestep;
            obj.tspan = [0,params.T];
            obj.initcons = [0;0];
            
            % init sensor struct
            obj.sensor = params.sensor;
            obj.sensor(1).measurements = zeros(2,64);
            obj.sensor(1).numMeasurements = 0;
            
            
            obj.step = 0;
            
            amp = params.signal.amplitude;
            freq = params.signal.frequency;
            obj.u = @(t) amp * square(2*pi*freq*t);
            
            n = (params.T/obj.dt)+1;
            obj.t = obj.tspan(1):obj.dt:obj.tspan(2);
            
            obj.trueRegister = zeros(3, n);
            obj.estimateRegister = zeros(3, n);
            obj.varRegister = zeros(3, n);
            obj.errRegister = zeros(3, n);
            obj.trueIndex = 0;
            obj.estIndex = 0;
            obj.errIndex = 0;
            
        end
        function run(obj)
            pendODE = @(t, y) obj.ode(t, y);
            state = obj.initcons;
            obj.x_est = obj.initcons;
            single_step = odeset('InitialStep', obj.dt);
            
            obj.P = eye(2) * 0.1^2;
            obj.R = deg2rad(5)^2;
            
            for t = 0:obj.dt:obj.tspan(2)
                
                % add calculated state to state register
                obj.addCalculation(t, state);
                obj.addEstimate(t, obj.x_est, obj.P);
                % step simulation
                [time, state] = PendulumSystem.RK4(pendODE, t, state, obj.dt);
                
                % estimate step
                obj.estimate();
                % measure
                meas = readSensor(obj, t, state(2));
                if ~isempty(meas)
                    obj.update(meas);
                end
                
                % compute error
                obj.addError(t,state-obj.x_est);
                
            end
            balls = obj.trueRegister(:, 1:obj.trueIndex);
            tits = obj.estimateRegister(:, 1:end);
            fprintf('balls');
        end
        
        function Simulate(obj)
            % Define the ODE function
            pendODE = @(t, y) obj.ode(t, y);
            
            % Using ode45 to solve
            [obj.t, obj.y] = ode45(pendODE, obj.tspan, obj.initcons);
            
        end
        
        function dydt = ode(obj, t, y)
            dydt = zeros(2, 1);
            dydt(1) = y(2); % omega
            dydt(2) = -obj.a*sin(y(1)) - obj.b*y(2) + obj.c*obj.u(t); % acceleration
        end
        
        function estimate(obj)
            F = [1, -obj.b*obj.dt; -obj.a*cos(obj.x_est(1) - obj.b*obj.x_est(2))*obj.dt, 1];
            G = [0;obj.c*obj.dt];
            
            % Compute the partial derivatives of the state transition function
            F = [1, -obj.b*obj.dt; -obj.a*cos(obj.x_est(1) - obj.b*obj.x_est(2))*obj.dt, 1];
            
            % Compute G matrix
            G = [0; obj.c*obj.dt];
            F11 = 1;
            F12 = obj.dt;
            
            % Compute the second row of F matrix
            F21 = -obj.a*cos(obj.x_est(1))*obj.dt;
            F22 = 1 - obj.b*obj.dt;
            
            % Compute G matrix
            G1 = 0;
            G2 = obj.c*obj.dt;
            
            % Assemble F matrix
            F = [F11, F12; F21, F22];
            
            % Assemble G matrix
            G = [G1; G2];
            
            obj.x_est = F * obj.x_est;
            obj.P = F * obj.P * F' + G * obj.R * G';
        end
        
        function update(obj, meas)
            
            H = [0, 1]; % Measurement matrix (only measure angle)
            % S = H * obj.P * H' + obj.R;
            % K = obj.P * H' / S;
            % obj.x_est = obj.x_est + K * (meas - H * obj.x_est);
            % obj.P = (eye(2) - K * H) * obj.P;
            
            H = [0, 1];
            
            
            % Compute innovation (measurement residual)
            delta_meas = meas - obj.x_est(2);
            
            % Compute innovation covariance
            S = H * obj.P* H' + obj.R;
            
            % Compute Kalman gain
            K = obj.P * H' / S;
            
            % Update state estimate
            obj.x_est = obj.x_est + K * delta_meas;
            
            % Update covariance estimate
            obj.P = (eye(2) - K * H) * obj.P;
            
        end
        
        function plot(obj)
            figure;
            subplot(2, 1, 1);
            idx = obj.sensor.numMeasurements;
            plot(obj.t, obj.estimateRegister(2, :));
            hold on;
            plot(obj.t, obj.trueRegister(2, :));
            title('Estimated and Actual Pendulum Angle vs. Time');
            xlabel('Time (s)');
            ylabel('Angle (\phi)');
            legend('Estimated Angle', 'Actual Angle');
            
            subplot(2, 1, 2);
            plot(obj.sensor.measurements(1,1:idx), obj.sensor.measurements(2, 1:idx), "LineStyle", "none","Marker","o");
            hold on;
            plot(obj.t, obj.estimateRegister(3, :));
            plot(obj.t, obj.trueRegister(3, :));
            title('Estimated and Actual Pendulum Angular Velocity vs. Time');
            xlabel('Time (s)');
            legend('Measured Velocity', 'Estimated Velocity', 'Actual Velocity');
        end
        
        function y = readSensor(obj, t, reading)
            arguments
                obj
                t double
                reading double
            end
            y = double.empty();
            if (mod(t,obj.sensor.sampleTime) == 0)
                noise = randn(1,1)*obj.sensor.noise;
                y = noise+reading;
                obj.addMeasurement(t, y);
            end
        end
        function addMeasurement(obj, t, y)
            arguments
                obj
                t double
                y double
            end
            idx = obj.sensor.numMeasurements + 1;
            if(idx > size(obj.sensor.measurements, 2))
                obj.sensor.measurements = cat(2, obj.sensor.measurements, zeros(2,64));
            end
            obj.sensor.measurements(:,idx) = [t;y];
            obj.sensor.numMeasurements = idx;
        end
        function addCalculation(obj, t, y)
            arguments
                obj
                t double
                y (2,1) double
            end
            idx = obj.trueIndex + 1;
            if(idx > size(obj.trueRegister, 2))
                obj.trueRegister = cat(2, obj.trueRegister, zeros(3,512));
            end
            obj.trueRegister(:,idx) = [t;y];
            obj.trueIndex = idx;
        end
        function addError(obj, t, y)
            arguments
                obj
                t double
                y (2,1) double
            end
            idx = obj.errIndex + 1;
            if(idx > size(obj.errRegister, 2))
                obj.errRegister = cat(2, obj.errRegister, zeros(3,512));
            end
            obj.errRegister(:,idx) = [t;y];
            obj.errIndex = idx;
        end
        function addEstimate(obj, t, y, P)
            arguments
                obj
                t double
                y (2,1) double
                P (2,2) double
            end
            idx = obj.estIndex + 1;
            if(idx > size(obj.estimateRegister, 2))
                obj.estimateRegister = cat(2, obj.estimateRegister, zeros(3,512));
                obj.varRegister = cat(2, obj.varRegister, zeros(3,512));
            end
            obj.estimateRegister(:,idx) = [t;y];
            obj.varRegister(:,idx) = [t;P(1,1);P(2,2)];
            obj.estIndex = idx;
        end
        
    end
    methods (Access = private)
    end
    methods (Static)
        function [t, y] = RK4(func, tCurr, yCurr, step_size)
            % using RK4 function to step through ODE
            
            k1 = step_size * feval(func, tCurr, yCurr);
            k2 = step_size * feval(func, tCurr + step_size/2, yCurr + k1/2);
            k3 = step_size * feval(func, tCurr + step_size/2, yCurr + k2/2);
            k4 = step_size * feval(func, tCurr + step_size, yCurr + k3);
            
            y = yCurr + (k1 + 2*k2 + 2*k3 + k4) / 6;
            t = tCurr + step_size;
        end
        
    end
end