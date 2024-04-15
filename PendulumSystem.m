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
        amp double
        step uint32
        u function_handle
        x_est (2,1) double
        P_diag (2,:) double
        P (2,2) double
        R double
        
        anim struct
        
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
            obj.amp = amp;
            freq = params.signal.frequency;
            obj.u = @(t) amp * square(2*pi*freq*t);
            
            n = (params.T/obj.dt)+1;
            obj.t = obj.tspan(1):obj.dt:obj.tspan(2);
            
            %init registers
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
            
            obj.P = [50^2, 0; 0, 1^2] % Uncertainty in starting vals
            obj.R = deg2rad(5)^2;
            
            obj.initAnimation();
            uint32 frame;
            frame = 0;
            
            for t = 0:obj.dt:obj.tspan(2)
                frame = frame+1;
                
                % add calculated state to state register
                obj.addCalculation(t, state);
                obj.addEstimate(t, obj.x_est, obj.P);
                % step simulation
                [time, state] = PendulumSystem.RK4(pendODE, t, state, obj.dt);
                
                % estimate step
                [p_est, x_est] = obj.estimate();
                % measure
                meas = readSensor(obj, t, state(2));
                if ~isempty(meas)
                    obj.update(meas, p_est, x_est);
                else
                    obj.x_est = x_est;
                    obj.P = p_est;
                end
                
                
                % compute error
                obj.addError(t,state-obj.x_est);
                
                if (mod(frame,4) == 0)
                    obj.animate(state(1), obj.x_est(1), t, frame);
                end
            end
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
        
        function [p, x] = estimate(obj)
            F = [1, -obj.b*obj.dt*10; -obj.a*cos(obj.x_est(1) - obj.b*obj.x_est(2))*obj.dt*10, 1];
            G = [0;obj.c*obj.dt];
            
            % Compute the partial derivatives of the state transition function
            F = [1, -obj.b*obj.dt*10; -obj.a*cos(obj.x_est(1) - obj.b*obj.x_est(2))*obj.dt*10, 1];
            
            % Compute G matrix
            G = [0; obj.c*obj.dt];
            F11 = 1;
            F12 = obj.dt;
            
            % Compute the second row of F matrix
            F21 = -obj.a*cos(obj.x_est(1))*obj.dt;
            F22 = 1 - obj.b*obj.dt;
            
            % Compute G matrix
            G1 = 0;
            G2 = obj.c*1.5;
            
            % Assemble F matrix
            F = [F11, F12; F21, F22];
            
            % Assemble G matrix
            G = [G1; G2];
            
            x = F * obj.x_est;
            p = F * obj.P * F' + G * obj.R * G';
        end
        
        function update(obj, meas, p_est, x_est)
            arguments
                obj
                meas (1,1) double
                p_est (2,2) double
                x_est (2,1) double
            end
            H = [0,1]; % Jacobian of measurement function
            K = p_est * H' * inv(H * p_est * H' + obj.R);
            obj.x_est = x_est + K * (meas - x_est(2));
            obj.P = (eye(2) - K * H) * p_est;
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
        
        function plot(obj)
            figure(3);
            subplot(2, 1, 1);
            idx = obj.sensor.numMeasurements;
            plot(obj.t, obj.trueRegister(2, :));
            hold on;
            plot(obj.t, obj.estimateRegister(2, :));
            title('Estimated and Actual Pendulum Angle vs. Time');
            xlabel('Time (s)');
            ylabel('Angle (\phi)');
            legend('Actual Angle', 'Estimated Angle');
            
            subplot(2, 1, 2);
            plot(obj.sensor.measurements(1,1:idx), obj.sensor.measurements(2, 1:idx), "LineStyle", "none","Marker","o");
            hold on;
            plot(obj.t, obj.estimateRegister(3, :));
            plot(obj.t, obj.trueRegister(3, :));
            title('Estimated and Actual Pendulum Angular Velocity vs. Time');
            xlabel('Time (s)');
            legend('Measured Velocity', 'Estimated Velocity', 'Actual Velocity');
        end
        
        function produceErrorPlots(obj)
            arguments
                obj
            end
            %     t (1,:) double
            %     e (3,:) double % Error matrix with row 1 as xerr, row 2 as yerr and row 3 as herr
            %     std_devs (3,:) double % std_dev matrix with row 1 as x_std, row 2 as y_std and row 3 as h_std
            % end
            t = obj.errRegister(1,:);
            e = obj.errRegister(2:3,:);
            std_devs = obj.varRegister(2:3, :);
            
            finalIdx = length(t);
            
            x = t(1,1:finalIdx);
            y1 = e(1,1:finalIdx);
            std1 = std_devs(1,1:finalIdx);
            
            y2 = e(2,1:finalIdx);
            std2 = std_devs(2,1:finalIdx);
            
            colours = ["#3154b5","#0aeff7","#2fe053", "#000000"];
            
            % Create a new figure
            fig = figure(2);
            title('Error Plots');
            screenSize = get(groot, 'ScreenSize');
            figWidth = 600; % Adjust the width of the figure as needed
            figHeight = 500; % Adjust the height of the figure as needed
            figPosX = (screenSize(3) - figWidth) / 2;
            figPosY = (screenSize(4) - figHeight) / 2;
            set(fig, 'Position', [figPosX, figPosY, figWidth, figHeight]);
            ybounds1 = [-1, 1];
            ybounds2 = [-10, 10];
            xbounds = [0, t(end)];
            
            
            % Create the first subplot
            ax1 = subplot(2,1,1);
            tline1 = "Error and Standard Deviation in \phi";
            [probs, max, avg] = platform.getSTDinfo(y1, std1); p = probs*100; p = round(p);
            tline2 = sprintf('Within: 1\\sigma - %i%%, 2\\sigma - %i%%, 3\\sigma - %i%%',p(1), p(2), p(3));
            tline3 = sprintf('Max: %.2f, Average: %.2f', max, avg);
            plot(ax1, x, y1, "Color", colours(1), "LineWidth", 3);
            hold(ax1, "on");
            plot(ax1, x, std1, "Color", colours(2));
            plot(ax1, x, -std1, "Color", colours(2));
            plot(ax1, x, 2*std1, "Color", colours(3));
            plot(ax1, x, -2*std1, "Color", colours(3));
            plot(ax1, x, 3*std1, "Color", colours(4));
            plot(ax1, x, -3*std1, "Color", colours(4));
            hold(ax1, "off");
            
            xlim(ax1, xbounds);
            ylim(ax1, [-4*max, 4*max]);
            
            title([tline1, append(tline2, ', ', tline3)]);
            xlabel('t, seconds');
            ylabel('Err $\phi$','interpreter','latex')
            
            % Create the second subplot
            ax2 = subplot(2,1,2);
            tline1 = "Error and Standard Deviation in \omega";
            [probs, max, avg] = platform.getSTDinfo(y2, std2); p = probs*100; p = round(p);
            tline2 = sprintf('Within: 1\\sigma - %i%%, 2\\sigma - %i%%, 3\\sigma - %i%%',p(1), p(2), p(3));
            tline3 = sprintf('Max: %.2f, Average: %.2f', max, avg);
            plot(ax2, x, y2, "Color", colours(1), "LineWidth", 3);
            hold(ax2, "on");
            plot(ax2, x, std2, "Color", colours(2));
            plot(ax2, x, -std2, "Color", colours(2));
            plot(ax2, x, 2*std2, "Color", colours(3));
            plot(ax2, x, -2*std2, "Color", colours(3));
            plot(ax2, x, 3*std2, "Color", colours(4));
            plot(ax2, x, -3*std2, "Color", colours(4));
            hold(ax2, "off");
            
            xlim(ax2, xbounds);
            ylim(ax2, [-4*max, 4*max]);
            
            title([tline1, append(tline2, ', ', tline3)]);
            xlabel('t, seconds');
            ylabel('Err $\dot{\phi}$','interpreter','latex')
            
            
        end
        
        
    end
    methods (Access = private)
        %% Plotting Functions
        
        function initAnimation(obj)
            %init animation objects
            fig = figure(1);
            
            screenSize = groot().ScreenSize;
            width = 800;
            height = 900;
            x = (screenSize(3) - width) / 2;
            y = (screenSize(4) - height) / 2;
            set(fig, 'Position', [x, y, width, height], 'Name', 'Pendulum Animation');
            
            % animation axis
            ax1 = subplot(2,1,1);
            ax2 = subplot(2,1,2);
            axis(ax1, "equal");
            
            ax1.Title.String = "Pendulum Animation";
            ax1.FontSize = 14;
            ax1.FontWeight = 'bold';
            ax1.XAxis.Visible = 'off';
            ax1.YAxis.Visible = 'off';
            
            %input axis
            
            ax2.Title.String = "Input u(t)";
            xlim(ax2, [0, obj.tspan(2)]);
            ylim(ax2, [-1.5*obj.amp, 1.5*obj.amp]);
            
            ax1.Position = [0.13 0.28 0.77 0.6];
            ax1.PositionConstraint = "outerposition";
            ax2.Position = [0.13 0.11 0.77 0.11];
            ax2.PositionConstraint = "innerposition";
            
            
            
            iCons = obj.initcons;
            
            
            % black, grey, blue
            colour = ["#000000", "#5e636b", "#4287f5", "#eb4034", "#f27e18"];
            
            %arm params
            
            length = 30;
            
            xlim(ax1, [-1.5*length, 1.5*length]);
            ylim(ax1, [-1.5*length, 1.5*length]);
            
            initPos = [length*sin(iCons(1)); -length*cos(iCons(1))];
            
            arm = line(ax1,...
                [0, initPos(1)], [0, initPos(2)],...
                'Color', colour(2),...
                'LineWidth', 1,...
                'Marker', '+',...
                'MarkerSize', 5);
            
            e_arm = line(ax1,...
                [0, initPos(1)], [0, initPos(2)],...
                'Color', colour(2),...
                'LineWidth', 1,...
                'LineStyle', "--",...
                'Marker', '+',...
                'MarkerSize', 5);
            
            
            radius = 4;
            bob = rectangle(ax1,...
                'Position', [initPos(1)-radius, initPos(2)-radius, radius*2, radius*2],...
                'Curvature', [1, 1], ...
                'EdgeColor', colour(3),...
                'LineWidth', 2);
            
            e_bob = rectangle(ax1,...
                'Position', [initPos(1)-radius, initPos(2)-radius, radius*2, radius*2],...
                'Curvature', [1, 1], ...
                'EdgeColor', colour(4),...
                'LineWidth', 1.5);
            
            sig = line(ax2,...
                zeros(1,obj.tspan(2)/obj.dt+1), zeros(1,obj.tspan(2)/obj.dt+1),...
                'Color', colour(4),...
                'LineStyle', "none", ...
                'Marker', ".",...
                'MarkerSize', 10 ...
                );
            
            
            obj.anim = struct (...
                'pAx', ax1,...
                'inAx', ax2,...
                'length', length,...
                'radius', radius,...
                'pos', initPos,...
                'arm', arm,...
                'bob', bob,...
                'e_arm', e_arm,...
                'e_bob', e_bob,...
                'signal', sig...
                );
            
            
        end
        
        function animate(obj, act_theta, est_theta, t, frame)
            arguments
                obj
                act_theta (1,1) double
                est_theta (1,1) double
                t (1,1) double
                frame uint16
            end
            rad = obj.anim.radius;
            % get actual pos
            act_x = obj.anim.length * sin(act_theta);
            act_y = -obj.anim.length * cos(act_theta);
            act_vec = [act_x-rad, act_y-rad, rad*2, rad*2];
            
            % get estimator pos
            est_x = obj.anim.length * sin(est_theta);
            est_y = -obj.anim.length * cos(est_theta);
            est_vec = [est_x-rad, est_y-rad, rad*2, rad*2];
            
            obj.anim.arm.XData = [0,act_x];
            obj.anim.arm.YData = [0,act_y];
            obj.anim.bob.Position = act_vec;
            
            obj.anim.e_arm.XData = [0,est_x];
            obj.anim.e_arm.YData = [0,est_y];
            obj.anim.e_bob.Position = est_vec;
            
            
            obj.anim.signal.XData(frame) = t;
            obj.anim.signal.YData(frame) = obj.u(t);
            pause(obj.dt);
            
            
        end
        
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