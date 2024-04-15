close all;clc;clearvars;
% car = platform("file",'aDataUsr_006b.mat');
%
% car.run()
sensorParams = struct('noise', 0.1,...
    'sampleTime', 1e-2);
signalParams = struct('amplitude', 5, ...
    'frequency', 1/(1.5*2));
pendulum = PendulumSystem(a=8, b=1.4, c=1, T=18, timestep=1e-3, x0=[0;0], ...
    sensor=sensorParams, signal=signalParams);
pendulum.run()
pendulum.plot()
pendulum.produceErrorPlots();