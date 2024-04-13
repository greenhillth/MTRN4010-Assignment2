classdef platform < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        api
        menu interface
        initFile (1, :) char
        loadedFile string
        
        params struct
        
        ekf EKF
        
        kinematicModel function_handle
        
        imuReading (2, 1) double
        signal (2,1) double
        readTime (2, 1) double
        
        % Registers
        positionRegister (4, :) double
        errRegister (4,:) double
        
        localiseMatrix(3,:) double
        
        OOI struct
        
        
        currentPose (3,1) double
        localisedPose (3,1) double
        t single
        
        % indexes
        posIndex uint16
        lidarIndex uint16
        errIndex uint8
        estIndex uint16
        
        
        activeLidar uint8
        
        
        
        status string
        play uint8
        
    end
    
    methods
        function obj = platform(parameters)
            arguments
                parameters.position (3,1) double = [0; 0; 0]
                parameters.kinematicModel function_handle = @(v, phi, omega) ...
                    [v*cos(phi); v*sin(phi); omega]
                parameters.length double = 1;
                parameters.gain double {mustBePositive} = 1;
                parameters.bias double = 0;
                parameters.file (1,:) char = 'aDataUsr_006b.mat';
            end
            obj.api = APImtrn4010_v05();
            obj.loadedFile = "";
            obj.status = "initialising";
            
            obj.initFile = parameters.file;
            
            % setting params
            obj.params = struct('length', parameters.length, 'gain', parameters.gain, 'bias', parameters.bias, 'lidarPose', zeros(2,3));
            obj.kinematicModel = parameters.kinematicModel;
            obj.imuReading = [0;0];
            obj.readTime = [0;0];
            
            obj.OOI = struct('expected', single.empty(2,0),...
                'scanned', single.empty(2,0),...
                'visible', single.empty(2,0));
            
            % initialising remaining fields
            obj.positionRegister = zeros(4, 4096);
            obj.estIndex = 1;
            obj.positionRegister(:,1) = [parameters.position; 0];
            obj.posIndex = 1;
            obj.lidarIndex = 0;
            obj.activeLidar = 1;
            obj.errRegister = zeros(4, 4096);
            obj.errIndex = 0;
            
            obj.OOI.expected = double.empty(2, 0);
            
            obj.ekf = EKF();
        end
        
        % Configure parameters
        function configureParameters(obj, parameters)
            arguments
                obj                 platform
                parameters.length   double
                parameters.gain     double
                parameters.bias     double
                parameters.lidarPose (2,3) double
            end
            obj.params.length = parameters.length;
            obj.params.gain = parameters.gain;
            obj.params.bias = parameters.bias;
            
        end
        
        function loadFile(obj, dataFile)
            arguments
                obj platform
                dataFile (1, :) char
            end
            
            %obj.updateStatus("Loading path");
            obj.loadedFile = string(dataFile);
            obj.api.b.LoadDataFile(char("./datasets/" + obj.loadedFile));
            obj.api.b.Rst();
            %obj.updateStatus("Loaded path");
        end
        
        function run(obj)
            obj.loadFile(obj.initFile);
            obj.initialiseMenu();
            
            % TODO - grab and apply modified params
            while ~(obj.menu.flags.userInput)
                pause(0.05);
            end
            
            obj.applyParams(obj.menu.params);
            control = 1;
            frame = 0;
            tic;
            % event cycle
            while (control ~= 0)
                frame = frame+1;
                
                obj.menu.ProcessStep.Value = toc*1e3;
                tic;
                control = obj.menu.control;
                switch (control)
                    case 1      % pause
                        pause(0.05);
                        continue;
                    case 2      % play
                        finished = obj.processEvent();
                    case 3      % reset
                        obj.reset();
                        frame = 0;
                        obj.menu.control = 1;       %set to pause
                end
            end
            
        end
        
        function reset(obj)
            obj.menu.flags.reset = false;
            obj.api.b.Rst();
            obj.imuReading = [0;0];
            obj.readTime = [0;0];
            obj.lidarIndex = 0;
            obj.localisedPose = obj.positionRegister(1:3,1);
            obj.posIndex = 1;
            
            obj.OOI.scanned = double.empty(2,0);
        end
        
        function applyParams(obj, params)
            arguments
                obj platform
                params struct
            end
            if (params.fileChanged)
                obj.loadFile(params.file);
            end
            if (params.modified)
                obj.params.gain = params.gain;
                obj.params.bias = params.bias;
                obj.positionRegister(:,1) = [obj.menu.params.initPos;0];
                obj.params.lidarPose = params.lidarPos;
                
            end
            obj.ekf.configure("x0", obj.menu.params.initPos);
        end
        
        
        
        function inProgress = processEvent(obj)
            
            nextEvent = obj.api.RdE;
            event = nextEvent();
            switch (event.ty)
                case 0  % end case
                    disp('End of event');
                    inProgress = false;
                    return;
                case 1  % lidar case
                    obj.updateStatus("Processing Lidar");
                    obj.t = event.t;
                    obj.processLidar(event);
                case 2
                    obj.t = event.t;
                    obj.updateStatus("Processing IMU");
                    obj.processIMU(event);
                otherwise
                    
            end
            inProgress = true;
        end
        
        function updatePlot(obj, GCFvecs, Lvecs, ooi, landmarks, ooiCart)
            obj.menu.updatePlotVectors(GCFvecs, Lvecs(1:4,:), Lvecs(5:8,:), obj.errRegister, obj.lidarIndex, obj.OOI.visible, obj.localisedPose);
            obj.menu.updateLidarOOIs(ooi, landmarks, ooiCart);
        end
        
        
        function processLidar(obj, eventData)
            %update current position
            obj.lidarIndex = obj.lidarIndex + 1;
            computedPos = obj.ekf.estimate(obj.imuReading, [obj.readTime(2);eventData.t]);
            
            obj.currentPose = computedPos(1:3);
            
            
            
            %update errors
            dPos = computedPos(1:3) - double(obj.api.gcgt());
            obj.addToErrorBuffer([double(eventData.t);dPos]);
            
            % get range and intensities of both lidars
            theta = deg2rad(-75:0.5:75);
            [dist, intensity] = platform.scanToRI(eventData.d');
            
            %convert to cartesian
            [x, y] = pol2cart([theta;theta], dist);
            
            
            % apply cluster analysis to get OOIs
            dL = 0.6; step = 0.5;
            clusterFcn = obj.api.b.FindSmallSegmentsFS;
            [~, clust1, n1] = clusterFcn(dist(1, :), dL, step);
            [~, clust2, n2] = clusterFcn(dist(2, :), dL, step);
            
            clust1 = reshape(clust1, [n1,3]); clust2 = reshape(clust2, [n2,3]);
            
            
            ooi1P = [clust1(:,3), clust1(:,2)]';  % polar OOIS
            ooi2P = [clust2(:,3), clust2(:,2)]';
            
            if ~(isempty(ooi1P))
                [xL, yL] = pol2cart(deg2rad(ooi1P(1,:)), ooi1P(2,:));
                ooi1C = [xL;yL];
            else
                ooi1C= single.empty(2,0);
            end
            
            if ~(isempty(ooi2P))
                [xL, yL] = pol2cart(deg2rad(ooi2P(1,:)), ooi2P(2,:));
                ooi2C = [xL;yL];
            else
                ooi2C= single.empty(2,0);
            end
            
            %transform lidar data to GCF
            scansCart = [x(1,:); y(1,:); x(2,:); y(2,:)];
            [L1G, L2G, O1G, O2G] = obj.lidarsToGCF(scansCart, ooi1C, ooi2C);
            
            % Associate scans if new OOI found
            obj.findOOIs(cat(2,O1G, O2G));
            
            %update plots (lidar, global and OOI)
            if (obj.menu.TabGroup.SelectedTab == obj.menu.StatusTab)
                Lvecs = [x(obj.activeLidar,:);y(obj.activeLidar,:);theta;dist(obj.activeLidar,:);
                    L1G;L2G];
                
                obj.updatePlot([computedPos;obj.imuReading(1);obj.imuReading(2);0;0], Lvecs, ooi1P, ooi1C, obj.OOI.scanned);
            end
            % update lidar time
            obj.readTime(1) = eventData.t;
        end
        
        function [L1G, L2G, O1G, O2G] = lidarsToGCF(obj, scans, ooi1, ooi2)
            arguments
                obj
                scans (4,:) double
                ooi1
                ooi2
            end
            
            % get coords relative to vehicleCF
            Lpose = obj.params.lidarPose;
            L1C = platform.localToGlobalCF(scans(1:2, :), Lpose(1,3), [Lpose(1,1);Lpose(1,2)]);
            L2C = platform.localToGlobalCF(scans(3:4, :), Lpose(2,3), [Lpose(2,1);Lpose(2,2)]);
            
            O1C = platform.localToGlobalCF(ooi1, Lpose(1,3), [Lpose(1,1);Lpose(1,2)]);
            O2C = platform.localToGlobalCF(ooi2, Lpose(2,3), [Lpose(2,1);Lpose(2,2)]);
            
            % transform coords to globalCF
            cP = obj.currentPose;
            L1G = platform.localToGlobalCF(L1C, cP(3), cP(1:2));
            L2G = platform.localToGlobalCF(L2C, cP(3), cP(1:2));
            
            O1G = platform.localToGlobalCF(O1C, cP(3), cP(1:2));
            O2G = platform.localToGlobalCF(O2C, cP(3), cP(1:2));
            
            
        end
        
        function processIMU(obj, eventData)
            % add IMU reading to register
            imu = eventData.d;
            obj.imuReading = [
                imu(1)*obj.params.gain;
                imu(2)+obj.params.bias];
            
            
            % generate pose estimate and store in position register
            computedPos = obj.ekf.estimate(imu, [obj.readTime(2);eventData.t]);
            obj.addMeasurement(computedPos);
            
            
            % update imu read time
            obj.readTime(2) = eventData.t;
            
            
        end
        
        function added = findOOIs(obj, scan)
            arguments
                obj
                scan (2,:) single
            end
            
            % associate currently visible points with oois
            [ref, vis, ~] = obj.associateScans(scan, obj.OOI.expected);
            % update EKF
            if ~(isempty(ref))
                obj.ekf.update(ref, vis);
            end
            
            % update sonar plot
            obj.menu.updateSonar(vis, obj.currentPose(1:2));
            obj.OOI.visible = vis;
            
            % update existing OOI position register
            dist = pdist2(obj.OOI.scanned', vis', 'fasteuclidean');
            tol = 0.7;
            [regIndex, pointIndex] = find(dist<tol);
            
            if ~(isempty(pointIndex)) %if corresponding index found
                nonMatching = setdiff(1:size(vis, 2), pointIndex');
                visible = vis(:,pointIndex);
                %overwrite existing
                obj.OOI.scanned(:,regIndex) = visible;
                %append non-matching
                obj.OOI.scanned = cat(2, obj.OOI.scanned, vis(:, nonMatching));
                added = true;
            else
                obj.OOI.scanned = cat(2, obj.OOI.scanned, vis);
                added = false;
            end
            
        end
        
        % get transformation matrix
        function [reference, measured, delta] = associateScans(obj, meas, ref)
            % - get list of landmarks from scanned OOIS
            
            OOIs = meas;
            eOOIs = ref;
            dist = pdist2(eOOIs', OOIs', 'fasteuclidean');
            
            
            tol = 0.75;    %tolerance of euclidean distance
            [exIndex, scanIndex] = find(dist<tol);
            reference = eOOIs(:,exIndex);
            measured = OOIs(:, scanIndex);
            delta = transpose(dist(sub2ind(size(dist),exIndex, scanIndex)));
            
        end
        
        function [pose, valid] = EstimatePoseD(obj, referencePoints, scannedPoints)
            [tform, ~, validtform] = estgeotform2d(referencePoints', scannedPoints', "rigid", 'MaxDistance', 3, 'MaxNumTrials',1000);
            rotAngle = obj.currentPose(3)-deg2rad(tform.RotationAngle);
            inverseOrigin = transformPointsInverse(tform, [0 0])';
            valid = (validtform == 0);
            if valid
                obj.localiseMatrix = [inverseOrigin; rotAngle];
            else
                obj.localiseMatrix = double.empty(3,0);
            end
            
            pose = [platform.localToGlobalCF(inverseOrigin, rotAngle, obj.currentPose(1:2)); rotAngle];
            
        end
        
        function f = initialiseMenu(obj)
            arguments
                obj platform
            end
            obj.menu = interface();
            env = obj.api.b.GetInfo();
            GT = obj.api.b.GetGroundTruth();
            LC = env.LidarsCfg;
            lidarPose = [LC.Lidar1.Lx, LC.Lidar1.Ly, LC.Lidar1.Alpha;
                LC.Lidar2.Lx, LC.Lidar2.Ly, LC.Lidar2.Alpha;];
            
            obj.menu.initialise('directory', dir('datasets/*.mat'), ...
                'loadedFile', obj.loadedFile, ...
                'Walls', env.Context.Walls, ...
                'OOI', env.Context.Landmarks,...
                'GT', GT, ...
                'Position', env.pose0,...
                'LidarPose', lidarPose);
            
            %set initial position - mmove?
            obj.positionRegister(:, 1) = [env.pose0;0];
            obj.params.lidarPose = lidarPose;
            obj.OOI.expected = env.Context.Landmarks;
            
            % set initial lidars
            
            obj.menu.setActiveTab(obj.menu.ParameterControlTab);
            f = obj.menu.MTRN4010ControlCentreUIFigure;
            
        end
        
    end
    methods (Access = private)
        
        function addMeasurement(obj, measurement)
            arguments
                obj platform
                measurement (4, 1) double
            end
            obj.posIndex = obj.posIndex + 1;
            obj.positionRegister(:, obj.posIndex) = measurement;
        end
        
        function path = getPathVectors(obj)
            path = obj.positionRegister(:, 1:obj.posIndex);
        end
        
        function updateStatus(obj, status)
            obj.status = status;
            obj.menu.Status.Value=status;
        end
        
        function addToErrorBuffer(obj, error)
            arguments
                obj
                error (4,1) double
            end
            obj.errIndex = obj.errIndex + 1;
            if(obj.errIndex >= size(obj.errRegister, 2))
                obj.errRegister = cat(2, obj.errRegister, zeros(4, 2048));
            end
            
            obj.errRegister(:,obj.errIndex) = error;
        end
    end
    methods (Static)
        function computedVal = predictPose(time, x0, imu, model)
            arguments
                time (2, 1) uint32
                x0 (3, 1) double
                imu (2, 1) double
                model function_handle
            end
            time = double(time)*1e-4;
            t0 = time(1);
            dt = time(2) - t0;
            
            
            m = model(imu(1), x0(3), imu(2));
            computedVal = [x0(1:3)+dt*m;t0+dt*imu(1);dt*imu(2);m];
            
        end
        
        function globalC = localToGlobalCF(localC, rotation, translation)
            arguments
                localC (2,:) double
                rotation double
                translation (2,:) double
            end
            globalC = [cos(rotation), -sin(rotation);
                sin(rotation), cos(rotation)] * localC + translation;
            
        end
        
        function localC = globalToLocalCF(globalC, rotation, translation)
            arguments
                globalC (2,:) double
                rotation double
                translation (2,:) double
            end
            localC = [cos(rotation), sin(rotation);
                -sin(rotation), cos(rotation)] * (globalC-translation);
            
        end
        
        function [range, intensity] = scanToRI(scan)
            arguments
                scan (2, 301) uint16;
            end
            range = single(bitshift(bitshift(scan, 2), -2));
            range = range.*1e-2;      % convert to m
            intensity = uint8(bitshift(scan, -14));
            
        end
    end
end