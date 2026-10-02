classdef SensorSuite < handle
    %SENSORSUITE  Camera plus long- and short-range radar (Section 5.3).
    %
    %   Two interchangeable backends behind one detection contract:
    %
    %     'toolbox'    visionDetectionGenerator + drivingRadarDataGenerator.
    %                  Physically derived noise, occlusion and false alarms.
    %     'synthetic'  SyntheticSensor: ground truth passed through FOV,
    %                  range, Gaussian noise and dropout.  Pure MATLAB.
    %
    %   They are FUNCTIONALLY equivalent, not NUMERICALLY identical, and that
    %   distinction is deliberate.  The toolbox camera derives its measurement
    %   noise from bounding-box accuracy and the camera intrinsics, so its
    %   error grows with range in a way a fixed sigma cannot reproduce.  The
    %   synthetic backend uses the fixed sigmas in cfg.sensors.  Results are
    %   always labelled with the backend that produced them, and the measured
    %   RMSE of each is reported rather than assumed equal.
    %
    %   FRAMES
    %   The toolbox returns detections in EGO/BODY coordinates.  AdaptaDrive
    %   tracks, plans and reasons in the WORLD frame, so detections are
    %   transformed here, covariance included (R_world = C*R_ego*C').  Tracking
    %   in the ego frame would make every stationary object appear to move
    %   whenever the ego does.
    %
    %   DETECTION CONTRACT (Section 4), one struct array element per detection
    %       time      s, simulation time
    %       sensor    'camera' | 'radarLong' | 'radarShort'
    %       pos       [x y] world frame
    %       vel       [vx vy] world frame; NaN for the camera
    %       hasVel    logical
    %       classId   0..7; camera only, 0 elsewhere and when unclassified
    %       noiseCov  2x2 position covariance, world frame
    %       targetId  ActorID the simulator attributes it to (association and
    %                 evaluation only; never read by the planner)
    %
    %   See also SYNTHETICSENSOR, TRACKERWRAPPER, CAMERACLASSMODEL.

    properties (SetAccess = private)
        cfg
        backend
        cam
        radarLong
        radarShort
        synthetic
        rs                  % class-model stream, seeded per run
        lastValidTime = -inf
        stats               % running counts, for the M2 report
    end

    methods
        function obj = SensorSuite(cfg, scenario)
            obj.cfg = cfg;
            obj.backend = cfg.env.backends.sensors;

            % Never claim a backend that is not really there.
            if strcmp(obj.backend, 'toolbox') && ...
                    (isempty(scenario.scenarioObj) || ...
                     exist('visionDetectionGenerator', 'file') == 0)
                obj.backend = 'synthetic';
            end

            obj.rs = RandStream('mrg32k3a', 'Seed', cfg.seed);
            obj.rs.Substream = 7;

            obj.stats = struct('camDets', 0, 'radDets', 0, 'frames', 0, ...
                'classCorrect', 0, 'classWrong', 0, 'classUnknown', 0);

            switch obj.backend
                case 'toolbox'
                    obj.buildToolboxSensors(cfg, scenario);
                otherwise
                    obj.synthetic = SyntheticSensor(cfg);
            end
        end

        % ----------------------------------------------------------------
        function [dets, raw] = step(obj, t, egoState, scenario)
            %STEP  Produce detections for time t.
            %
            %   DETS is the contract struct array in world coordinates.
            %   RAW is a cell of objectDetection for the toolbox tracker, or
            %   {} when that class is unavailable.

            switch obj.backend
                case 'toolbox'
                    [dets, raw] = obj.stepToolbox(t, egoState, scenario);
                otherwise
                    dets = obj.synthetic.step(t, egoState, scenario);
                    raw = SensorSuite.toObjectDetections(dets, t, obj.cfg);
            end

            obj.stats.frames = obj.stats.frames + 1;
        end

        % ----------------------------------------------------------------
        function s = report(obj)
            s = obj.stats;
            s.backend = obj.backend;
        end
    end

    % ====================================================================
    methods (Access = private)

        function buildToolboxSensors(obj, cfg, scenario)
            %BUILDTOOLBOXSENSORS  Configure the ADT generators.

            c = SensorSuite.applyStress(cfg.sensors.camera, cfg);
            r = cfg.sensors.radar;

            % visionDetectionGenerator.FieldOfView is READ-ONLY on R2026a: it
            % is derived from the camera intrinsics.  Solve the intrinsics
            % that produce the horizontal FOV the specification asks for
            % (DEVIATIONS D14).
            %
            % Resolution is 1920x1080, a standard automotive front camera.
            % It is not a free parameter: detection also requires the target
            % to exceed MinObjectImageSize (15x15 px by default), so
            % resolution sets the range at which each class becomes visible.
            % At 640x480 a 0.6 m pedestrian subtends 15 px only within ~22 m,
            % which would make a "70 m camera" meaningless for exactly the
            % vulnerable classes this project is about.  The measured
            % per-class recall in the M2 report is the evidence for this
            % choice, and it is reported rather than assumed.
            imgW = 1920; imgH = 1080;
            fx = imgW / (2 * tan(c.fov / 2));
            intr = cameraIntrinsics([fx fx], [imgW/2 imgH/2], [imgH imgW]);

            obj.cam = visionDetectionGenerator( ...
                'SensorIndex', 1, ...
                'SensorLocation', [cfg.vehicle.wheelbase, 0], ...
                'Intrinsics', intr, ...
                'MaxRange', c.range, ...
                'UpdateInterval', 1 / cfg.sensors.rate, ...
                'DetectionProbability', 1 - c.dropout, ...
                'FalsePositivesPerImage', c.falsePositivesPerImage, ...
                'ActorProfiles', actorProfiles(scenario.scenarioObj));

            profiles = actorProfiles(scenario.scenarioObj);

            obj.radarLong = drivingRadarDataGenerator( ...
                'SensorIndex', 2, ...
                'MountingLocation', [cfg.vehicle.length - cfg.vehicle.rearOverhang, 0, 0.2], ...
                'FieldOfView', [rad2deg(r.fovLong), 5], ...
                'RangeLimits', [0, r.rangeLong], ...
                'UpdateRate', cfg.sensors.rate, ...
                'TargetReportFormat', 'Detections', ...
                'DetectionCoordinates', 'Body', ...
                'HasRangeRate', true, ...
                'HasNoise', true, ...
                'HasFalseAlarms', false, ...
                'DetectionProbability', 1 - r.dropout, ...
                'Profiles', profiles);

            obj.radarShort = drivingRadarDataGenerator( ...
                'SensorIndex', 3, ...
                'MountingLocation', [cfg.vehicle.length - cfg.vehicle.rearOverhang, 0, 0.2], ...
                'FieldOfView', [rad2deg(r.fovShort), 5], ...
                'RangeLimits', [0, r.rangeShort], ...
                'UpdateRate', cfg.sensors.rate, ...
                'TargetReportFormat', 'Detections', ...
                'DetectionCoordinates', 'Body', ...
                'HasRangeRate', true, ...
                'HasNoise', true, ...
                'HasFalseAlarms', false, ...
                'DetectionProbability', 1 - r.dropout, ...
                'Profiles', profiles);
        end

        % ----------------------------------------------------------------
        function d = degradeCamera(obj, d)
            %DEGRADECAMERA  B5 sensor stress, applied to camera detections.
            %
            %   `visionDetectionGenerator.DetectionProbability` already carries
            %   the nominal dropout, but its noise model is internal and there is
            %   no property for "add this much position error". So the stress
            %   condition is applied to the reported detection instead: extra
            %   dropout on top of the sensor's own, and extra position noise.
            %
            %   The reported measurement covariance is widened to MATCH the noise
            %   added. Degrading a measurement without telling the tracker it is
            %   degraded would not be a sensor-stress test - it would be a test
            %   of a tracker being lied to, and the result would say nothing
            %   about robustness to a poor camera.
            if ~isfield(obj.cfg, 'stress') || strcmp(obj.cfg.stress.name, 'none')
                return
            end
            extra = obj.cfg.stress.cameraDropout;
            if ~isempty(extra) && extra > obj.cfg.sensors.camera.dropout
                % Only the ADDITIONAL dropout: the generator already applied
                % the nominal rate, so re-applying it would compound.
                pAdd = (extra - obj.cfg.sensors.camera.dropout) / ...
                       max(1 - obj.cfg.sensors.camera.dropout, 1e-6);
                if rand(obj.rs) < pAdd
                    d = [];
                    return
                end
            end
            sig = obj.cfg.stress.cameraPosSigma;
            if ~isempty(sig) && sig > obj.cfg.sensors.camera.posSigma
                add = sqrt(max(sig^2 - obj.cfg.sensors.camera.posSigma^2, 0));
                d.pos = d.pos + add * randn(obj.rs, 1, 2);
                d.noiseCov = d.noiseCov + add^2 * eye(2);
            end
        end

        function [dets, raw] = stepToolbox(obj, t, egoState, scenario)
            dets = SensorSuite.emptyDetections();
            raw = {};

            tp = targetPoses(scenario.egoActor);
            if isempty(tp)
                return
            end

            collected = {};

            % --- camera ------------------------------------------------
            [cd, nc, validC] = obj.cam(tp, t);
            if validC && nc > 0
                for k = 1:numel(cd)
                    d = SensorSuite.fromObjectDetection(cd{k}, 'camera', ...
                        t, egoState, false);
                    d = obj.attachCameraClass(d, scenario);
                    d = obj.degradeCamera(d);
                    if isempty(d)
                        continue        % dropped by the stress condition
                    end
                    collected{end+1} = d; %#ok<AGROW>
                end
                obj.stats.camDets = obj.stats.camDets + numel(cd);
            end

            % --- radars ------------------------------------------------
            radars = {obj.radarLong, 'radarLong'; obj.radarShort, 'radarShort'};
            for i = 1:size(radars, 1)
                [rd, nr, validR] = radars{i, 1}(tp, t);
                if ~validR || nr == 0
                    continue
                end
                for k = 1:numel(rd)
                    d = SensorSuite.fromObjectDetection(rd{k}, radars{i, 2}, ...
                        t, egoState, true);
                    collected{end+1} = d; %#ok<AGROW>
                end
                obj.stats.radDets = obj.stats.radDets + numel(rd);
            end

            if ~isempty(collected)
                dets = [collected{:}];
            end

            % The tracker is fed detections rebuilt from OUR world-frame
            % contract, not the toolbox objects themselves.  Two reasons, both
            % load-bearing:
            %   1. Frame.  The generators report in ego/body coordinates.
            %      Tracking there would make every stationary object appear to
            %      accelerate whenever the ego does.
            %   2. Attribute shape.  Camera detections carry
            %      ObjectAttributes{1} = struct(TargetIndex), radar carries
            %      struct(TargetIndex, SNR).  multiObjectTracker concatenates
            %      the attributes of all detections assigned to a track, and
            %      horzcat of two different struct layouts errors out
            %      (DEVIATIONS D16).
            % Rebuilding also means both backends hand the tracker identical
            % input, so a tracker comparison is a fair one.
            raw = SensorSuite.toObjectDetections(dets, t, obj.cfg);
        end

        % ----------------------------------------------------------------
        function d = attachCameraClass(obj, d, scenario)
            %ATTACHCAMERACLASS  Run the classifier model on a camera detection.
            d.classId = 0;
            if isnan(d.targetId) || ~isKey(scenario.actorIdToAgentIdx, d.targetId)
                return   % false alarm: no object behind it, so no class
            end
            idx = scenario.actorIdToAgentIdx(d.targetId);
            if idx < 1 || idx > numel(scenario.agents)
                return   % detection of an actor with no matching agent record
            end
            trueClass = scenario.agents(idx).classId;
            d.classId = cameraClassModel(trueClass, obj.rs);

            if d.classId == 0
                obj.stats.classUnknown = obj.stats.classUnknown + 1;
            elseif d.classId == trueClass
                obj.stats.classCorrect = obj.stats.classCorrect + 1;
            else
                obj.stats.classWrong = obj.stats.classWrong + 1;
            end
        end
    end

    % ====================================================================
    methods (Static)

        function c = applyStress(c, cfg)
            %APPLYSTRESS  Override camera quality for a B5 sensor-stress run.
            %
            %   The stress fields existed in defaultConfig but nothing read
            %   them, which is the worst of both worlds: the configuration
            %   advertises a capability the build does not have. They are read
            %   here, and `cfg.stress.name` is carried into the results so a
            %   stressed row can never be mistaken for a nominal one.
            %
            %   Camera only, deliberately. The stress condition in the spec is
            %   camera degradation (dropout and position noise); degrading the
            %   radar at the same time would make an unattributable result -
            %   if everything is worse, nothing is learned about which sensor
            %   the stack depends on.
            if ~isfield(cfg, 'stress')
                return
            end
            if ~isempty(cfg.stress.cameraDropout)
                c.dropout = cfg.stress.cameraDropout;
            end
            if ~isempty(cfg.stress.cameraPosSigma)
                c.posSigma = cfg.stress.cameraPosSigma;
            end
        end

        function d = emptyDetections()
            d = struct('time', {}, 'sensor', {}, 'pos', {}, 'vel', {}, ...
                'hasVel', {}, 'classId', {}, 'noiseCov', {}, 'targetId', {});
        end

        % ----------------------------------------------------------------
        function d = fromObjectDetection(od, sensorName, t, egoState, hasVel)
            %FROMOBJECTDETECTION  ADT detection -> our contract, ego -> world.

            m = od.Measurement;
            posEgo = double(m(1:2));
            if hasVel && numel(m) >= 5
                velEgo = double(m(4:5));
            else
                velEgo = [NaN; NaN];
            end

            c = cos(egoState.yaw);
            s = sin(egoState.yaw);
            C = [c, -s; s, c];

            posW = C * posEgo(:) + [egoState.x; egoState.y];

            if hasVel && all(isfinite(velEgo))
                % Body-frame velocity is relative to the ego, so the ego's own
                % world velocity has to be added back.
                egoVelW = [egoState.v * c; egoState.v * s];
                velW = C * velEgo(:) + egoVelW;
            else
                velW = [NaN; NaN];
            end

            R = double(od.MeasurementNoise);
            Rpos = R(1:2, 1:2);
            RposW = C * Rpos * C.';

            tid = NaN;
            if ~isempty(od.ObjectAttributes)
                oa = od.ObjectAttributes{1};
                if isfield(oa, 'TargetIndex')
                    tid = double(oa.TargetIndex);
                end
            end

            d = struct( ...
                'time', t, ...
                'sensor', sensorName, ...
                'pos', posW(:).', ...
                'vel', velW(:).', ...
                'hasVel', hasVel && all(isfinite(velW)), ...
                'classId', 0, ...
                'noiseCov', RposW, ...
                'targetId', tid);
        end

        % ----------------------------------------------------------------
        function raw = toObjectDetections(dets, t, cfg)
            %TOOBJECTDETECTIONS  Our contract -> objectDetection for the
            %   toolbox tracker, so the synthetic backend can also drive it.
            raw = {};
            if exist('objectDetection', 'file') == 0 || isempty(dets)
                return
            end

            velSigma = 1.5;
            if nargin >= 3 && isfield(cfg, 'tracking') && isfield(cfg.tracking, 'velSigma')
                velSigma = cfg.tracking.velSigma;
            end
            usePositionOnly = isnan(velSigma);

            if usePositionOnly
                % Track on position alone and let the constant-velocity filter
                % infer velocity.  Uniform 3-element measurements, so no
                % MeasurementParameters are needed.
                for k = 1:numel(dets)
                    d = dets(k);
                    R = zeros(3);
                    R(1:2, 1:2) = d.noiseCov;
                    R(3, 3) = 100;
                    raw{end+1, 1} = objectDetection(t, [d.pos(:); 0], ...
                        'MeasurementNoise', R, ...
                        'SensorIndex', SensorSuite.sensorIndex(d.sensor)); %#ok<AGROW>
                end
                return
            end
            % multiObjectTracker requires a UNIFORM measurement size across all
            % sensors: it builds one sample detection at setup and validates
            % every later detection against it, so mixing a 3-element camera
            % measurement with a 6-element radar one errors inside
            % initcvekf (DEVIATIONS D18).
            %
            % Every detection is therefore emitted as [x y z vx vy vz].  The
            % camera does not measure velocity, so its velocity entries are
            % zero with a variance of 1e6 - a Kalman gain of order 1e-6, i.e.
            % the filter ignores them.  This is the standard way to express
            % "not observed" to a fixed-size filter; it is not a claim that
            % the camera measured zero velocity.
            % initcvekf infers the measurement layout from MeasurementParameters.
            % Without it, a 6-element measurement is rejected with "Expected
            % Detection.Measurement to be an array with number of elements
            % equal to 3" - it assumes position only (DEVIATIONS D18).
            measParams = struct('Frame', 'rectangular', 'HasVelocity', true);

            unobserved = 1e6;
            for k = 1:numel(dets)
                d = dets(k);
                R = zeros(6);
                R(1:2, 1:2) = d.noiseCov;
                R(3, 3) = 100;              % z position: not observed
                R(6, 6) = unobserved;       % z velocity: not observed
                if d.hasVel
                    meas = [d.pos(:); 0; d.vel(:); 0];
                    R(4:5, 4:5) = eye(2) * velSigma^2;
                else
                    meas = [d.pos(:); 0; 0; 0; 0];
                    R(4:5, 4:5) = eye(2) * unobserved;
                end
                % No ObjectAttributes: the tracker concatenates them across
                % detections and differing struct layouts error (D16).  The
                % target id we need for evaluation lives in our own contract.
                raw{end+1, 1} = objectDetection(t, meas, ...
                    'MeasurementNoise', R, ...
                    'MeasurementParameters', measParams, ...
                    'SensorIndex', SensorSuite.sensorIndex(d.sensor)); %#ok<AGROW>
            end
        end

        % ----------------------------------------------------------------
        function i = sensorIndex(name)
            switch name
                case 'camera',     i = 1;
                case 'radarLong',  i = 2;
                case 'radarShort', i = 3;
                otherwise,         i = 4;
            end
        end
    end
end
