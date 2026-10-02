classdef SyntheticSensor < handle
    %SYNTHETICSENSOR  Pure-MATLAB sensor backend (graceful degradation).
    %
    %   Ground truth passed through field of view, range limit, Gaussian
    %   position noise and random dropout - the four effects that actually
    %   change what a planner has to cope with.  No toolbox required, so
    %   AdaptaDrive still runs end to end on a bare MATLAB installation.
    %
    %   WHAT IT DOES NOT MODEL, stated plainly: occlusion between objects,
    %   false alarms, range-dependent noise growth, and radar resolution
    %   cells.  The toolbox backend models all four.  A run made with this
    %   backend is therefore an EASIER perception problem, and check_env
    %   records which backend produced every result so the two are never
    %   averaged together.
    %
    %   See also SENSORSUITE.

    properties (SetAccess = private)
        cfg
        rs
    end

    methods
        function obj = SyntheticSensor(cfg)
            obj.cfg = cfg;
            obj.rs = RandStream('mrg32k3a', 'Seed', cfg.seed);
            obj.rs.Substream = 8;
        end

        % ----------------------------------------------------------------
        function dets = step(obj, t, egoState, scenario)
            dets = SensorSuite.emptyDetections();
            collected = {};

            c = obj.cfg.sensors.camera;
            r = obj.cfg.sensors.radar;

            % Sensor origin: front axle for the camera, front bumper for radar.
            camOrigin = [egoState.x, egoState.y] + ...
                obj.cfg.vehicle.wheelbase * [cos(egoState.yaw), sin(egoState.yaw)];
            radOrigin = [egoState.x, egoState.y] + ...
                (obj.cfg.vehicle.length - obj.cfg.vehicle.rearOverhang) * ...
                [cos(egoState.yaw), sin(egoState.yaw)];

            for i = 1:numel(scenario.agents)
                a = scenario.agents(i);
                if ~a.active
                    continue
                end
                truth = a.truth();
                actorId = obj.actorIdFor(scenario, i);

                % --- camera ---------------------------------------------
                if obj.inFov(camOrigin, egoState.yaw, [truth.x truth.y], c.fov, c.range) ...
                        && rand(obj.rs) > c.dropout
                    noise = c.posSigma * randn(obj.rs, 1, 2);
                    d = obj.makeDet(t, 'camera', [truth.x truth.y] + noise, ...
                        [NaN NaN], false, eye(2) * c.posSigma^2, actorId);
                    d.classId = cameraClassModel(truth.classId, obj.rs);
                    collected{end+1} = d; %#ok<AGROW>
                end

                % --- radars ---------------------------------------------
                specs = {r.fovLong, r.rangeLong, 'radarLong'; ...
                         r.fovShort, r.rangeShort, 'radarShort'};
                for k = 1:size(specs, 1)
                    if ~obj.inFov(radOrigin, egoState.yaw, [truth.x truth.y], ...
                            specs{k, 1}, specs{k, 2})
                        continue
                    end
                    if rand(obj.rs) <= r.dropout
                        continue
                    end
                    posNoise = r.posSigma * randn(obj.rs, 1, 2);
                    velNoise = r.velSigma * randn(obj.rs, 1, 2);
                    d = obj.makeDet(t, specs{k, 3}, ...
                        [truth.x truth.y] + posNoise, ...
                        [truth.vx truth.vy] + velNoise, true, ...
                        eye(2) * r.posSigma^2, actorId);
                    collected{end+1} = d; %#ok<AGROW>
                end
            end

            if ~isempty(collected)
                dets = [collected{:}];
            end
        end
    end

    methods (Access = private)
        function tf = inFov(~, origin, yaw, target, fov, range)
            rel = target - origin;
            d = hypot(rel(1), rel(2));
            if d > range || d < 1e-6
                tf = d <= range;
                return
            end
            ang = atan2(rel(2), rel(1)) - yaw;
            ang = mod(ang + pi, 2*pi) - pi;
            tf = abs(ang) <= fov / 2;
        end

        function d = makeDet(~, t, sensor, pos, vel, hasVel, R, targetId)
            d = struct('time', t, 'sensor', sensor, 'pos', pos, 'vel', vel, ...
                'hasVel', hasVel, 'classId', 0, 'noiseCov', R, 'targetId', targetId);
        end

        function id = actorIdFor(~, scenario, agentIdx)
            %ACTORIDFOR  Keep target ids consistent with the toolbox backend.
            id = agentIdx;
            if isfield(scenario, 'agentActors') && ~isempty(scenario.agentActors) ...
                    && numel(scenario.agentActors) >= agentIdx
                id = double(scenario.agentActors(agentIdx).ActorID);
            end
        end
    end
end
