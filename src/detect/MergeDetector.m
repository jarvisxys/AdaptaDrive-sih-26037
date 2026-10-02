classdef MergeDetector < handle
    %MERGEDETECTOR  Requirement A3: informal merges into the ego's corridor.
    %
    %   Flag a track when BOTH hold:
    %     1. its lateral velocity toward the ego's planned corridor exceeds
    %        0.3 m/s, and
    %     2. its predicted path intersects the ego's planned trajectory within
    %        3 s.
    %
    %   Two conditions rather than one, because either alone is common and
    %   harmless.  Lateral drift happens constantly on an unmarked road - an
    %   auto wandering half a metre is not merging.  A path crossing ours in
    %   three seconds is also normal if the other vehicle is holding its line
    %   and will pass behind.  What makes it a merge is intent plus conflict:
    %   moving toward us AND arriving where we will be.
    %
    %   "Informal" is the operative word.  There is no indicator to read, so
    %   the cue has to be kinematic.
    %
    %   The reported TTC at the moment of triggering is the A3 metric, so it
    %   is measured from the geometry that caused the trigger rather than
    %   recomputed later from a different definition.
    %
    %   See also WRONGWAYDETECTOR, BEHAVIORFSM.

    properties (SetAccess = private)
        cfg
        active          % containers.Map: trackId -> time first detected
    end

    properties (Constant)
        LateralRate = 0.3;      % m/s toward the corridor
        HorizonS = 3.0;         % s, conflict lookahead
        CorridorHalfWidth = 1.6;% m, half-width of the ego's corridor
    end

    methods
        function obj = MergeDetector(cfg)
            obj.cfg = cfg;
            obj.reset();
        end

        function reset(obj)
            obj.active = containers.Map('KeyType', 'double', 'ValueType', 'double');
        end

        % ----------------------------------------------------------------
        function [ids, events, minTTC] = step(obj, t, tracks, preds, egoState, egoPath)
            %STEP  Detect merges against the ego's current planned path.
            %
            %   egoPath is an n x 2 polyline the ego intends to follow.  When
            %   it is empty the ego's current heading is used as the corridor,
            %   which is the honest fallback before a planner exists.

            ids = [];
            events = struct('trackId', {}, 'time', {}, 'ttc', {}, 'text', {});
            minTTC = inf;

            if isempty(tracks)
                return
            end

            [corridor, egoSpeed] = obj.buildCorridor(egoState, egoPath);

            for k = 1:numel(tracks)
                tr = tracks(k);

                % A3 is about VEHICLES merging into our path.  A pedestrian
                % stepping off the kerb satisfies the same kinematics, but it
                % is a different event with a different response, and counting
                % it here would pollute the A3 metric - the TTC at YIELD in
                % the highway merge scenario - with village crossings.
                % Measured before this guard: 7 "merges" on the village
                % scenario, which contains no merging vehicle.  Pedestrian
                % crossings reach the planner through the risk map and the
                % multi-modal predictor, which is where they belong.
                if ~ismember(tr.classId, [1 2 3 4])
                    obj.clear(tr.trackId);
                    continue
                end

                % --- condition 1: closing laterally on the corridor --------
                [lateralRate, lateralDist] = obj.lateralApproach(tr, corridor);
                if lateralRate < obj.LateralRate
                    obj.clear(tr.trackId);
                    continue
                end

                % --- condition 2: predicted conflict within the horizon ----
                ttc = obj.conflictTime(tr, preds, corridor, egoState, egoSpeed);
                if ~isfinite(ttc) || ttc > obj.HorizonS
                    obj.clear(tr.trackId);
                    continue
                end

                minTTC = min(minTTC, ttc);
                ids(end+1) = tr.trackId; %#ok<AGROW>

                if ~isKey(obj.active, tr.trackId)
                    obj.active(tr.trackId) = t;
                    events(end+1) = struct( ...
                        'trackId', tr.trackId, 'time', t, 'ttc', ttc, ...
                        'text', sprintf('MERGE detected: track #%d, TTC %.1f s (lateral %.2f m/s at %.1f m)', ...
                            tr.trackId, ttc, lateralRate, lateralDist)); %#ok<AGROW>
                end
            end

            if isinf(minTTC)
                minTTC = NaN;
            end
        end

        function tf = isMerging(obj, trackId)
            tf = isKey(obj.active, trackId);
        end
    end

    % ====================================================================
    methods (Access = private)

        function [corridor, egoSpeed] = buildCorridor(obj, egoState, egoPath)
            %BUILDCORRIDOR  Origin, direction and normal of the ego's corridor.
            egoSpeed = max(egoState.v, 0.1);

            dirVec = [cos(egoState.yaw), sin(egoState.yaw)];
            if ~isempty(egoPath) && size(egoPath, 1) >= 2
                % Use the path's local direction a little way ahead, which is
                % where a merging vehicle will actually meet us.
                d = hypot(egoPath(:,1) - egoState.x, egoPath(:,2) - egoState.y);
                [~, i0] = min(d);
                i1 = min(i0 + 6, size(egoPath, 1));
                if i1 > i0
                    v = egoPath(i1, :) - egoPath(i0, :);
                    if hypot(v(1), v(2)) > 1e-6
                        dirVec = v / hypot(v(1), v(2));
                    end
                end
            end

            corridor = struct( ...
                'origin', [egoState.x, egoState.y], ...
                'dir', dirVec, ...
                'normal', [-dirVec(2), dirVec(1)], ...
                'path', egoPath);
        end

        % ----------------------------------------------------------------
        function [rate, dist] = lateralApproach(~, tr, corridor)
            %LATERALAPPROACH  Speed of closing on the corridor centreline.
            rel = [tr.x, tr.y] - corridor.origin;
            lat = rel * corridor.normal.';        % signed offset
            vLat = [tr.vx, tr.vy] * corridor.normal.';

            dist = abs(lat);
            % Positive when the lateral velocity reduces the offset.
            rate = -sign(lat) * vLat;
            if lat == 0
                rate = 0;   % already on the centreline: nothing to close
            end
        end

        % ----------------------------------------------------------------
        function ttc = conflictTime(obj, tr, preds, corridor, egoState, egoSpeed)
            %CONFLICTTIME  First time the track enters the ego's corridor at a
            %   point the ego also reaches at about the same moment.

            ttc = inf;

            mu = obj.trackPrediction(tr, preds);
            dt = obj.cfg.predict.dt;

            for i = 1:size(mu, 1)
                tk = i * dt;
                if tk > obj.HorizonS
                    break
                end

                rel = mu(i, :) - corridor.origin;
                along = rel * corridor.dir.';
                lat   = abs(rel * corridor.normal.');

                if along < 0 || lat > obj.CorridorHalfWidth
                    continue        % not in the corridor at this instant
                end

                % The ego reaches "along" at roughly along/speed.  A conflict
                % needs both to be there at once, not merely both to pass
                % through eventually.
                egoArrival = along / egoSpeed;
                if abs(egoArrival - tk) < 1.5
                    ttc = tk;
                    return
                end
            end
        end

        % ----------------------------------------------------------------
        function mu = trackPrediction(obj, tr, preds)
            %TRACKPREDICTION  Highest-probability predicted path for a track.
            mu = [];
            for i = 1:numel(preds)
                if preds(i).trackId ~= tr.trackId
                    continue
                end
                [~, best] = max([preds(i).modes.prob]);
                mu = preds(i).modes(best).mu;
                return
            end

            % No prediction available: fall back to constant velocity so the
            % detector still works before the predictor is wired in.
            T = round(obj.HorizonS / obj.cfg.predict.dt);
            tvec = (1:T)' * obj.cfg.predict.dt;
            mu = [tr.x + tr.vx * tvec, tr.y + tr.vy * tvec];
        end

        function clear(obj, id)
            if isKey(obj.active, id)
                remove(obj.active, id);
            end
        end
    end
end
