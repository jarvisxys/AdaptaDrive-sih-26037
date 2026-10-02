classdef WrongWayDetector < handle
    %WRONGWAYDETECTOR  Requirement A2: vehicles travelling against the flow.
    %
    %   For every vehicle-class track moving faster than 1.5 m/s, compare its
    %   heading with the road's expected travel direction at its position.
    %   Flag when the deviation exceeds 120 degrees for 5 consecutive tracker
    %   updates (0.5 s).
    %
    %   WHY SUSTAINED, NOT INSTANT
    %   Heading from a tracked velocity is noisy at low speed, and a vehicle
    %   swerving round a pothole can momentarily point 130 degrees away from
    %   the flow.  A single-frame test would light up constantly, and a
    %   detector that cries wolf gets ignored - or worse, drives the planner
    %   into avoidance manoeuvres for nothing.  The counter resets on any
    %   conforming frame, so only sustained opposition survives.
    %
    %   WHERE IT STAYS SILENT
    %   Junctions have no single expected direction, so RoadModel reports the
    %   field as undefined there and this detector abstains rather than
    %   guessing.  Tracks below 1.5 m/s are skipped: a pedestrian or a stopped
    %   vehicle has no meaningful travel direction to contradict.
    %
    %   It also abstains NEAR THE CENTRELINE of a two-way road, and that guard
    %   is not cosmetic.  The expected-direction field flips sign at the
    %   centreline, and tracked positions carry over a metre of error, so a
    %   perfectly lawful oncoming vehicle at 1.5 m offset can be placed on the
    %   far side by noise - whereupon its heading opposes the field by 180
    %   degrees and it is flagged.  Measured before the guard: 2 false
    %   wrong-way flags on the village scenario, which contains no wrong-way
    %   vehicle at all.  The ambiguity band scales with the track's own
    %   lateral uncertainty, so a well-localised vehicle can be judged closer
    %   to the centre than a poorly localised one.
    %
    %   OUTPUT feeds two places: the event log (with time-to-flag measured
    %   against ground truth for the A2 metric) and the risk map, where a
    %   flagged agent's contribution is multiplied by cfg.risk.wrongWayMultiplier.
    %
    %   See also MERGEDETECTOR, RISKMAP, ROADMODEL.

    properties (SetAccess = private)
        cfg
        road
        counts          % containers.Map: trackId -> consecutive violating updates
        flagged         % containers.Map: trackId -> time first flagged
        devAtFlag       % containers.Map: trackId -> deviation (deg) at flag
    end

    properties (Constant)
        MinSpeed = 1.5;         % m/s
        DeviationDeg = 120;     % degrees
        SustainUpdates = 5;     % consecutive updates (0.5 s at 10 Hz)
        CentreBandM = 1.0;      % minimum centreline ambiguity band
    end

    methods
        function obj = WrongWayDetector(cfg, road)
            obj.cfg = cfg;
            obj.road = road;
            obj.reset();
        end

        function reset(obj)
            obj.counts    = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.flagged   = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.devAtFlag = containers.Map('KeyType', 'double', 'ValueType', 'double');
        end

        % ----------------------------------------------------------------
        function [ids, events] = step(obj, t, tracks)
            %STEP  Returns currently flagged track ids and any new events.
            ids = [];
            events = struct('trackId', {}, 'time', {}, 'deviationDeg', {}, 'text', {});

            for k = 1:numel(tracks)
                tr = tracks(k);

                if ~obj.isVehicleClass(tr.classId)
                    continue
                end
                speed = hypot(tr.vx, tr.vy);
                if speed < obj.MinSpeed
                    obj.clearCount(tr.trackId);
                    continue
                end

                [cx, cy, defined] = obj.road.expectedDirection(tr.x, tr.y);
                if ~defined
                    obj.clearCount(tr.trackId);   % junction: abstain
                    continue
                end

                % Centreline ambiguity: the expected direction flips sign
                % here, so a lawful oncoming vehicle placed across the line by
                % track noise would read as a 180 degree violation.  Abstain
                % inside a band that widens with the track's own uncertainty.
                info = obj.road.nearest(tr.x, tr.y);
                band = obj.CentreBandM;
                if ~isempty(tr.P) && all(isfinite(tr.P(:)))
                    band = max(band, sqrt(max(abs(eig(tr.P)))));
                end
                if abs(info.d) < band
                    obj.clearCount(tr.trackId);
                    continue
                end

                % Angle between travel direction and expected direction.
                dotp = (tr.vx * cx + tr.vy * cy) / speed;
                dotp = min(max(dotp, -1), 1);
                devDeg = acosd(dotp);

                if devDeg > obj.DeviationDeg
                    n = obj.bump(tr.trackId);
                    if n >= obj.SustainUpdates && ~isKey(obj.flagged, tr.trackId)
                        obj.flagged(tr.trackId) = t;
                        obj.devAtFlag(tr.trackId) = devDeg;
                        events(end+1) = struct( ...
                            'trackId', tr.trackId, 'time', t, ...
                            'deviationDeg', devDeg, ...
                            'text', sprintf('WRONG_WAY flagged: track #%d (%.0f deg, %.2f s)', ...
                                tr.trackId, devDeg, t)); %#ok<AGROW>
                    end
                else
                    obj.clearCount(tr.trackId);
                end

                if isKey(obj.flagged, tr.trackId)
                    ids(end+1) = tr.trackId; %#ok<AGROW>
                end
            end
        end

        % ----------------------------------------------------------------
        function tf = isFlagged(obj, trackId)
            tf = isKey(obj.flagged, trackId);
        end

        function tm = flagTime(obj, trackId)
            if isKey(obj.flagged, trackId)
                tm = obj.flagged(trackId);
            else
                tm = NaN;
            end
        end
    end

    methods (Access = private)
        function n = bump(obj, id)
            if isKey(obj.counts, id)
                n = obj.counts(id) + 1;
            else
                n = 1;
            end
            obj.counts(id) = n;
        end

        function clearCount(obj, id)
            % Only the streak resets.  An agent already flagged stays flagged:
            % a wrong-way vehicle that briefly straightens has not become safe.
            if isKey(obj.counts, id)
                obj.counts(id) = 0;
            end
        end

        function tf = isVehicleClass(~, classId)
            % Vehicle classes only: car, bus, auto, two-wheeler.  A pedestrian
            % or a cow walking against the traffic is not "wrong way", it is
            % Tuesday.
            tf = ismember(classId, [1 2 3 4]);
        end
    end
end
