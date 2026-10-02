classdef SimpleKFTracker < handle
    %SIMPLEKFTRACKER  Pure-MATLAB fallback tracker: CV Kalman + GNN.
    %
    %   Constant-velocity Kalman filter per track, state [x vx y vy], with
    %   global-nearest-neighbour association solved by MATCHPAIRS (Munkres).
    %   Used when Automated Driving Toolbox is unavailable, and also as an
    %   independent cross-check on TrackerWrapper.
    %
    %   Association cost is the squared Mahalanobis distance in the innovation
    %   covariance, not raw Euclidean distance.  That matters here: a distant,
    %   uncertain track should be allowed to claim a detection a few metres
    %   away, while a well-observed pedestrian a metre off must not be.
    %
    %   Track lifecycle matches the toolbox wrapper so the two are comparable:
    %   confirm on 2 hits in the first 3 updates, delete after 5 consecutive
    %   misses.
    %
    %   See also TRACKERWRAPPER, SENSORSUITE.

    properties (SetAccess = private)
        cfg
        tracks          % internal struct array
        nextId = 1
        lastTime = NaN
        gate = 30       % chi-square gate on squared Mahalanobis distance
        confirmHits = 2
        confirmWindow = 3
        deleteMisses = 5
        q = 4.0         % process-noise intensity, m^2/s^3
    end

    methods
        function obj = SimpleKFTracker(cfg)
            obj.cfg = cfg;
            obj.tracks = SimpleKFTracker.emptyInternal();
        end

        % ----------------------------------------------------------------
        function out = step(obj, t, dets, ~)
            %STEP  Predict, associate, update, manage.

            if isnan(obj.lastTime)
                dt = 1 / obj.cfg.sensors.rate;
            else
                dt = max(t - obj.lastTime, 1e-3);
            end
            obj.lastTime = t;

            obj.predict(dt);
            obj.associateAndUpdate(dets);
            obj.manage(t);

            out = obj.exportTracks(t);
        end

        function n = numTracks(obj)
            n = sum([obj.tracks.confirmed]);
        end
    end

    % ====================================================================
    methods (Access = private)

        function predict(obj, dt)
            F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
            % Continuous white-noise acceleration model.
            q1 = obj.q * [dt^3/3, dt^2/2; dt^2/2, dt];
            Q = blkdiag(q1, q1);
            for k = 1:numel(obj.tracks)
                obj.tracks(k).x = F * obj.tracks(k).x;
                obj.tracks(k).P = F * obj.tracks(k).P * F.' + Q;
                obj.tracks(k).missed = obj.tracks(k).missed + 1;
            end
        end

        % ----------------------------------------------------------------
        function associateAndUpdate(obj, dets)
            if isempty(dets)
                return
            end
            nT = numel(obj.tracks);
            nD = numel(dets);

            if nT == 0
                for j = 1:nD
                    obj.spawn(dets(j));
                end
                return
            end

            % --- cost matrix: squared Mahalanobis distance ---------------
            cost = inf(nT, nD);
            for i = 1:nT
                for j = 1:nD
                    cost(i, j) = obj.mahalanobis(obj.tracks(i), dets(j));
                end
            end
            cost(cost > obj.gate) = inf;

            % matchpairs needs a finite cost of non-assignment; the gate is
            % exactly that boundary.
            unassignedCost = obj.gate / 2;
            finite = cost;
            finite(isinf(finite)) = 1e6;
            M = matchpairs(finite, unassignedCost);

            assignedD = false(1, nD);
            for m = 1:size(M, 1)
                i = M(m, 1);
                j = M(m, 2);
                if ~isfinite(cost(i, j))
                    continue      % rejected by the gate
                end
                obj.update(i, dets(j));
                assignedD(j) = true;
            end

            for j = find(~assignedD)
                obj.spawn(dets(j));
            end
        end

        % ----------------------------------------------------------------
        function d2 = mahalanobis(obj, tr, det)
            [H, R, z] = SimpleKFTracker.measurementModel(det);
            S = H * tr.P * H.' + R;
            nu = z - H * tr.x;
            d2 = nu.' * (S \ nu);
        end

        % ----------------------------------------------------------------
        function update(obj, i, det)
            [H, R, z] = SimpleKFTracker.measurementModel(det);
            tr = obj.tracks(i);

            S = H * tr.P * H.' + R;
            K = tr.P * H.' / S;
            tr.x = tr.x + K * (z - H * tr.x);
            tr.P = (eye(4) - K * H) * tr.P;
            tr.P = 0.5 * (tr.P + tr.P.');    % keep it symmetric

            tr.missed = 0;
            tr.hits = tr.hits + 1;
            tr.updates = tr.updates + 1;

            if det.classId >= 1
                tr.classVotes(det.classId) = tr.classVotes(det.classId) + 1;
            end

            obj.tracks(i) = tr;
        end

        % ----------------------------------------------------------------
        function spawn(obj, det)
            x = [det.pos(1); 0; det.pos(2); 0];
            if det.hasVel && all(isfinite(det.vel))
                x(2) = det.vel(1);
                x(4) = det.vel(2);
                velVar = 1.0;
            else
                velVar = 100;    % no velocity measured: wide prior
            end

            P = blkdiag([det.noiseCov(1,1), 0; 0, velVar], ...
                        [det.noiseCov(2,2), 0; 0, velVar]);

            votes = zeros(1, 7);
            if det.classId >= 1
                votes(det.classId) = 1;
            end

            tr = struct('id', obj.nextId, 'x', x, 'P', P, ...
                'hits', 1, 'missed', 0, 'updates', 1, ...
                'confirmed', false, 'born', det.time, 'classVotes', votes);
            obj.nextId = obj.nextId + 1;
            obj.tracks(end+1) = tr;
        end

        % ----------------------------------------------------------------
        function manage(obj, ~)
            keep = true(1, numel(obj.tracks));
            for k = 1:numel(obj.tracks)
                tr = obj.tracks(k);
                if ~tr.confirmed && tr.updates <= obj.confirmWindow && ...
                        tr.hits >= obj.confirmHits
                    obj.tracks(k).confirmed = true;
                end
                if tr.missed >= obj.deleteMisses
                    keep(k) = false;
                elseif ~tr.confirmed && tr.updates > obj.confirmWindow
                    keep(k) = false;   % never confirmed in its window
                end
            end
            obj.tracks = obj.tracks(keep);
        end

        % ----------------------------------------------------------------
        function out = exportTracks(obj, t)
            out = TrackerWrapper.emptyTracks();
            conf = obj.tracks([obj.tracks.confirmed]);
            if isempty(conf)
                return
            end
            cells = cell(1, numel(conf));
            for k = 1:numel(conf)
                tr = conf(k);
                vx = tr.x(2); vy = tr.x(4);
                sp = hypot(vx, vy);
                if sp > 0.3
                    heading = atan2(vy, vx);
                else
                    heading = NaN;
                end
                [best, arg] = max(tr.classVotes);
                if best == 0
                    cls = 0;
                else
                    cls = arg;
                end
                cells{k} = struct( ...
                    'trackId', tr.id, 'classId', cls, ...
                    'x', tr.x(1), 'y', tr.x(3), 'vx', vx, 'vy', vy, ...
                    'speed', sp, 'heading', heading, ...
                    'P', tr.P([1 3], [1 3]), ...
                    'age', t - tr.born, 'updates', tr.updates, 'confirmed', true);
            end
            out = [cells{:}];
        end
    end

    % ====================================================================
    methods (Static, Access = private)
        function [H, R, z] = measurementModel(det)
            if det.hasVel && all(isfinite(det.vel))
                H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
                z = [det.pos(1); det.pos(2); det.vel(1); det.vel(2)];
                R = blkdiag(det.noiseCov, eye(2) * 0.5^2);
            else
                H = [1 0 0 0; 0 0 1 0];
                z = [det.pos(1); det.pos(2)];
                R = det.noiseCov;
            end
        end

        function s = emptyInternal()
            s = struct('id', {}, 'x', {}, 'P', {}, 'hits', {}, 'missed', {}, ...
                'updates', {}, 'confirmed', {}, 'born', {}, 'classVotes', {});
        end
    end
end
