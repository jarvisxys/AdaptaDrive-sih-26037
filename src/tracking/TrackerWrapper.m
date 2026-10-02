classdef TrackerWrapper < handle
    %TRACKERWRAPPER  Track-level fusion of camera and radar (Section 5.4).
    %
    %   Wraps multiObjectTracker (EKF, constant velocity) and adds the one
    %   thing the toolbox tracker does not provide: a CLASS LABEL per track.
    %
    %   CLASS VOTING
    %   The camera reports a noisy class per detection (CAMERACLASSMODEL); the
    %   radar reports none.  A track's class is the majority vote over every
    %   camera class ever associated with it, so a single misclassification
    %   does not flip a pedestrian into a bus for one cycle and change the
    %   risk weighting under the planner.
    %
    %   multiObjectTracker does not expose its detection-to-track assignment,
    %   so the vote re-associates classified camera detections to the returned
    %   tracks by nearest position within a gate.  That is an APPROXIMATION of
    %   the tracker's own assignment and is stated as such: in dense clutter
    %   it can credit a class to the wrong neighbouring track.  The
    %   alternative - feeding ObjectClassID into the tracker - is worse, since
    %   multiObjectTracker then refuses to associate detections whose class
    %   disagrees, and a noisy classifier would shatter one object into a new
    %   track per misread.
    %
    %   Class 0 (unknown) is a legitimate, supported outcome, not an error: a
    %   track the camera has never classified keeps classId 0 and is handled
    %   by the deliberately conservative "unknown" row of CLASSPRIORS.
    %
    %   See also SIMPLEKFTRACKER, SENSORSUITE, CLASSPRIORS.

    properties (SetAccess = private)
        cfg
        tracker
        classVotes      % containers.Map: trackId -> 1x7 vote histogram
        firstSeen       % containers.Map: trackId -> time
        lastTracks
    end

    properties (Constant, Access = private)
        % State layout of initcvekf in 3-D: [x; vx; y; vy; z; vz]
        IX = 1; IVX = 2; IY = 3; IVY = 4;
        ClassGate = 4.0;   % m, radius for re-associating a camera class
    end

    methods
        function obj = TrackerWrapper(cfg)
            obj.cfg = cfg;

            % AssignmentThreshold is normalised distance (chi-square gate).
            % NOTE: multiObjectTracker takes ONE threshold for all sensors;
            % per-sensor gating would need trackerGNN.  See DEVIATIONS D15.
            tk = cfg.tracking;
            initFcn = @initAdaptaDriveCV;
            if isfield(tk, 'filterInit') && ~isempty(tk.filterInit)
                initFcn = tk.filterInit;
            end
            obj.tracker = multiObjectTracker( ...
                'FilterInitializationFcn', initFcn, ...
                'AssignmentThreshold', [tk.gate inf], ...
                'ConfirmationThreshold', tk.confirmThreshold, ...
                'DeletionThreshold', tk.deleteThreshold, ...
                'MaxNumTracks', tk.maxNumTracks);

            obj.classVotes = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.firstSeen  = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.lastTracks = TrackerWrapper.emptyTracks();
        end

        % ----------------------------------------------------------------
        function tracks = step(obj, t, dets, raw)
            %STEP  Update the tracker and return tracks in our contract.

            if isempty(raw)
                % multiObjectTracker still needs to be stepped so existing
                % tracks coast and eventually die; skipping would freeze them.
                try
                    confirmed = obj.tracker(cell(0, 1), t);
                catch
                    confirmed = [];
                end
            else
                confirmed = obj.tracker(raw, t);
            end

            tracks = obj.convert(confirmed, t);
            tracks = obj.applyClassVotes(tracks, dets, t);
            tracks = obj.mergeDuplicates(tracks);
            obj.lastTracks = tracks;
        end

        % ----------------------------------------------------------------
        function n = numTracks(obj)
            n = numel(obj.lastTracks);
        end
    end

    % ====================================================================
    methods (Access = private)

        function tracks = convert(obj, confirmed, t)
            %CONVERT  multiObjectTracker output -> the track contract.
            tracks = TrackerWrapper.emptyTracks();
            if isempty(confirmed)
                return
            end

            out = cell(1, numel(confirmed));
            for k = 1:numel(confirmed)
                tr = confirmed(k);
                st = tr.State;
                P = tr.StateCovariance;

                x  = st(obj.IX);
                vx = st(obj.IVX);
                y  = st(obj.IY);
                vy = st(obj.IVY);

                % Position covariance in [x y] order.
                idx = [obj.IX, obj.IY];
                Ppos = P(idx, idx);

                id = double(tr.TrackID);
                if ~isKey(obj.firstSeen, id)
                    obj.firstSeen(id) = t;
                end

                sp = hypot(vx, vy);
                if sp > 0.3
                    heading = atan2(vy, vx);
                else
                    heading = NaN;   % too slow to attribute a heading
                end

                out{k} = struct( ...
                    'trackId', id, ...
                    'classId', 0, ...
                    'x', x, 'y', y, 'vx', vx, 'vy', vy, ...
                    'speed', sp, ...
                    'heading', heading, ...
                    'P', Ppos, ...
                    'age', t - obj.firstSeen(id), ...
                    'updates', double(tr.Age), ...
                    'confirmed', true);
            end
            tracks = [out{:}];
        end

        % ----------------------------------------------------------------
        function tracks = applyClassVotes(obj, tracks, dets, ~)
            %APPLYCLASSVOTES  Majority vote of camera classes per track.
            if isempty(tracks)
                return
            end

            % Accumulate this cycle's classified camera detections.
            for i = 1:numel(dets)
                d = dets(i);
                if ~strcmp(d.sensor, 'camera') || d.classId < 1
                    continue
                end
                [nearest, dist] = obj.nearestTrack(tracks, d.pos);
                if isempty(nearest) || dist > obj.ClassGate
                    continue
                end
                id = tracks(nearest).trackId;
                if isKey(obj.classVotes, id)
                    v = obj.classVotes(id);
                else
                    v = zeros(1, 7);
                end
                v(d.classId) = v(d.classId) + 1;
                obj.classVotes(id) = v;
            end

            % Resolve each track's class from its accumulated history.
            for k = 1:numel(tracks)
                id = tracks(k).trackId;
                if ~isKey(obj.classVotes, id)
                    tracks(k).classId = 0;      % never classified: stays unknown
                    continue
                end
                v = obj.classVotes(id);
                [best, arg] = max(v);
                if best == 0
                    tracks(k).classId = 0;
                else
                    tracks(k).classId = arg;
                end
            end
        end

    end

    % ====================================================================
    % PUBLIC deliberately: mergeDuplicates is a pure function of the track
    % list, and testing it directly is the point.  Asserting on a whole run
    % instead means a merge defect surfaces as a stall 200 m later, which says
    % nothing about the cause.  See tests/testDeadlockRecovery.m.
    methods

        function tracks = mergeDuplicates(obj, tracks)
            %MERGEDUPLICATES  Collapse several tracks on one physical object.
            %
            %   Two cameras and two radars looking at the same vehicle, with a
            %   lax confirmation threshold ([2 4]) and a 1 s deletion grace,
            %   produce more confirmed tracks than there are objects.  Measured:
            %   6 tracks inside a 2 m circle for a single pushcart in village,
            %   33 tracks for 5 agents on the highway.
            %
            %   Duplicates are not a cosmetic problem.  Every copy is predicted
            %   forward independently and every copy is painted into the risk
            %   map, so N copies of one slow vehicle 12 m ahead produce a wall
            %   of predicted occupancy that one vehicle would not, and the ego
            %   yields to a crowd that is not there.  That was the village
            %   stall: six copies of a 0.9 m/s pushcart made a 6 m road
            %   impassable.
            %
            %   Clustering is by footprint overlap, not a fixed radius: what
            %   makes two tracks the same object is that they cannot both be
            %   there.
            %
            %   A cluster is FUSED, not thinned to a survivor, because both
            %   obvious ways to pick a survivor were measured and both failed,
            %   in opposite directions:
            %
            %     most updates     stable ids, but keeps the long-lived copy
            %                      that has been coasting and has drifted off
            %                      its object.  Highway position recall 0.546
            %                      with the track COUNT correct (5 for 5).
            %     smallest P       recovers recall to 0.963, but the winner of
            %                      the cluster changes as covariances fluctuate,
            %                      so track ids churn, the risk field flickers,
            %                      and control degrades - village fell to 76.8 m
            %                      and cattle left the road.
            %
            %   Fusing gives what each half was for: an information-weighted
            %   mean is as well localised as the best member, while the id comes
            %   from the oldest member so downstream state (prediction history,
            %   the wrong-way and merge detectors) stays attached to one object.
            if numel(tracks) < 2
                return
            end

            n = numel(tracks);
            cluster = zeros(1, n);      % cluster index per track, 0 = unassigned
            nc = 0;
            for i = 1:n
                if cluster(i) == 0
                    nc = nc + 1;
                    cluster(i) = nc;
                end
                for j = i+1:n
                    if cluster(j) ~= 0, continue, end
                    d = hypot(tracks(j).x - tracks(i).x, tracks(j).y - tracks(i).y);
                    % One fixed, deliberately TIGHT radius.  An earlier version
                    % scaled the radius with the class footprint, which for two
                    % cars reaches 1.8 m - and on the highway that fused tracks
                    % belonging to DIFFERENT vehicles.  The fused estimate then
                    % sat between two real cars and matched neither, taking
                    % position recall to 0.48 while the track count looked
                    % healthy.  Merging distinct objects is far worse than
                    % leaving a duplicate: a duplicate over-states an obstacle
                    % that is really there, whereas a bad merge invents a
                    % vehicle where there is none and loses two that exist.
                    % So the test is only ever "closer together than the sensors
                    % can distinguish".
                    if d <= obj.cfg.tracking.mergeMinGap
                        cluster(j) = cluster(i);
                    end
                end
            end

            if nc == n
                return      % nothing overlapped
            end

            out = tracks(1:nc);
            for k = 1:nc
                mem = tracks(cluster == k);
                if numel(mem) == 1
                    out(k) = mem;
                    continue
                end
                out(k) = obj.fuseCluster(mem);
            end

            % Ascending ids: downstream code and the UI both read better when
            % the list does not reshuffle every cycle.
            [~, byId] = sort([out.trackId]);
            tracks = out(byId);
        end
    end

    % ====================================================================
    methods (Access = private)

        function T = fuseCluster(~, mem)
            %FUSECLUSTER  Information-weighted fusion of co-located tracks.
            %
            %   Position and velocity are combined in information form, so a
            %   confident member dominates a vague one without either being
            %   discarded.  The fused covariance is then inflated by the SPREAD
            %   of the members: if four tracks disagree by a metre about where
            %   one object is, that metre of disagreement is real uncertainty and
            %   must survive into the risk map.  Without it, fusing four vague
            %   tracks would manufacture one confident one.
            T = mem(1);

            Y = zeros(2, 2); z = zeros(2, 1);
            for i = 1:numel(mem)
                Pi = mem(i).P(1:2, 1:2);
                % Guard a singular or tiny covariance: an exactly-zero P would
                % give infinite weight to one member.
                Pi = Pi + 1e-6 * eye(2);
                Wi = inv(Pi);
                Y = Y + Wi;
                z = z + Wi * [mem(i).x; mem(i).y];
            end
            Pf = inv(Y);
            xf = Pf * z;

            % Velocity: weight by the same position confidence.  The tracker's
            % velocity uncertainty is not independent of its position
            % uncertainty here - both come from the same filter - so reusing the
            % position weights is closer to the truth than treating them as
            % separate estimates would be.
            wv = zeros(1, numel(mem));
            for i = 1:numel(mem)
                wv(i) = 1 / (trace(mem(i).P(1:2, 1:2)) + 1e-6);
            end
            wv = wv / sum(wv);
            vxf = sum(wv .* [mem.vx]);
            vyf = sum(wv .* [mem.vy]);

            % Spread of the members about the fused point, added as variance.
            dx = [mem.x] - xf(1);
            dy = [mem.y] - xf(2);
            spread = max([dx.^2, dy.^2]);
            Pf = Pf + spread * eye(2);

            T.x = xf(1);
            T.y = xf(2);
            T.vx = vxf;
            T.vy = vyf;
            T.speed = hypot(vxf, vyf);
            if T.speed > 1e-6
                T.heading = atan2(vyf, vxf);
            end
            T.P(1:2, 1:2) = Pf;

            % Id from the OLDEST member (ids increase), so whatever downstream
            % state is keyed on trackId stays attached to this object.
            T.trackId  = min([mem.trackId]);
            T.updates  = max([mem.updates]);
            T.age      = max([mem.age]);
            T.confirmed = any([mem.confirmed]);

            % Class by vote among members that have one.  Class comes from the
            % camera, not from track age, so an identified member outranks an
            % unknown one however long the unknown has lived.
            ids = [mem.classId];
            ids = ids(ids ~= 0);
            if isempty(ids)
                T.classId = 0;
            else
                T.classId = mode(ids);
            end
        end

        % ----------------------------------------------------------------
        function [idx, dist] = nearestTrack(~, tracks, pos)
            d = hypot([tracks.x] - pos(1), [tracks.y] - pos(2));
            [dist, idx] = min(d);
            if isempty(dist)
                idx = [];
                dist = inf;
            end
        end
    end

    % ====================================================================
    methods (Static)
        function t = emptyTracks()
            t = struct('trackId', {}, 'classId', {}, 'x', {}, 'y', {}, ...
                'vx', {}, 'vy', {}, 'speed', {}, 'heading', {}, 'P', {}, ...
                'age', {}, 'updates', {}, 'confirmed', {});
        end
    end
end
