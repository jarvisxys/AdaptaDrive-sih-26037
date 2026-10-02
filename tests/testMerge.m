classdef testMerge < matlab.unittest.TestCase
    %TESTMERGE  Requirement A3 detector.
    %
    %   A merge needs BOTH cues: closing laterally on our corridor AND
    %   arriving where we will be. Each alone is ordinary traffic, so both
    %   are tested in isolation as negatives.

    properties
        cfg
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig();
        end
    end

    methods (Test)

        % ---------------- true positive ----------------

        function lateralApproachIntoCorridorTriggers(tc)
            % Ego at the origin heading +x at 10 m/s.  A slower car 15 m ahead
            % and 4 m to the left, doing 6 m/s and closing laterally at
            % 1.5 m/s.
            %
            % It enters the 1.6 m corridor at t = (4-1.6)/1.5 = 1.6 s, by
            % which point it has reached x = 15 + 6*1.6 = 24.6 m.  We reach
            % 24.6 m at 2.46 s.  Arriving 0.86 s apart is a genuine conflict.
            %
            % Note what does NOT qualify: a car 25 m ahead doing 9 m/s while
            % we do 10 m/s also drifts into the lane, but stays 20 m ahead
            % throughout.  That is an ordinary lane change well in front of
            % us, and flagging it would make the detector noise.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            tr = tc.track(1, 15, 4, 6, -1.5);

            [ids, ev, ttc] = d.step(1.0, tr, [], ego, []);

            tc.verifyNotEmpty(ids, 'A vehicle merging into the corridor must be flagged.');
            tc.verifyNotEmpty(ev);
            tc.verifyEqual(ev(1).trackId, 1);
            tc.verifyTrue(isfinite(ttc) && ttc > 0 && ttc <= 3.0, ...
                sprintf('Reported TTC %.2f s is outside the 3 s horizon.', ttc));
            tc.verifySubstring(ev(1).text, 'MERGE');
        end

        function eventIsRaisedOnceNotEveryCycle(tc)
            % The event log drives the UI timeline; one merge must be one
            % entry, not forty.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            total = 0;
            for k = 1:5
                [~, ev] = d.step(k * 0.1, tc.track(1, 15, 4, 6, -1.5), [], ego, []);
                total = total + numel(ev);
            end
            tc.verifyEqual(total, 1, 'A sustained merge must raise exactly one event.');
        end

        % ---------------- true negatives ----------------

        function parallelTravelDoesNotTrigger(tc)
            % Same direction, same speed, holding its line 5 m to the left.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            for k = 1:10
                [ids, ~] = d.step(k * 0.1, tc.track(1, 15, 4, 10, 0), [], ego, []);
                tc.verifyEmpty(ids, 'Parallel travel is not a merge.');
            end
        end

        function lateralDriftWithoutConflictDoesNotTrigger(tc)
            % Closing laterally, but 120 m ahead: no conflict inside 3 s.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            for k = 1:10
                [ids, ~] = d.step(k * 0.1, tc.track(1, 120, 5, 9, -2), [], ego, []);
                tc.verifyEmpty(ids, 'A distant drift is not a merge.');
            end
        end

        function slowLateralDriftDoesNotTrigger(tc)
            % 0.1 m/s lateral: below the 0.3 m/s cue.  Unmarked roads are full
            % of this and flagging it would make the detector useless.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            for k = 1:10
                [ids, ~] = d.step(k * 0.1, tc.track(1, 15, 4, 6, -0.1), [], ego, []);
                tc.verifyEmpty(ids, 'Ordinary lateral wander is not a merge.');
            end
        end

        function vehicleMovingAwayDoesNotTrigger(tc)
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            for k = 1:10
                [ids, ~] = d.step(k * 0.1, tc.track(1, 15, 4, 6, +1.5), [], ego, []);
                tc.verifyEmpty(ids, 'A vehicle moving away is not merging.');
            end
        end

        function pedestrianIsNotAMerge(tc)
            % A3 is about vehicles.  A crossing pedestrian satisfies the same
            % kinematics but is a different event, handled by the risk map.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            tr = tc.track(1, 15, 4, 6, -1.5);
            tr.classId = 5;
            for k = 1:10
                [ids, ~] = d.step(k * 0.1, tr, [], ego, []);
                tc.verifyEmpty(ids, 'A pedestrian crossing must not be logged as a merge.');
            end
        end

        function behindTheEgoDoesNotTrigger(tc)
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            for k = 1:10
                [ids, ~] = d.step(k * 0.1, tc.track(1, -30, 4, 6, -1.5), [], ego, []);
                tc.verifyEmpty(ids, 'Something behind us is not merging into our path.');
            end
        end

        % ---------------- uses predictions when available ----------------

        function usesPredictedPathWhenSupplied(tc)
            % With a predictor available the detector must use the predicted
            % mode rather than its constant-velocity fallback.
            d = MergeDetector(tc.cfg);
            ego = tc.ego(0, 0, 0, 10);
            tr = tc.track(3, 15, 4, 6, -1.5);

            T = round(tc.cfg.predict.horizon / tc.cfg.predict.dt);
            tv = (1:T)' * tc.cfg.predict.dt;
            mu = [tr.x + 6 * tv, tr.y - 1.5 * tv];
            preds = struct('trackId', 3, 'classId', 1, 'modes', ...
                struct('name', 'cv', 'prob', 1.0, 'mu', mu, ...
                       'Sigma', repmat(eye(2) * 0.5, 1, 1, T), ...
                       'heading', repmat(atan2(-1.5, 6), T, 1)));

            [ids, ~, ttc] = d.step(1.0, tr, preds, ego, []);
            tc.verifyNotEmpty(ids);
            tc.verifyTrue(isfinite(ttc));
        end
    end

    methods (Access = private)
        function e = ego(~, x, y, yaw, v)
            e = struct('x', x, 'y', y, 'yaw', yaw, 'v', v, 'a', 0, ...
                'steer', 0, 't', 0);
        end

        function tr = track(~, id, x, y, vx, vy)
            tr = struct('trackId', id, 'classId', 1, 'x', x, 'y', y, ...
                'vx', vx, 'vy', vy, 'speed', hypot(vx, vy), ...
                'heading', atan2(vy, vx), 'P', eye(2) * 0.25, ...
                'age', 1.0, 'updates', 10, 'confirmed', true);
        end
    end
end
