classdef testWrongWay < matlab.unittest.TestCase
    %TESTWRONGWAY  Requirement A2 detector.
    %
    %   Both directions are tested, and that matters: the centreline
    %   ambiguity guard added after false positives on the village scenario
    %   could equally well have silenced the detector altogether.  The tests
    %   here prove it still fires on a genuine wrong-way vehicle within 0.5 s,
    %   and the negative tests prove the false positives are gone.

    properties
        cfg
        road
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig();
            % Straight road along +x, 7 m wide.  Keep-left: a vehicle at
            % d > 0 is expected to travel +x, one at d < 0 to travel -x.
            tc.road = RoadModel(struct('centers', [0 0; 200 0], 'width', 7, ...
                'name', 'straight'), 'res', 0.25);
        end
    end

    methods (Test)

        % ---------------- true positives ----------------

        function sustainedOppositeHeadingFlagsWithinHalfSecond(tc)
            % Vehicle at d = +2 (expected +x) driving -x at 8 m/s: 180 deg.
            d = WrongWayDetector(tc.cfg, tc.road);
            flaggedAt = NaN;
            for k = 1:10
                t = k * 0.1;
                tr = tc.track(1, 100, 2.0, -8, 0);
                [ids, ev] = d.step(t, tr);
                if ~isempty(ev) && isnan(flaggedAt)
                    flaggedAt = t;
                end
                if ~isempty(ids)
                    break
                end
            end
            tc.verifyFalse(isnan(flaggedAt), 'A 180 degree violation was never flagged.');
            tc.verifyLessThanOrEqual(flaggedAt, 0.5 + 1e-9, ...
                sprintf('Flagged at %.2f s; A2 requires within 0.5 s.', flaggedAt));
        end

        function headingOf170DegreesFlags(tc)
            % 170 deg deviation: still well past the 120 deg threshold.
            d = WrongWayDetector(tc.cfg, tc.road);
            v = 8 * [cosd(170), sind(170)];
            flagged = false;
            for k = 1:8
                [ids, ~] = d.step(k * 0.1, tc.track(1, 100, 2.0, v(1), v(2)));
                flagged = flagged || ~isempty(ids);
            end
            tc.verifyTrue(flagged, 'A 170 degree deviation must flag.');
        end

        function flaggedVehicleStaysFlagged(tc)
            % A wrong-way vehicle that momentarily straightens has not become
            % safe, so the flag must persist.
            d = WrongWayDetector(tc.cfg, tc.road);
            for k = 1:8
                d.step(k * 0.1, tc.track(1, 100, 2.0, -8, 0));
            end
            tc.verifyTrue(d.isFlagged(1));
            for k = 9:14
                d.step(k * 0.1, tc.track(1, 100, 2.0, 8, 0));
            end
            tc.verifyTrue(d.isFlagged(1), 'The flag must not be cleared by a brief straighten.');
        end

        % ---------------- true negatives ----------------

        function briefWobbleDoesNotFlag(tc)
            % 130 deg for 0.2 s (2 updates), then conforming.  Below the
            % 5-update sustain requirement, so it must not flag.
            d = WrongWayDetector(tc.cfg, tc.road);
            v = 8 * [cosd(130), sind(130)];
            for k = 1:2
                [ids, ~] = d.step(k * 0.1, tc.track(1, 100, 2.0, v(1), v(2)));
                tc.verifyEmpty(ids);
            end
            for k = 3:10
                [ids, ~] = d.step(k * 0.1, tc.track(1, 100, 2.0, 8, 0));
                tc.verifyEmpty(ids, 'A 0.2 s wobble must never flag.');
            end
        end

        function lawfulOncomingVehicleDoesNotFlag(tc)
            % The village false positive, reproduced directly.  A vehicle at
            % d = -2 travelling -x is on its correct side.
            d = WrongWayDetector(tc.cfg, tc.road);
            for k = 1:20
                [ids, ~] = d.step(k * 0.1, tc.track(1, 100, -2.0, -8, 0));
                tc.verifyEmpty(ids, 'A lawful oncoming vehicle must never flag.');
            end
        end

        function noisyOncomingVehicleNearCentrelineDoesNotFlag(tc)
            % The actual failure mode: an oncoming vehicle whose tracked
            % position wobbles across the centreline.  Without the ambiguity
            % guard this flags within half a second.
            d = WrongWayDetector(tc.cfg, tc.road);
            offsets = [-1.4, -0.6, 0.3, -0.2, 0.5, -0.9, 0.1, 0.4, -0.3, 0.2];
            for k = 1:numel(offsets)
                [ids, ~] = d.step(k * 0.1, tc.track(1, 100, offsets(k), -8, 0));
                tc.verifyEmpty(ids, sprintf( ...
                    'Flagged a lawful oncoming vehicle at centreline offset %.1f m.', ...
                    offsets(k)));
            end
        end

        function slowTrackIsIgnored(tc)
            % Below 1.5 m/s a heading is not meaningful.
            d = WrongWayDetector(tc.cfg, tc.road);
            for k = 1:20
                [ids, ~] = d.step(k * 0.1, tc.track(1, 100, 2.0, -0.8, 0));
                tc.verifyEmpty(ids);
            end
        end

        function pedestrianIsNeverWrongWay(tc)
            % A pedestrian walking against traffic is not a wrong-way vehicle.
            d = WrongWayDetector(tc.cfg, tc.road);
            for k = 1:20
                tr = tc.track(1, 100, 2.0, -3, 0);
                tr.classId = 5;
                [ids, ~] = d.step(k * 0.1, tr);
                tc.verifyEmpty(ids);
            end
        end

        function junctionIsIgnored(tc)
            % Where no travel direction is defined the detector must abstain.
            junction = RoadModel(struct('centers', [0 0; 60 0], 'width', 7, ...
                'name', 'box', 'directionDefined', false), 'res', 0.25);
            d = WrongWayDetector(tc.cfg, junction);
            for k = 1:20
                [ids, ~] = d.step(k * 0.1, tc.track(1, 30, 2.0, -8, 0));
                tc.verifyEmpty(ids, 'A junction has no expected direction to violate.');
            end
        end

        function reportsDeviationAndTime(tc)
            d = WrongWayDetector(tc.cfg, tc.road);
            ev = [];
            for k = 1:8
                [~, e] = d.step(k * 0.1, tc.track(7, 100, 2.0, -8, 0));
                if ~isempty(e), ev = e; break, end
            end
            tc.assertNotEmpty(ev, 'No event was produced.');
            tc.verifyEqual(ev(1).trackId, 7);
            tc.verifyGreaterThan(ev(1).deviationDeg, 120);
            tc.verifyGreaterThan(ev(1).time, 0);
            tc.verifySubstring(ev(1).text, 'WRONG_WAY');
        end
    end

    methods (Access = private)
        function tr = track(~, id, x, y, vx, vy)
            tr = struct('trackId', id, 'classId', 1, 'x', x, 'y', y, ...
                'vx', vx, 'vy', vy, 'speed', hypot(vx, vy), ...
                'heading', atan2(vy, vx), 'P', eye(2) * 0.25, ...
                'age', 1.0, 'updates', 10, 'confirmed', true);
        end
    end
end
