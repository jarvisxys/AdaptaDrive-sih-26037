classdef testTTC < matlab.unittest.TestCase
    %TESTTTC  Head-on, crossing and parallel cases (Section 9).
    %
    %   Expected values are computed by hand from the closing geometry; the
    %   sampling step inside TTC is 0.02 s, so tolerances are set to one
    %   sample plus a margin rather than to an arbitrary "close enough".

    properties (Constant)
        SampleTol = 0.03;   % one 0.02 s sample, rounded up
    end

    methods (Test)

        function headOnClosing(tc)
            % Two 4 x 2 boxes, centres 24 m apart, closing at 10 + 10 = 20 m/s.
            % Gap between faces = 24 - 2 - 2 = 20 m -> contact at t = 1.0 s.
            a = makeOBB(0,  0, 0,  4, 2);
            b = makeOBB(24, 0, pi, 4, 2);
            [t, info] = ttc(a, [10 0], b, [-10 0]);
            tc.verifyEqual(t, 1.0, 'AbsTol', tc.SampleTol);
            tc.verifyTrue(info.collides);
            tc.verifyFalse(info.capped);
        end

        function followingSameSpeedNeverCollides(tc)
            % Same direction, same speed: the gap never closes -> capped.
            a = makeOBB(0,  0, 0, 4, 2);
            b = makeOBB(20, 0, 0, 4, 2);
            [t, info] = ttc(a, [10 0], b, [10 0]);
            tc.verifyEqual(t, 10.0);
            tc.verifyFalse(info.collides);
            tc.verifyTrue(info.capped);
        end

        function parallelLanesNeverCollide(tc)
            % Travelling side by side 6 m apart laterally: half-widths are 1
            % each, so they never touch no matter how long they run.
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(0, 6, 0, 4, 2);
            [t, info] = ttc(a, [12 0], b, [12 0]);
            tc.verifyEqual(t, 10.0);
            tc.verifyFalse(info.collides);
            tc.verifyEqual(info.dMin, 4, 'AbsTol', 1e-9);   % 6 - 1 - 1
        end

        function crossingPaths(tc)
            % a runs east at 10 m/s from (0,0); b runs north at 10 m/s from
            % (30,-30).  Both reach the (30,0) area at about t = 3 s.
            % a's front face reaches x=28 (b's near face) at t = (28-2)/10
            % = 2.6 s; b's front reaches y=-1 at t = (29-2)/10 = 2.7 s.
            % First overlap is therefore near 2.7 s.
            a = makeOBB(0,   0,   0,    4, 2);
            b = makeOBB(30, -30,  pi/2, 4, 2);
            [t, info] = ttc(a, [10 0], b, [0 10]);
            tc.verifyTrue(info.collides, 'Crossing paths must register a conflict.');
            tc.verifyGreaterThan(t, 2.4);
            tc.verifyLessThan(t, 3.0);
        end

        function crossingButMissesInTime(tc)
            % Same crossing geometry, but b is 60 m further back, so it
            % arrives long after a has cleared: no conflict.
            a = makeOBB(0,   0,   0,    4, 2);
            b = makeOBB(30, -95,  pi/2, 4, 2);
            [t, info] = ttc(a, [10 0], b, [0 10]);
            tc.verifyFalse(info.collides);
            tc.verifyEqual(t, 10.0);
        end

        function alreadyOverlappingIsZero(tc)
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(1, 0, 0, 4, 2);
            [t, info] = ttc(a, [0 0], b, [0 0]);
            tc.verifyEqual(t, 0, 'AbsTol', 1e-12);
            tc.verifyTrue(info.collides);
        end

        function bothStationaryAndApart(tc)
            a = makeOBB(0,  0, 0, 4, 2);
            b = makeOBB(20, 0, 0, 4, 2);
            [t, info] = ttc(a, [0 0], b, [0 0]);
            tc.verifyEqual(t, 10.0);
            tc.verifyFalse(info.collides);
            tc.verifyEqual(info.dMin, 16, 'AbsTol', 1e-9);
        end

        function separatingNeverCollides(tc)
            % Moving apart: the quadratic window lies in the past.
            a = makeOBB(0,  0, 0, 4, 2);
            b = makeOBB(20, 0, 0, 4, 2);
            [t, info] = ttc(a, [-5 0], b, [5 0]);
            tc.verifyEqual(t, 10.0);
            tc.verifyFalse(info.collides);
        end

        function capIsRespected(tc)
            % Closing so slowly that contact happens after the cap.
            % Gap 16 m, closing 1 m/s -> 16 s, beyond a 5 s cap.
            a = makeOBB(0,  0, 0, 4, 2);
            b = makeOBB(20, 0, 0, 4, 2);
            [t, info] = ttc(a, [1 0], b, [0 0], 5.0);
            tc.verifyEqual(t, 5.0);
            tc.verifyTrue(info.capped);
            tc.verifyFalse(info.collides);
        end

        function pedestrianStepsOut(tc)
            % Ego 4.5 x 1.8 at 8 m/s; pedestrian 0.6 x 0.6 crossing at 1.4 m/s
            % from 4 m to the right, 20 m ahead.  Ego covers 20 m in 2.5 s;
            % the pedestrian needs about (4 - 0.9 - 0.3)/1.4 = 2.0 s to reach
            % the ego's side, so a conflict exists.
            ego = makeOBB(0,  0,  0, 4.5, 1.8);
            ped = makeOBB(20, -4, pi/2, 0.6, 0.6);
            [t, info] = ttc(ego, [8 0], ped, [0 1.4]);
            tc.verifyTrue(info.collides, 'A pedestrian stepping out must produce a finite TTC.');
            tc.verifyLessThan(t, 3.0);
            tc.verifyGreaterThan(t, 1.5);
        end
    end
end
