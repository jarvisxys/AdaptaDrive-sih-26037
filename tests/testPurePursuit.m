classdef testPurePursuit < matlab.unittest.TestCase
    %TESTPUREPURSUIT  Lateral controller: the law, and backend parity.
    %
    %   The parity tests exist because the two backends silently disagreed
    %   once: controllerPurePursuit returns CURVATURE, not angular velocity,
    %   and converting it as though it were omega under-steered by a factor
    %   equal to the speed.  Nothing failed loudly - the ego just drifted off
    %   a 6 m road.  These tests turn that class of mistake into a red test.

    properties
        cfg
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig();
        end
    end

    methods (Test)

        % ---------------- the law ----------------

        function lookaheadSchedule(tc)
            pp = PurePursuit(tc.cfg, 'native');
            c = tc.cfg.control;
            % Ld = clamp(1.5 + 0.6 v, 3, 15)
            tc.verifyEqual(pp.lookahead(0),  c.lookaheadMin, 'AbsTol', 1e-12);
            tc.verifyEqual(pp.lookahead(10), 1.5 + 0.6*10,   'AbsTol', 1e-12);
            tc.verifyEqual(pp.lookahead(50), c.lookaheadMax, 'AbsTol', 1e-12);
        end

        function straightPathGivesZeroSteer(tc)
            pp = PurePursuit(tc.cfg, 'native');
            path = [(0:1:60)', zeros(61, 1)];
            st = struct('x', 0, 'y', 0, 'yaw', 0, 'v', 8, 'a', 0, 'steer', 0, 't', 0);
            tc.verifyEqual(pp.step(st, path), 0, 'AbsTol', 1e-9);
        end

        function offsetLeftSteersLeft(tc)
            % Vehicle 1 m to the RIGHT of a straight path must steer LEFT
            % (positive steer -> positive yaw rate in the bicycle model).
            pp = PurePursuit(tc.cfg, 'native');
            path = [(0:1:60)', zeros(61, 1)];
            st = struct('x', 0, 'y', -1, 'yaw', 0, 'v', 8, 'a', 0, 'steer', 0, 't', 0);
            tc.verifyGreaterThan(pp.step(st, path), 0);
        end

        function offsetRightSteersRight(tc)
            pp = PurePursuit(tc.cfg, 'native');
            path = [(0:1:60)', zeros(61, 1)];
            st = struct('x', 0, 'y', 1, 'yaw', 0, 'v', 8, 'a', 0, 'steer', 0, 't', 0);
            tc.verifyLessThan(pp.step(st, path), 0);
        end

        function steerMatchesGeometryOnACircle(tc)
            % On a circle of radius R the required steer is atan(L/R).
            % Pure pursuit must land close to that from a point on the circle.
            R = 40;
            th = linspace(0, pi/2, 400)';
            path = [R*sin(th), R - R*cos(th)];
            pp = PurePursuit(tc.cfg, 'native');
            st = struct('x', 0, 'y', 0, 'yaw', 0, 'v', 8, 'a', 0, 'steer', 0, 't', 0);
            expected = atan(tc.cfg.vehicle.wheelbase / R);
            tc.verifyEqual(pp.step(st, path), expected, 'RelTol', 0.15);
        end

        % ---------------- toolbox semantics ----------------

        function toolboxSecondOutputIsCurvatureNotOmega(tc)
            % Pins the R2026a behaviour this project depends on.  If a future
            % release changes the second output back to an angular velocity,
            % this test fails and PurePursuit must be revisited.
            tc.assumeTrue(exist('controllerPurePursuit', 'file') > 0, ...
                'controllerPurePursuit not installed.');

            R = 50;
            th = linspace(0, pi/2, 200)';
            wp = [R*sin(th), R - R*cos(th)];

            out = zeros(1, 4);
            speeds = [1 2 10 20];
            for k = 1:numel(speeds)
                pp = controllerPurePursuit;
                pp.Waypoints = wp;
                pp.LookaheadDistance = 5;
                pp.DesiredLinearVelocity = speeds(k);
                [~, out(k)] = pp([0; 0; 0]);
            end

            % Invariant under speed => it is curvature, not omega.
            tc.verifyEqual(max(out) - min(out), 0, 'AbsTol', 1e-9, ...
                'Second output changed with speed: it is an angular velocity, not curvature.');
            tc.verifyEqual(out(1), 1/R, 'RelTol', 0.02, ...
                'Second output does not equal 1/R on a circle of radius R.');
        end

        % ---------------- backend parity ----------------

        function backendsAgreeOnCurvedPath(tc)
            tc.assumeTrue(exist('controllerPurePursuit', 'file') > 0, ...
                'controllerPurePursuit not installed.');

            R = 60;
            th = linspace(0, pi/2, 400)';
            path = [R*sin(th), R - R*cos(th)];

            ppN = PurePursuit(tc.cfg, 'native');
            ppT = PurePursuit(tc.cfg, 'toolbox');

            for v = [3 8 14]
                for lat = [-1 0 1]
                    st = struct('x', 0, 'y', lat, 'yaw', 0, 'v', v, ...
                        'a', 0, 'steer', 0, 't', 0);
                    sN = ppN.step(st, path);
                    sT = ppT.step(st, path);
                    tc.verifyEqual(sT, sN, 'AbsTol', 0.02, sprintf( ...
                        'Backends disagree at v=%.1f lat=%.1f: native %.5f vs toolbox %.5f', ...
                        v, lat, sN, sT));
                end
            end
        end

        function backendSteerIsSpeedInvariantOnAFixedPath(tc)
            % Pure pursuit curvature depends on the lookahead distance, which
            % depends on speed - but at a FIXED lookahead the geometry cannot
            % depend on speed.  This is the property the omega bug violated.
            tc.assumeTrue(exist('controllerPurePursuit', 'file') > 0, ...
                'controllerPurePursuit not installed.');

            cfgFixed = tc.cfg;
            cfgFixed.control.lookaheadBase = 6;
            cfgFixed.control.lookaheadGain = 0;    % Ld pinned at 6 m
            cfgFixed.control.lookaheadMin = 6;
            cfgFixed.control.lookaheadMax = 6;

            R = 60;
            th = linspace(0, pi/2, 400)';
            path = [R*sin(th), R - R*cos(th)];

            for backend = ["native", "toolbox"]
                pp = PurePursuit(cfgFixed, char(backend));
                s1 = pp.step(struct('x',0,'y',0,'yaw',0,'v',3,'a',0,'steer',0,'t',0), path);
                s2 = pp.step(struct('x',0,'y',0,'yaw',0,'v',15,'a',0,'steer',0,'t',0), path);
                tc.verifyEqual(s2, s1, 'AbsTol', 1e-6, sprintf( ...
                    '%s backend steer changed with speed at fixed lookahead (%.5f vs %.5f)', ...
                    backend, s1, s2));
            end
        end

        % ---------------- closed loop ----------------

        function tracksACurvedRoadWithoutLeavingIt(tc)
            % End-to-end guard: drive the village reference path open loop and
            % require the lateral error to stay small.  This is the test that
            % would have caught the curvature bug at once.
            cfgL = defaultConfig('name', 'scripted', 'scenario', 'village', 'seed', 1);
            sc = buildScenario('village', 1, 1.0, cfgL);

            for backend = ["native", "toolbox"]
                if strcmp(backend, "toolbox") && exist('controllerPurePursuit', 'file') == 0
                    continue
                end
                veh = KinematicBicycle(cfgL, sc.ego);
                pp  = PurePursuit(cfgL, char(backend));
                spd = SpeedController(cfgL);
                off = cfgL.vehicle.length/2 - cfgL.vehicle.rearOverhang;

                maxErr = 0;
                reached = false;
                for k = 0:1500
                    st = veh.state();
                    cx = st.x + off*cos(st.yaw);
                    cy = st.y + off*sin(st.yaw);
                    tc.verifyTrue(sc.road.isDrivable(cx, cy), sprintf( ...
                        '%s backend left the road at t=%.2f s', backend, st.t));
                    ni = sc.road.nearest(cx, cy);
                    maxErr = max(maxErr, abs(ni.d - sc.ego.laneOffset));
                    if hypot(sc.ego.goal(1)-st.x, sc.ego.goal(2)-st.y) < sc.ego.goalTol
                        reached = true;
                        break
                    end
                    steer = pp.step(st, sc.refPath);
                    a = spd.step(cfgL.sim.dt, st.v, 25/3.6, false);
                    veh.step(cfgL.sim.dt, a, steer);
                end

                tc.verifyTrue(reached, sprintf('%s backend never reached the goal', backend));
                tc.verifyLessThan(maxErr, 0.6, sprintf( ...
                    '%s backend tracking error %.3f m is too large', backend, maxErr));
            end
        end
    end
end
