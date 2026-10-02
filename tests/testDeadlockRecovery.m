classdef testDeadlockRecovery < matlab.unittest.TestCase
    %TESTDEADLOCKRECOVERY  The four rules that stopped the vehicle deadlocking.
    %
    %   Every test here is a regression test for a failure that was MEASURED in
    %   a run, and each names the run it came from.  They are grouped because
    %   they share one shape: the vehicle stops somewhere it could have driven,
    %   and nothing in the loop can ever change its mind.
    %
    %     departedAgentsLeavePerception   highway stalled at 384 m of 392
    %     duplicateTracksAreMerged        village yielded to 6 copies of a cart
    %     antiStallBreaksACostDeadlock    village froze 30 m from the goal
    %     escapeLeavesAViolatingCell      village sat in STOP for 122 s
    %
    %   See also TESTDWA, TESTTRACKING.

    methods (Test)

        % ------------------------------------------------------------------
        function departedAgentsLeavePerception(tc)
            %   An agent that runs off the end of the modelled stretch used to
            %   stay in the drivingScenario container at its last pose, where
            %   the sensors went on detecting it.  Four highway agents piled up
            %   within a couple of metres of each other past the end of the
            %   ribbon and became a wall of ~30 phantom tracks 16 m in front of
            %   the ego, which stopped at 384 m of 392 and never resumed.
            %
            %   The assertion is on the TRUTH the sensors are shown, not on the
            %   track list, because that is where the defect was: every other
            %   consumer already skipped inactive agents.
            cfg = configPreset('PROPOSED', 'scenario', 'highway', 'seed', 1);
            s = buildScenario('highway', 1, 1.0, cfg);
            r = SimEngine(s, cfg).run();
            L = r.log;

            % Find a step where at least one agent has left the stretch.
            kLate = [];
            for k = round(L.n/2):L.n
                if any(~[L.agents{k}.active])
                    kLate = k; break
                end
            end
            tc.assumeNotEmpty(kLate, ...
                'No agent left the stretch in this run; nothing to check.');

            A = L.agents{kLate};
            gone = A(~[A.active]);
            tc.verifyNotEmpty(gone);

            % A thing that is not moving must not report a speed.  Inactive
            % agents used to freeze mid-stride and keep claiming 7-16 m/s, which
            % is what stopped the tracker's filters converging on them.
            for i = 1:numel(gone)
                tc.verifyLessThan(hypot(gone(i).vx, gone(i).vy), 1e-9, ...
                    sprintf(['Inactive agent %d reports %.2f m/s. An agent ' ...
                    'that has left the stretch is not moving.'], ...
                    gone(i).id, hypot(gone(i).vx, gone(i).vy)));
            end
        end

        function noTrackClusterOutsideTheStretch(tc)
            %   The observable consequence of the fix: the tracker must not hold
            %   a crowd of tracks beyond the end of the route.  Before the fix
            %   this run carried 29.6 tracks on average for 5 agents, 90% of
            %   them false, all piled past the 392 m mark.
            cfg = configPreset('PROPOSED', 'scenario', 'highway', 'seed', 1);
            s = buildScenario('highway', 1, 1.0, cfg);
            r = SimEngine(s, cfg).run();

            nTruth = numel(s.agents);
            meanTracks = r.metrics.perception.meanTracks;

            % Some duplication is expected from four sensors; a six-fold
            % over-count is the defect.  3x the agent population is a generous
            % ceiling that the phantom wall (5.9x) clears and a healthy run
            % (2.8x measured after the fix) stays under.
            tc.verifyLessThan(meanTracks, 3 * nTruth, sprintf( ...
                ['Tracker holds %.1f tracks on average for %d agents. ' ...
                 'A cluster of phantom tracks past the end of the route is ' ...
                 'the highway-stall defect.'], meanTracks, nTruth));
        end

        % ------------------------------------------------------------------
        function duplicateTracksAreMerged(tc)
            %   Six confirmed tracks inside a 2 m circle, all on one pushcart,
            %   were each predicted forward and each painted into the risk map.
            %   The combined predicted occupancy made a 6 m road impassable and
            %   the ego yielded to a crowd that was not there.
            cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'village', 'seed', 1);
            tw = TrackerWrapper(cfg);

            % Six tracks on one object, within the merge radius.
            base = [225.4, 12.7];
            tr = TrackerWrapper.emptyTracks();
            for i = 1:6
                tr(end+1) = testDeadlockRecovery.aTrack(200 + i, 6, ...
                    base(1) + 0.3*(i-3.5)/6, base(2) + 0.2*(i-3.5)/6, ...
                    0.9, 0.0, 0.4, 10 + i); %#ok<AGROW>
            end

            merged = testDeadlockRecovery.callMerge(tw, tr);
            tc.verifyEqual(numel(merged), 1, sprintf( ...
                'Six tracks on one object merged to %d, expected 1.', ...
                numel(merged)));

            % The survivor must carry the OLDEST id, so anything downstream
            % keyed on trackId stays attached to the same object.
            tc.verifyEqual(merged(1).trackId, 201);

            % ...and it must sit on the object, not somewhere between copies.
            tc.verifyLessThan(hypot(merged(1).x - base(1), merged(1).y - base(2)), ...
                0.5, 'Merged track is not on the object.');

            % ...and it must not claim more certainty than its inputs had.
            tc.verifyGreaterThanOrEqual(trace(merged(1).P(1:2,1:2)), 0, ...
                'Merged covariance is not positive.');
        end

        function distinctVehiclesAreNeverMerged(tc)
            %   The opposite error, and the worse one.  Merging two real
            %   vehicles invents an obstacle between them and loses both: an
            %   earlier version scaled the merge radius with the class footprint
            %   (1.8 m for two cars), fused tracks belonging to different
            %   highway vehicles, and took position recall to 0.48 while the
            %   track COUNT still looked healthy.
            cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'highway', 'seed', 1);
            tw = TrackerWrapper(cfg);

            for gap = [1.5, 2.0, 3.0, 4.5]
                tr = TrackerWrapper.emptyTracks();
                tr(end+1) = testDeadlockRecovery.aTrack(1, 5, 100.0, 3.0, ...
                    12.0, 0, 0.3, 40); %#ok<AGROW>
                tr(end+1) = testDeadlockRecovery.aTrack(2, 5, 100.0 + gap, 3.0, ...
                    12.0, 0, 0.3, 40); %#ok<AGROW>

                merged = testDeadlockRecovery.callMerge(tw, tr);
                tc.verifyEqual(numel(merged), 2, sprintf( ...
                    ['Two distinct vehicles %.1f m apart were merged into %d ' ...
                     'track(s). Merging real objects is worse than keeping a ' ...
                     'duplicate: it invents one obstacle and loses two.'], ...
                    gap, numel(merged)));
            end
        end

        % ------------------------------------------------------------------
        function antiStallBreaksACostDeadlock(tc)
            %   Standing still occupies one cell; if that cell is clear its risk
            %   term is exactly 0, while any moving rollout scores above 0.  So
            %   for any positive risk weight there is a risk level at which
            %   paralysis is the cheapest option - and since the ego then does
            %   not move, the same comparison holds forever.
            %
            %   Measured on village seed 1 at t = 100.1 s, 30 m from the goal:
            %   v = 0 cost 0.3668, cheapest moving candidate 0.3853.  The ego
            %   held that spot until the run timed out.
            cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'village', 'seed', 1);
            s = buildScenario('village', 1, 0, cfg);      % no traffic
            dwa = DWAPlanner(cfg, s.road);

            st = struct('x', s.ego.x, 'y', s.ego.y, 'yaw', s.ego.yaw, ...
                'v', 0, 'a', 0, 'steer', 0, 't', 0);
            rm = RiskMap(cfg, s.road, s.hazards);
            rm.update(st, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            ctx = contextParams(s.context, cfg);
            decision = struct('speedCap', ctx.speedCap, ...
                'clearanceMargin', ctx.lateralClearance, ...
                'emergency', false, 'stateName', 'CRUISE');

            % Hold the ego stationary and step the planner past stallBreakTime.
            % Whatever the cost function prefers, a planner that is still
            % choosing v = 0 after the budget has expired is deadlocked.
            dt = 1 / cfg.sim.planRate;
            nSteps = ceil((cfg.plan.stallBreakTime + 1.0) / dt);
            moved = false;
            for k = 1:nSteps
                st.t = (k-1) * dt;
                out = dwa.plan(st, rm, s.refPath, Predictor.emptyPredictions(), ...
                    decision, ctx, s.refPath(end, :));
                tc.assertTrue(out.feasible, ...
                    'No feasible candidate on an empty village road.');
                if out.chosenV > DWAPlanner.MovingSpeed
                    moved = true; break
                end
            end
            tc.verifyTrue(moved, sprintf( ...
                ['The planner chose v = 0 for %.1f s while the FSM was asking ' ...
                 'for %.2f m/s on a clear road. Stopping is the behaviour ' ...
                 'layer''s decision, not a side effect of the cost function.'], ...
                nSteps * dt, ctx.speedCap));
        end

        function antiStallRespectsAnFSMStop(tc)
            %   The override must never fire against a real STOP.  The FSM
            %   expresses "stay put" as speedCap = 0, and an anti-stall rule
            %   that ignored that would drive through the decision it exists to
            %   serve.
            cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'village', 'seed', 1);
            s = buildScenario('village', 1, 0, cfg);
            dwa = DWAPlanner(cfg, s.road);

            st = struct('x', s.ego.x, 'y', s.ego.y, 'yaw', s.ego.yaw, ...
                'v', 0, 'a', 0, 'steer', 0, 't', 0);
            rm = RiskMap(cfg, s.road, s.hazards);
            rm.update(st, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            ctx = contextParams(s.context, cfg);
            decision = struct('speedCap', 0, ...
                'clearanceMargin', ctx.lateralClearance, ...
                'emergency', false, 'stateName', 'STOP');

            dt = 1 / cfg.sim.planRate;
            for k = 1:ceil((cfg.plan.stallBreakTime + 2.0) / dt)
                st.t = (k-1) * dt;
                out = dwa.plan(st, rm, s.refPath, Predictor.emptyPredictions(), ...
                    decision, ctx, s.refPath(end, :));
                tc.verifyLessThanOrEqual(out.chosenV, DWAPlanner.MovingSpeed, ...
                    sprintf(['Planner chose %.2f m/s under an FSM STOP ' ...
                    '(speedCap 0) at t = %.1f s.'], out.chosenV, st.t));
            end
        end

        % ------------------------------------------------------------------
        function escapeLeavesAViolatingCell(tc)
            %   When the ego's own footprint is at or above rejectRisk, every
            %   candidate is rejected INCLUDING holding position.  "Everything
            %   was rejected" is then not information about where the ego may
            %   go, it is information about where the ego already is, and
            %   refusing to move preserves the violation rather than ending it.
            %
            %   Measured on village seed 1: the ego sat at (95.0, 1.5) in STOP
            %   with 77 of 77 candidates rejected, from t = 55 s to the timeout
            %   at 177.5 s.
            cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'village', 'seed', 1);
            s = buildScenario('village', 1, 0, cfg);
            dwa = DWAPlanner(cfg, s.road);
            ctx = contextParams(s.context, cfg);

            decision = struct('speedCap', ctx.speedCap, ...
                'clearanceMargin', ctx.lateralClearance, ...
                'emergency', false, 'stateName', 'CRUISE');

            % Walk the ego out sideways until EVERY candidate is rejected.  A
            % fixed offset will not do: at 1.5 m off the line 30 of 77 rollouts
            % are still feasible and the planner correctly just drives out, which
            % is the system working rather than the case under test.  The escape
            % rule only governs the state where nothing at all survives.
            [st, rm, out] = testDeadlockRecovery.findBlockedState( ...
                cfg, s, dwa, ctx, decision);
            tc.assumeNotEmpty(out, ...
                'No fully blocked placement found; nothing to test.');

            tc.verifyTrue(out.blocked, ...
                'A state where every candidate is rejected must report blocked.');
            tc.verifyTrue(out.escape, sprintf( ...
                ['Ego at (%.1f, %.1f) has all %d candidates rejected and no ' ...
                 'escape. Holding position preserves the violation instead of ' ...
                 'ending it.'], st.x, st.y, out.nRejected));
            tc.verifyNotEmpty(out.traj, 'An escape must supply a trajectory.');
            tc.verifyGreaterThan(out.chosenV, DWAPlanner.MovingSpeed, ...
                'An escape that does not move is not an escape.');
            tc.verifyLessThanOrEqual(out.chosenV, cfg.plan.escapeSpeed, ...
                'An escape must be at walking pace, not a commitment to progress.');
        end

        function noEscapeWhenHoldingIsSafe(tc)
            %   The escape rule must not weaken the ordinary rejection rule.  On
            %   a clear road the ego is not in violation, so nothing may be
            %   flagged as an escape however the cost falls.
            cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'cattle', 'seed', 1);
            s = buildScenario('cattle', 1, 0, cfg);
            dwa = DWAPlanner(cfg, s.road);
            ctx = contextParams(s.context, cfg);

            st = struct('x', s.ego.x, 'y', s.ego.y, 'yaw', s.ego.yaw, ...
                'v', 3.0, 'a', 0, 'steer', 0, 't', 5.0);
            rm = RiskMap(cfg, s.road, s.hazards);
            rm.update(st, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            decision = struct('speedCap', ctx.speedCap, ...
                'clearanceMargin', ctx.lateralClearance, ...
                'emergency', false, 'stateName', 'CRUISE');

            out = dwa.plan(st, rm, s.refPath, Predictor.emptyPredictions(), ...
                decision, ctx, s.refPath(end, :));
            tc.verifyFalse(out.escape, ...
                'An escape was flagged on a clear road where holding is safe.');
            tc.verifyFalse(out.blocked, 'A clear road must not report blocked.');
        end
    end

    % ====================================================================
    methods (Static, Access = private)

        function T = aTrack(id, classId, x, y, vx, vy, pvar, updates)
            %ATRACK  A track matching the contract TrackerWrapper returns.
            T = struct('trackId', id, 'classId', classId, 'x', x, 'y', y, ...
                'vx', vx, 'vy', vy, 'speed', hypot(vx, vy), ...
                'heading', atan2(vy, vx), 'P', eye(4) * pvar, ...
                'age', updates, 'updates', updates, 'confirmed', true);
        end

        function out = callMerge(tw, tr)
            out = tw.mergeDuplicates(tr);
        end

        function [st, rm, out] = findBlockedState(cfg, s, dwa, ctx, decision)
            %FINDBLOCKEDSTATE  Nearest placement where every candidate is rejected.
            %
            %   Steps the ego sideways off the reference line, and also turns it
            %   to face away from the road, until the planner reports blocked.
            %   A fixed offset will not do: measured at 1.5 m off the line, the
            %   body centre sits on risk 0.98 but 30 of 77 rollouts are still
            %   feasible and the planner correctly just drives out - which is the
            %   system working, not the case under test.
            %
            %   St carries the REAR-AXLE pose the planner expects; the risk
            %   samples inside the planner are taken at the body centre.
            rp = s.refPath;
            i = max(1, round(0.35 * size(rp, 1)));
            j = min(i + 1, size(rp, 1));
            tang = atan2(rp(j,2) - rp(i,2), rp(j,1) - rp(i,1));
            nrm = [-sin(tang), cos(tang)];
            off = cfg.vehicle.length/2 - cfg.vehicle.rearOverhang;
            rm = RiskMap(cfg, s.road, s.hazards);
            st = []; out = [];

            for d = 1.0:0.5:10
                for sgn = [1 -1]
                    % Face outward as well as sit outward: a vehicle pointing
                    % along the road can usually roll back onto it, which is the
                    % feasible case rather than the blocked one.
                    for dyaw = [0, sgn*pi/3, sgn*pi/2]
                        yaw = tang + dyaw;
                        p = rp(i,:) + sgn * d * nrm;
                        cand = struct('x', p(1) - off*cos(yaw), ...
                            'y', p(2) - off*sin(yaw), 'yaw', yaw, ...
                            'v', 0, 'a', 0, 'steer', 0, 't', 55.0);
                        rm.update(cand, Predictor.emptyPredictions(), ...
                            TrackerWrapper.emptyTracks());
                        o = dwa.plan(cand, rm, s.refPath, ...
                            Predictor.emptyPredictions(), decision, ctx, ...
                            s.refPath(end, :));
                        if o.blocked
                            st = cand; out = o;
                            return
                        end
                    end
                end
            end
        end
    end
end
