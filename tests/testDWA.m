classdef testDWA < matlab.unittest.TestCase
    %TESTDWA  Local planner: bounded cost terms and speed selection.
    %
    %   These three tests exist because four separate planner defects shared
    %   one shape - a cost term escaping its expected scale and quietly
    %   overriding every other term - and none of them raised an error.  They
    %   showed up only as a vehicle that drove badly.
    %
    %     termsAreBounded         asserts the property the defects violated
    %     reachesSpeedCapOnClearRoad  the behaviour they destroyed
    %     standingStillIsNeverBest    the specific symptom they produced

    properties
        cfg
        scenario
        ctx
        riskMap
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig('name', 'PROPOSED', 'scenario', 'cattle', 'seed', 1);
            tc.scenario = testDWA.emptyRoadScenario(tc.cfg);
            tc.ctx = contextParams('rural', tc.cfg);

            st = tc.egoAt(3.0);
            tc.riskMap = RiskMap(tc.cfg, tc.scenario.road, tc.scenario.hazards);
            tc.riskMap.update(st, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
        end
    end

    methods (Test)

        % ---------------- (a) every term in [0,1] ----------------

        function weightsArePartitionOfOne(tc)
            % The bound on the terms is only meaningful if the weights are a
            % partition, so check every context.
            T = contextParams();
            for k = 1:numel(T)
                w = T(k).dwaWeights;
                s = w.goal + w.risk + w.clear + w.smooth + w.speed;
                tc.verifyEqual(s, 1, 'AbsTol', 1e-12, sprintf( ...
                    'Context "%s" weights sum to %.6f, not 1.', T(k).name, s));
                for f = ["goal", "risk", "clear", "smooth", "speed"]
                    tc.verifyGreaterThanOrEqual(w.(f), 0);
                end
            end
        end

        function termsAreBounded(tc)
            % Random rollouts over a wide range of states, speeds and
            % curvatures: no term may leave [0,1].
            rs = RandStream('mrg32k3a', 'Seed', 31);
            dwa = DWAPlanner(tc.cfg, tc.scenario.road);
            gp = tc.scenario.refPath;

            nChecked = 0;
            for trial = 1:300
                st = tc.egoAt(rand(rs) * tc.ctx.speedCap);
                st.x = st.x + (rand(rs) - 0.5) * 40;
                st.y = st.y + (rand(rs) - 0.5) * 10;
                st.yaw = st.yaw + (rand(rs) - 0.5) * 1.2;
                st.steer = (rand(rs) - 0.5) * 2 * tc.cfg.vehicle.maxSteer;

                rm = RiskMap(tc.cfg, tc.scenario.road, tc.scenario.hazards);
                preds = tc.randomPredictions(rs, st);
                rm.update(st, preds, tc.tracksFor(preds));

                [vs, kappas, kappa0, dKappaMax] = dwa.dynamicWindow(st, tc.ctx.speedCap);
                obst = tc.flatten(preds);

                for iv = [1, numel(vs)]
                    for ik = [1, ceil(numel(kappas)/2), numel(kappas)]
                        traj = dwa.rollout(st, vs(iv), kappas(ik));
                        off = tc.cfg.vehicle.length/2 - tc.cfg.vehicle.rearOverhang;
                        bx = traj(:,1) + off*cos(traj(:,3));
                        by = traj(:,2) + off*sin(traj(:,3));
                        rv = rm.sampleWorld(bx, by);

                        terms = dwa.costTerms(traj, rv, kappas(ik), kappa0, ...
                            dKappaMax, obst, gp, st, tc.ctx.speedCap, []);

                        for f = ["goal", "risk", "clear", "smooth", "speed"]
                            val = terms.(f);
                            tc.verifyTrue(isfinite(val), sprintf( ...
                                'term "%s" is not finite (%g)', f, val));
                            tc.verifyGreaterThanOrEqual(val, 0, sprintf( ...
                                'term "%s" = %.4f is below 0', f, val));
                            tc.verifyLessThanOrEqual(val, 1, sprintf( ...
                                'term "%s" = %.4f is above 1', f, val));
                        end
                        nChecked = nChecked + 1;
                    end
                end
            end
            tc.verifyGreaterThan(nChecked, 500, 'Too few rollouts were checked.');
        end

        function totalCostIsBounded(tc)
            % With bounded terms and weights summing to 1, the total is in
            % [0,1] too - which is what makes costs comparable across contexts.
            dwa = DWAPlanner(tc.cfg, tc.scenario.road);
            st = tc.egoAt(5);
            terms = struct('goal', 1, 'risk', 1, 'clear', 1, 'smooth', 1, 'speed', 1);
            tc.verifyEqual(dwa.combine(terms, tc.ctx.dwaWeights), 1, 'AbsTol', 1e-12);
            terms = struct('goal', 0, 'risk', 0, 'clear', 0, 'smooth', 0, 'speed', 0);
            tc.verifyEqual(dwa.combine(terms, tc.ctx.dwaWeights), 0, 'AbsTol', 1e-12);
        end

        % ---------------- (c) standing still is never best ----------------

        function standingStillIsNeverBest(tc)
            % On clear road the argmin must never be the zero-speed candidate.
            % This is the exact symptom the unbounded clearance term and the
            % speed-change smoothness term both produced.
            dwa = DWAPlanner(tc.cfg, tc.scenario.road);
            gp = tc.scenario.refPath;

            for v0 = [0, 0.5, 2, 5, 8]
                st = tc.egoAt(v0);
                rm = RiskMap(tc.cfg, tc.scenario.road, tc.scenario.hazards);
                rm.update(st, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());

                decision = struct('speedCap', tc.ctx.speedCap, ...
                    'clearanceMargin', tc.ctx.lateralClearance, ...
                    'emergency', false, 'stateName', 'CRUISE');

                out = dwa.plan(st, rm, gp, Predictor.emptyPredictions(), decision, tc.ctx);

                tc.verifyTrue(out.feasible, sprintf( ...
                    'No feasible candidate on clear road at v0 = %.1f.', v0));
                tc.verifyGreaterThan(out.chosenV, 0, sprintf( ...
                    ['Standing still was the argmin on clear road at v0 = %.1f ' ...
                     '(chose %.3f m/s under a %.2f m/s cap).'], ...
                    v0, out.chosenV, tc.ctx.speedCap));
                tc.verifyGreaterThanOrEqual(out.chosenV, v0 - 1e-9, sprintf( ...
                    'Chose to decelerate on clear road at v0 = %.1f.', v0));
            end
        end

        function acceleratesFromRestOnClearRoad(tc)
            dwa = DWAPlanner(tc.cfg, tc.scenario.road);
            st = tc.egoAt(0);
            rm = RiskMap(tc.cfg, tc.scenario.road, tc.scenario.hazards);
            rm.update(st, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            decision = struct('speedCap', tc.ctx.speedCap, ...
                'clearanceMargin', tc.ctx.lateralClearance, ...
                'emergency', false, 'stateName', 'CRUISE');

            out = dwa.plan(st, rm, tc.scenario.refPath, ...
                Predictor.emptyPredictions(), decision, tc.ctx);
            tc.verifyGreaterThan(out.chosenV, 1.0, ...
                'A stationary vehicle on clear road must choose to accelerate.');
        end

        % ---------------- (b) closed loop reaches the cap ----------------

        function reachesSpeedCapOnClearRoad(tc)
            % End to end: with no agents anywhere, the ego must reach 95% of
            % its cap within 5 s and hold it.
            cap = tc.ctx.speedCap;
            r = SimEngine(tc.scenario, tc.cfg).run();

            t = r.log.t(1:r.log.n);
            v = r.log.ego(1:r.log.n, 4);

            idx5 = find(t >= 5.0, 1);
            tc.assertNotEmpty(idx5, 'Run was shorter than 5 s.');

            vBy5 = max(v(1:idx5));
            tc.verifyGreaterThanOrEqual(vBy5, 0.95 * cap, sprintf( ...
                ['Only reached %.2f m/s within 5 s on an empty road ' ...
                 '(cap %.2f, need %.2f).'], vBy5, cap, 0.95 * cap));

            % ...and holds it: mean speed after 5 s stays high.
            after = v(idx5:end);
            tc.verifyGreaterThanOrEqual(mean(after), 0.85 * cap, sprintf( ...
                ['Mean speed after 5 s was %.2f m/s on an empty road ' ...
                 '(cap %.2f); the vehicle does not hold its speed.'], ...
                mean(after), cap));

            tc.verifyTrue(r.metrics.completed, sprintf( ...
                'Empty-road run did not complete: %s', r.outcome.reason));
        end
    end

    % ====================================================================
    methods (Access = private)

        function st = egoAt(tc, v)
            e = tc.scenario.ego;
            st = struct('x', e.x, 'y', e.y, 'yaw', e.yaw, 'v', v, ...
                'a', 0, 'steer', 0, 't', 1.0);
        end

        function preds = randomPredictions(tc, rs, st)
            preds = Predictor.emptyPredictions();
            n = randi(rs, [0 3]);
            T = round(tc.cfg.predict.horizon / tc.cfg.predict.dt);
            tv = (1:T)' * tc.cfg.predict.dt;
            for i = 1:n
                cls = randi(rs, [1 7]);
                x0 = st.x + 5 + rand(rs) * 40;
                y0 = st.y + (rand(rs) - 0.5) * 8;
                vx = (rand(rs) - 0.5) * 10;
                vy = (rand(rs) - 0.5) * 3;
                mu = [x0 + vx*tv, y0 + vy*tv];
                preds(end+1) = struct('trackId', i, 'classId', cls, ...
                    'modes', struct('name', 'cv', 'prob', 1.0, 'mu', mu, ...
                    'Sigma', repmat(eye(2)*0.5, 1, 1, T), ...
                    'heading', repmat(atan2(vy, vx), T, 1))); %#ok<AGROW>
            end
        end

        function tr = tracksFor(~, preds)
            tr = TrackerWrapper.emptyTracks();
            for i = 1:numel(preds)
                mu = preds(i).modes(1).mu;
                tr(end+1) = struct('trackId', preds(i).trackId, ...
                    'classId', preds(i).classId, 'x', mu(1,1), 'y', mu(1,2), ...
                    'vx', 0, 'vy', 0, 'speed', 0, 'heading', 0, ...
                    'P', eye(2)*0.3, 'age', 1, 'updates', 5, 'confirmed', true); %#ok<AGROW>
            end
        end

        function obst = flatten(~, preds)
            obst = struct('a', {}, 'b', {}, 'radius', {}, 'prob', {});
            for i = 1:numel(preds)
                p = classPriors(preds(i).classId);
                for k = 1:numel(preds(i).modes)
                    m = preds(i).modes(k);
                    h = m.heading; h(isnan(h)) = 0;
                    off = (p.length/2) * [cos(h), sin(h)];
                    obst(end+1) = struct('a', m.mu - off, 'b', m.mu + off, ...
                        'radius', p.width/2, 'prob', m.prob); %#ok<AGROW>
                end
            end
        end
    end

    methods (Static)
        function s = emptyRoadScenario(cfg)
            %EMPTYROADSCENARIO  A real scenario built with no traffic at all.
            %
            %   density = 0, NOT "build it then delete the agents".  Deleting
            %   them from the struct leaves them in the drivingScenario
            %   container, where the sensors keep detecting them frozen at
            %   their spawn positions - an earlier version of this helper did
            %   exactly that and produced 4.25 phantom tracks per cycle on a
            %   road that was supposed to be empty, which then looked like a
            %   planner defect.
            s = buildScenario('cattle', 1, 0, cfg);
        end
    end
end
