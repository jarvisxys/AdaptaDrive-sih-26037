classdef testTracking < matlab.unittest.TestCase
    %TESTTRACKING  Perception contract, fusion and the fragmentation guard.
    %
    %   The heavyweight test here is trackerDoesNotFragment: it pins down the
    %   defect that cost M2 the most time.  A camera detection has to declare
    %   velocity "unobserved" with a huge variance, initcvekf seeds a new
    %   track's velocity covariance from it, and the resulting position
    %   uncertainty makes association meaningless.  Nothing errors - recall
    %   silently halves.  A metric-level test is the only thing that catches it.

    properties
        cfg
        scenario
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig('name', 'perception', 'scenario', 'village', 'seed', 1);
            tc.scenario = buildScenario('village', 1, 1.0, tc.cfg);
        end
    end

    methods (Test)

        % ---------------- detection contract ----------------

        function detectionsHonourTheContract(tc)
            suite = SensorSuite(tc.cfg, tc.scenario);
            st = struct('x', tc.scenario.ego.x, 'y', tc.scenario.ego.y, ...
                'yaw', tc.scenario.ego.yaw, 'v', 7, 'a', 0, 'steer', 0, 't', 0.1);

            [dets, raw] = suite.step(0.1, st, tc.scenario);

            for f = ["time", "sensor", "pos", "vel", "hasVel", "classId", ...
                     "noiseCov", "targetId"]
                tc.verifyTrue(isfield(dets, f) || isempty(dets), ...
                    sprintf('detection contract is missing "%s"', f));
            end
            for k = 1:numel(dets)
                tc.verifyEqual(numel(dets(k).pos), 2);
                tc.verifyEqual(size(dets(k).noiseCov), [2 2]);
                tc.verifyTrue(all(isfinite(dets(k).pos)));
                tc.verifyTrue(dets(k).classId >= 0 && dets(k).classId <= 7);
                tc.verifyTrue(ismember(dets(k).sensor, ...
                    {'camera', 'radarLong', 'radarShort'}));
                % Only the camera may report a class.
                if ~strcmp(dets(k).sensor, 'camera')
                    tc.verifyEqual(dets(k).classId, 0, ...
                        'Only the camera is allowed to report a class.');
                end
            end
            tc.verifyEqual(numel(raw), numel(dets), ...
                'Every detection must have a tracker-facing counterpart.');
        end

        function detectionsAreInWorldFrame(tc)
            % A detection must land near the object that produced it, in WORLD
            % coordinates.  If the ego-to-world transform were dropped, errors
            % would be of order the ego's distance from the origin.
            suite = SensorSuite(tc.cfg, tc.scenario);
            st = struct('x', tc.scenario.ego.x, 'y', tc.scenario.ego.y, ...
                'yaw', tc.scenario.ego.yaw, 'v', 7, 'a', 0, 'steer', 0, 't', 0.1);

            dets = suite.step(0.1, st, tc.scenario);
            truths = arrayfun(@(a) a.truth(), tc.scenario.agents);

            checked = 0;
            for k = 1:numel(dets)
                d = dets(k);
                if isnan(d.targetId) || ~isKey(tc.scenario.actorIdToAgentIdx, d.targetId)
                    continue
                end
                gt = truths(tc.scenario.actorIdToAgentIdx(d.targetId));
                err = hypot(d.pos(1) - gt.x, d.pos(2) - gt.y);
                tc.verifyLessThan(err, 8, sprintf( ...
                    '%s detection is %.1f m from its target: frame error?', d.sensor, err));
                checked = checked + 1;
            end
            tc.assumeGreaterThan(checked, 0, 'No associated detections to check.');
        end

        % ---------------- class model ----------------

        function cameraClassModelIsNoisyButMostlyRight(tc)
            % The classifier must neither be perfect (that would be ground
            % truth wearing a disguise) nor useless.
            rs = RandStream('mrg32k3a', 'Seed', 5);
            n = 4000;
            for trueClass = 1:7
                out = zeros(1, n);
                for i = 1:n
                    out(i) = cameraClassModel(trueClass, rs);
                end
                acc = mean(out == trueClass);
                unk = mean(out == 0);
                tc.verifyGreaterThan(acc, 0.5, sprintf( ...
                    'class %d accuracy %.2f is too low to be useful', trueClass, acc));
                tc.verifyLessThan(acc, 0.999, sprintf( ...
                    'class %d is never wrong: that is ground truth, not a classifier', trueClass));
                tc.verifyGreaterThan(unk, 0, sprintf( ...
                    'class %d never abstains', trueClass));
                tc.verifyTrue(all(out >= 0 & out <= 7));
            end
        end

        function pedestrianIsNeverConfusedWithABus(tc)
            % Confusion must follow silhouette, not be uniform noise: a
            % pedestrian read as a bus would invert the risk weighting.
            rs = RandStream('mrg32k3a', 'Seed', 9);
            out = arrayfun(@(~) cameraClassModel(5, rs), 1:5000);
            tc.verifyFalse(any(out == 2), 'pedestrian must never be reported as a bus');
            tc.verifyFalse(any(out == 1), 'pedestrian must never be reported as a car');
        end

        % ---------------- tracker ----------------

        function trackContractIsHonoured(tc)
            r = tc.runPerception();
            found = false;
            for c = 1:r.log.nCycles
                tr = r.log.cycles(c).tracks;
                for j = 1:numel(tr)
                    found = true;
                    for f = ["trackId", "classId", "x", "y", "vx", "vy", ...
                             "P", "age", "heading"]
                        tc.verifyTrue(isfield(tr(j), f), ...
                            sprintf('track contract is missing "%s"', f));
                    end
                    tc.verifyEqual(size(tr(j).P), [2 2]);
                    tc.verifyTrue(tr(j).classId >= 0 && tr(j).classId <= 7);
                    tc.verifyGreaterThanOrEqual(tr(j).age, 0);
                end
            end
            tc.verifyTrue(found, 'The run produced no tracks at all.');
        end

        function trackerDoesNotFragment(tc)
            % Regression guard for the initcvekf velocity-covariance defect.
            % With it, 64 cycles over 5 agents produced track ids past 160 and
            % recall near 46%.  Fixed, ids stay under ~40 and recall is >85%.
            r = tc.runPerception();
            p = r.metrics.perception;

            ids = [];
            for c = 1:r.log.nCycles
                tr = r.log.cycles(c).tracks;
                if ~isempty(tr)
                    ids = [ids, [tr.trackId]]; %#ok<AGROW>
                end
            end
            tc.assumeNotEmpty(ids, 'No tracks produced.');

            nAgents = numel(tc.scenario.agents);
            tc.verifyLessThan(max(ids), 15 * nAgents, sprintf( ...
                ['Track ids reached %d for %d agents: the tracker is ' ...
                 'discarding and re-creating tracks (fragmentation).'], ...
                max(ids), nAgents));
            tc.verifyGreaterThan(p.recall, 0.85, sprintf( ...
                'Recall %.1f%% is far below the measured baseline of ~93%%.', ...
                100 * p.recall));
        end

        function trackAccuracyIsReasonable(tc)
            r = tc.runPerception();
            p = r.metrics.perception;
            tc.verifyLessThan(p.posRMSE, 2.5, ...
                'Track position RMSE regressed well past the measured 1.21 m.');
            tc.verifyLessThan(p.velRMSE, 3.0, ...
                'Track velocity RMSE regressed well past the measured 1.58 m/s.');
            tc.verifyLessThan(p.falseTrackRate, 0.15, ...
                'False-track rate regressed past the measured 0%.');
        end

        function classVotingBeatsSingleFrameClassification(tc)
            % Majority voting over a track's history must be at least as good
            % as the per-detection classifier it is built from.
            r = tc.runPerception();
            p = r.metrics.perception;
            tc.verifyGreaterThan(p.classAccuracy, 0.85, sprintf( ...
                'Track class accuracy %.1f%% is below the per-frame model.', ...
                100 * p.classAccuracy));
        end

        function unknownClassIsSupportedNotAnError(tc)
            % classId 0 must be a legal, handled value everywhere.
            p = classPriors(0);
            tc.verifyEqual(p.id, 0);
            tc.verifyEqual(p.name, 'unknown');
            tc.verifyEqual(p.riskWeight, 1.00, ...
                'Unknown must carry the most cautious risk weight.');
            T = classPriors();
            tc.verifyGreaterThanOrEqual(p.qLat, max([T.qLat]), ...
                'Unknown must have the widest lateral uncertainty growth.');
        end

        % ---------------- fallback tracker ----------------

        function fallbackTrackerRunsAndTracks(tc)
            % SimpleKFTracker is a real fallback, not an untested branch.
            cfgF = tc.cfg;
            cfgF.env.backends.tracker = 'simpleKF';
            sc = buildScenario('village', 1, 1.0, cfgF);
            r = SimEngine(sc, cfgF).run();

            total = 0;
            for c = 1:r.log.nCycles
                total = total + numel(r.log.cycles(c).tracks);
            end
            tc.verifyGreaterThan(total, 0, ...
                'The pure-MATLAB fallback tracker produced no tracks at all.');
            tc.verifyGreaterThan(r.metrics.perception.recall, 0.3, ...
                'Fallback tracker recall is implausibly low.');
        end
    end

    methods (Access = private)
        function r = runPerception(tc)
            persistent cached cachedSeed
            if isempty(cached) || ~isequal(cachedSeed, tc.cfg.seed)
                sc = buildScenario('village', tc.cfg.seed, 1.0, tc.cfg);
                cached = SimEngine(sc, tc.cfg).run();
                cachedSeed = tc.cfg.seed;
            end
            r = cached;
        end
    end
end
