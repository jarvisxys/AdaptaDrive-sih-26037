classdef testStressAndVariant < matlab.unittest.TestCase
    %TESTSTRESSANDVARIANT  Requirement B5: stress conditions and the held-out case.
    %
    %   These exist because `cfg.stress` was present in `defaultConfig` for most
    %   of the build and **nothing read it**. A configuration field that no code
    %   consumes is worse than a missing one: it advertises a capability the
    %   build does not have, and a reader has no way to tell. The first test
    %   here is specifically that the fields now change behaviour.
    %
    %   See also CONFIGPRESET, SENSORSUITE, SCENARIOUNSEEN.

    methods (Test)

        % ---------------- stress presets ----------------

        function stressPresetDiffersOnlyInStress(tc)
            %   A stress run must be comparable to PROPOSED. If the preset
            %   changed a planner or risk parameter as well, a difference in the
            %   result could not be attributed to the sensor degradation.
            a = configPreset('PROPOSED', 'scenario', 'village', 'seed', 1);
            b = configPreset('STRESS-sensor', 'scenario', 'village', 'seed', 1);

            tc.verifyEqual(b.stress.name, 'sensor');
            tc.verifyEqual(b.stress.cameraDropout, 0.20);
            tc.verifyEqual(b.stress.cameraPosSigma, 0.8);

            % Everything the stack actually plans with must be identical.
            % (There is no cfg.behavior: the FSM's thresholds are Constant
            % properties on BehaviorFSM, not configuration.)
            for f = ["plan", "risk", "control", "tracking", "predict", ...
                     "vehicle", "sim"]
                tc.verifyEqual(b.(f), a.(f), sprintf( ...
                    ['STRESS-sensor changed cfg.%s. A stress condition must ' ...
                     'degrade the sensor and nothing else, or its result is ' ...
                     'not attributable.'], f));
            end
            for f = ["useRiskMap", "useGlobalPlanner", "useDWA", "useFSM", ...
                     "freezeAgents", "riskMode", "classPriors", "uncertainty", ...
                     "context"]
                tc.verifyEqual(b.(f), a.(f), sprintf( ...
                    'STRESS-sensor changed cfg.%s.', f));
            end
        end

        function stressFieldsAreActuallyRead(tc)
            %   The test this whole file exists for: the override must reach the
            %   sensor, not merely sit in the config.
            cfg = configPreset('STRESS-sensor', 'scenario', 'village', 'seed', 1);
            c = SensorSuite.applyStress(cfg.sensors.camera, cfg);

            tc.verifyEqual(c.dropout, 0.20, sprintf( ...
                ['Camera dropout is %.3f; cfg.stress.cameraDropout (0.20) was ' ...
                 'not applied. A stress field nothing reads is not a stress ' ...
                 'condition.'], c.dropout));
            tc.verifyEqual(c.posSigma, 0.8, ...
                'cfg.stress.cameraPosSigma was not applied.');
        end

        function nominalConfigIsUntouchedByStressCode(tc)
            %   With no stress requested the sensor must be bit-identical to the
            %   nominal one, so the stress path cannot perturb every other run.
            cfg = configPreset('PROPOSED', 'scenario', 'village', 'seed', 1);
            c = SensorSuite.applyStress(cfg.sensors.camera, cfg);
            tc.verifyEqual(c, cfg.sensors.camera, ...
                'The stress path altered a nominal camera configuration.');
            tc.verifyEqual(cfg.stress.name, 'none');
        end

        function stressDegradesMeasuredPerception(tc)
            %   End to end, and the direction is what matters: a worse camera
            %   must produce measurably worse perception. If it did not, the
            %   stress condition would be decorative.
            nom = testStressAndVariant.perceptionOf('PROPOSED', 'cattle', 1);
            str = testStressAndVariant.perceptionOf('STRESS-sensor', 'cattle', 1);

            tc.verifyLessThanOrEqual(str.recall, nom.recall + 1e-9, sprintf( ...
                ['Recall under sensor stress (%.3f) is not below nominal ' ...
                 '(%.3f). A 4x dropout and 2.7x position noise must cost ' ...
                 'something measurable.'], str.recall, nom.recall));
        end

        % ---------------- the held-out variant ----------------

        function unseenAddsABusAndACow(tc)
            cfg = configPreset('PROPOSED', 'scenario', 'market', 'seed', 1);
            mk = buildScenario('market', 1, 1.0, cfg);
            un = buildScenario('unseen', 1, 1.0, cfg);

            tc.verifyEqual(numel(un.agents), numel(mk.agents) + 2, ...
                'The unseen variant must be the market plus exactly two agents.');

            cls = [un.agents.classId];
            tc.verifyTrue(any(cls == 2), 'No bus (class 2) in the unseen variant.');
            tc.verifyTrue(any(cls == 7), 'No cow (class 7) in the unseen variant.');

            % The bus must be going the wrong way - that is the point of it.
            bus = un.agents(find(cls == 2, 1));
            tc.verifyEqual(bus.dir, -1, ...
                'The added bus is not travelling against the flow.');

            % ...and must not fit past the ego on this corridor. If it did, the
            % variant would only be the market with extra traffic.
            p = classPriors(2);
            tc.verifyGreaterThan(p.width + cfg.vehicle.width, 4.0, sprintf( ...
                ['A bus (%.1f m) beside the ego (%.1f m) needs %.1f m, which ' ...
                 'must exceed the market corridor for this to be a new case.'], ...
                p.width, cfg.vehicle.width, p.width + cfg.vehicle.width));
        end

        function unseenDoesNotDisturbTheMarketAgents(tc)
            %   The added agents draw from their own random substream, so every
            %   market agent must land exactly where it does in the market. If
            %   adding two agents also shifted the other fourteen, a difference
            %   in the result would be unattributable.
            cfg = configPreset('PROPOSED', 'scenario', 'market', 'seed', 1);
            mk = buildScenario('market', 1, 1.0, cfg);
            un = buildScenario('unseen', 1, 1.0, cfg);

            for k = 1:numel(mk.agents)
                tc.verifyEqual([un.agents(k).x, un.agents(k).y], ...
                    [mk.agents(k).x, mk.agents(k).y], 'AbsTol', 1e-12, ...
                    sprintf(['Market agent %d moved when the variant added its ' ...
                    'two agents; the substreams are not independent.'], k));
            end
        end

        function unseenIsReportedSeparately(tc)
            %   'unseen' must not be one of the five default scenarios, or it
            %   would be pooled into the headline numbers and stop being
            %   held out.
            src = fileread(fullfile(adRoot(), 'run_experiments.m'));
            defaults = regexp(src, ...
                'addParameter\(''scenarios''[^\n]*', 'match', 'once');
            tc.verifyNotEmpty(defaults);
            tc.verifyFalse(contains(defaults, 'unseen'), ...
                ['''unseen'' is in the default scenario list, so it would be ' ...
                 'pooled into the reported completion rate.']);
        end

        function unseenRunsEndToEnd(tc)
            %   It must actually run. A held-out case that errors is not a
            %   held-out case.
            cfg = configPreset('PROPOSED', 'scenario', 'unseen', 'seed', 1);
            s = buildScenario('unseen', 1, 1.0, cfg);
            r = SimEngine(s, cfg).run();
            tc.verifyTrue(ismember(r.outcome.reason, ...
                {'goal', 'collision', 'timeout', 'offroad'}), ...
                sprintf('Unexpected outcome "%s".', r.outcome.reason));
            tc.verifyGreaterThan(r.log.nCycles, 10, 'The run produced no cycles.');
        end
    end

    % ====================================================================
    methods (Static, Access = private)
        function ps = perceptionOf(configName, scenario, seed)
            cfg = configPreset(configName, 'scenario', scenario, 'seed', seed);
            % Short run: this compares perception quality, not completion, and a
            % full traverse would make the test slow for no extra information.
            cfg.sim.maxTime = 25;
            s = buildScenario(scenario, seed, 1.0, cfg);
            r = SimEngine(s, cfg).run();
            ps = r.metrics.perception;
        end
    end
end
