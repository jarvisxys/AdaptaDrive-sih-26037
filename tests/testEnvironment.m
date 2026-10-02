classdef testEnvironment < matlab.unittest.TestCase
    %TESTENVIRONMENT  M0 acceptance: the environment probe is self-consistent.
    %
    %   These tests do not assert that any particular toolbox is present -
    %   AdaptaDrive must run without them.  They assert that CHECK_ENV never
    %   CLAIMS a capability it has not actually probed, and that the selected
    %   backends are real, callable implementations.

    methods (Test)

        function envHasRequiredFields(tc)
            env = check_env('print', false);
            tc.verifyClass(env, 'struct');
            for f = ["timestamp", "platform", "toolboxes", "backends", "warnings", "gitCommit"]
                tc.verifyTrue(isfield(env, f), sprintf('check_env is missing field "%s".', f));
            end
            for f = ["matlabRelease", "computer", "numCores", "cpu", "hostname", "ramGB"]
                tc.verifyTrue(isfield(env.platform, f), ...
                    sprintf('check_env().platform is missing field "%s".', f));
            end
            tc.verifyGreaterThan(env.platform.numCores, 0);
            tc.verifyNotEmpty(env.platform.cpu);
            % RAM may legitimately be unmeasurable off Windows, but it must
            % never be silently reported as a plausible-looking number.
            tc.verifyTrue(isnan(env.platform.ramGB) || env.platform.ramGB > 0);
        end

        function usableImpliesCallable(tc)
            % The core honesty check: nothing may be reported USABLE unless its
            % entry point actually resolves on this machine.
            env = check_env('print', false);
            for k = 1:numel(env.toolboxes)
                t = env.toolboxes(k);
                if t.usable
                    tc.verifyTrue(t.installed, ...
                        sprintf('%s reported usable but is not installed.', t.name));
                    tc.verifyTrue(t.licensed, ...
                        sprintf('%s reported usable but is not licensed.', t.name));
                    tc.verifyGreaterThan(exist(t.entryPoint, 'file') + exist(t.entryPoint, 'builtin'), 0, ...
                        sprintf('%s reported usable but entry point "%s" does not resolve.', ...
                        t.name, t.entryPoint));
                end
            end
        end

        function licenceAloneIsNotEvidence(tc)
            % Guards the measured quirk documented in DEVIATIONS.md: a licence
            % can test true for a toolbox that is not installed.  If that ever
            % starts driving "usable", this test fails.
            env = check_env('print', false);
            for k = 1:numel(env.toolboxes)
                t = env.toolboxes(k);
                if t.licensed && ~t.installed
                    tc.verifyFalse(t.usable, sprintf( ...
                        '%s is licensed but not installed; it must not be reported usable.', t.name));
                end
            end
        end

        function licenceProbesNeverThrow(tc)
            % license('test',...) errors on feature names >= 28 characters.
            % check_env must survive that, not crash the whole session.
            tc.verifyWarningFree(@() check_env('force', true, 'print', false));
        end

        function backendsAreKnownValues(tc)
            env = check_env('print', false);
            b = env.backends;
            tc.verifyTrue(ismember(b.scenario,      {'drivingScenario', 'native'}));
            tc.verifyTrue(ismember(b.sensors,       {'toolbox', 'synthetic'}));
            tc.verifyTrue(ismember(b.tracker,       {'multiObjectTracker', 'simpleKF'}));
            tc.verifyTrue(ismember(b.globalPlanner, {'hybridAStar', 'rrtStar', 'gridAStar'}));
            tc.verifyTrue(ismember(b.purePursuit,   {'toolbox', 'native'}));
            tc.verifyTrue(ismember(b.behaviorChart, {'stateflow', 'unavailable'}));
            tc.verifyEqual(b.behaviorRuntime, 'BehaviorFSM', ...
                'The runtime FSM must always be the MATLAB class, never the chart.');
            tc.verifyClass(b.parallel, 'logical');
            tc.verifyFalse(b.lidar, 'Lidar must default to off.');
        end

        function selectedBackendsResolve(tc)
            % Each selected toolbox backend must name something callable.
            env = check_env('print', false);
            b = env.backends;
            mustResolve = {};
            if strcmp(b.scenario, 'drivingScenario'),      mustResolve{end+1} = 'drivingScenario';      end %#ok<AGROW>
            if strcmp(b.tracker, 'multiObjectTracker'),    mustResolve{end+1} = 'multiObjectTracker';   end %#ok<AGROW>
            if strcmp(b.globalPlanner, 'hybridAStar'),     mustResolve{end+1} = 'plannerHybridAStar';   end %#ok<AGROW>
            if strcmp(b.globalPlanner, 'rrtStar'),         mustResolve{end+1} = 'plannerRRTStar';       end %#ok<AGROW>
            if strcmp(b.purePursuit, 'toolbox'),           mustResolve{end+1} = 'controllerPurePursuit';end %#ok<AGROW>
            if strcmp(b.behaviorChart, 'stateflow'),       mustResolve{end+1} = 'sfnew';                end %#ok<AGROW>
            for k = 1:numel(mustResolve)
                tc.verifyGreaterThan(exist(mustResolve{k}, 'file') + exist(mustResolve{k}, 'builtin'), 0, ...
                    sprintf('Backend selected "%s" but it does not resolve.', mustResolve{k}));
            end
        end

        function cacheIsStable(tc)
            a = check_env('print', false);
            b = check_env('print', false);
            tc.verifyEqual(b.backends, a.backends);
            tc.verifyEqual({b.toolboxes.usable}, {a.toolboxes.usable});
        end

        function repoLayoutIsOnPath(tc)
            root = adRoot();
            tc.verifyTrue(isfile(fullfile(root, 'startup.m')));
            tc.verifyTrue(isfile(fullfile(root, 'check_env.m')));
            tc.verifyTrue(isfolder(fullfile(root, 'src')));
            tc.verifyTrue(isfolder(fullfile(root, 'tests')));
        end

        function defaultConfigIsSane(tc)
            cfg = defaultConfig();
            tc.verifyEqual(cfg.sim.dt, 0.05);
            tc.verifyEqual(cfg.riskMode, 'full');
            tc.verifyTrue(cfg.classPriors && cfg.uncertainty && cfg.context);
            tc.verifyFalse(cfg.lidar, 'Lidar must default to off.');
            % Reported seeds and tuning seeds must not overlap (freeze rule).
            tc.verifyEmpty(intersect(cfg.experiment.seeds, cfg.experiment.tuneSeeds), ...
                'Tuning seeds must be disjoint from reported seeds.');
        end

        function configOverridesWork(tc)
            cfg = defaultConfig('riskMode', 'binary', 'vehicle.wheelbase', 3.1);
            tc.verifyEqual(cfg.riskMode, 'binary');
            tc.verifyEqual(cfg.vehicle.wheelbase, 3.1);
        end

        function configRejectsTypos(tc)
            % A silently-created config field would mean an ablation switch that
            % never takes effect, which would corrupt results.
            tc.verifyError(@() defaultConfig('riskMoad', 'binary'), ...
                'defaultConfig:unknownField');
        end
    end
end
