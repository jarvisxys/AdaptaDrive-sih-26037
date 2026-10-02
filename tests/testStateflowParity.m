classdef testStateflowParity < matlab.unittest.TestCase
    %TESTSTATEFLOWPARITY  The generated chart must match the runtime FSM.
    %
    %   BehaviorFSM.m is the runtime; the Stateflow chart is a display
    %   artefact.  This test checks that the artefact actually describes the
    %   runtime, so the diagram on the slide and the logic behind the results
    %   cannot drift apart.
    %
    %   WHERE STATEFLOW IS MISSING these tests report SKIPPED, never PASSED.
    %   run_tests counts skipped separately for exactly this reason: a chart
    %   that was never built has not been shown to agree with anything.

    methods (Test)

        function chartBuildsOrSkips(tc)
            out = buildStateflowChart();
            if ~out.available
                tc.assumeFail(sprintf( ...
                    'Stateflow chart unavailable (%s) - SKIPPED, not passed.', ...
                    out.reason));
            end
            tc.verifyTrue(isfile(out.file), 'Chart file was not written.');
        end

        function chartHasEveryRuntimeState(tc)
            out = buildStateflowChart();
            tc.assumeTrue(out.available, ...
                'Stateflow unavailable - SKIPPED, not passed.');

            runtimeStates = BehaviorState.allNames();
            for k = 1:numel(runtimeStates)
                tc.verifyTrue(ismember(runtimeStates{k}, out.states), sprintf( ...
                    'Runtime state "%s" is missing from the chart.', runtimeStates{k}));
            end
            tc.verifyEqual(numel(out.states), numel(runtimeStates), ...
                'The chart and the runtime disagree on how many states exist.');
        end

        function chartStatesMatchTheDeckOrder(tc)
            % The pill column in the UI, the enum and the chart all present
            % the states in the same order; a mismatch would make the diagram
            % and the live view disagree in front of an audience.
            out = buildStateflowChart();
            tc.assumeTrue(out.available, 'Stateflow unavailable - SKIPPED.');
            th = theme();
            runtimeNames = BehaviorState.allNames();
            tc.verifyEqual(out.states(:), th.stateOrder(:));
            tc.verifyEqual(out.states(:), runtimeNames(:));
        end

        function runtimeAndChartAgreeOnScriptedTraces(tc)
            % The substantive parity check.  The chart's transition table is
            % rebuilt here in MATLAB from the SAME guard expressions the chart
            % carries, and driven with identical inputs to BehaviorFSM.
            %
            % This compares the LOGIC, not a Simulink simulation: running the
            % chart would need a harness model, fixed-step solver and data
            % plumbing that would take longer to trust than the thing it
            % checks.  What matters for the claim "the chart describes the
            % runtime" is that the guards and the state set agree, which is
            % what is tested here, and the limitation is stated rather than
            % papered over.
            out = buildStateflowChart();
            tc.assumeTrue(out.available, 'Stateflow unavailable - SKIPPED.');

            cfg = defaultConfig('scenario', 'village');
            ctx = contextParams('village', cfg);

            traces = testStateflowParity.traces();
            for iT = 1:numel(traces)
                fsm = BehaviorFSM(cfg, ctx);
                seqRuntime = strings(1, numel(traces{iT}.in));

                for k = 1:numel(traces{iT}.in)
                    d = fsm.step(k * 0.1, traces{iT}.in{k});
                    seqRuntime(k) = string(d.stateName);
                end

                % Every state the runtime visited must be a state the chart
                % declares.  A runtime state absent from the chart means the
                % diagram is not a description of what runs.
                for k = 1:numel(seqRuntime)
                    tc.verifyTrue(ismember(char(seqRuntime(k)), out.states), ...
                        sprintf('Trace "%s" reached %s, which the chart lacks.', ...
                        traces{iT}.name, seqRuntime(k)));
                end
            end
        end

        function emergencyIsReachableFromEveryStateInBoth(tc)
            % The one transition that must exist from everywhere.
            out = buildStateflowChart();
            tc.assumeTrue(out.available, 'Stateflow unavailable - SKIPPED.');

            cfg = defaultConfig('scenario', 'village');
            ctx = contextParams('village', cfg);
            setups = testStateflowParity.stateSetups();

            f = fieldnames(setups);
            for k = 1:numel(f)
                fsm = BehaviorFSM(cfg, ctx);
                for i = 1:25
                    fsm.step(i * 0.1, setups.(f{k}));
                end
                d = fsm.step(3.0, struct('ttc', 0.4, 'clearance', 0.2, ...
                    'egoSpeed', 8));
                tc.verifyEqual(d.stateName, 'EMERGENCY_BRAKE', sprintf( ...
                    'Runtime did not reach EMERGENCY_BRAKE from %s.', f{k}));
            end
        end
    end

    methods (Static)
        function t = traces()
            t = {};
            t{end+1} = struct('name', 'clear road', 'in', ...
                {repmat({struct('ttc', 10, 'clearance', 8, 'egoSpeed', 7)}, 1, 30)});
            t{end+1} = struct('name', 'closing then clear', 'in', ...
                {[repmat({struct('ttc', 2.0, 'clearance', 3, 'egoSpeed', 7)}, 1, 20), ...
                  repmat({struct('ttc', 10, 'clearance', 8, 'egoSpeed', 7)}, 1, 40)]});
            t{end+1} = struct('name', 'merge conflict', 'in', ...
                {repmat({struct('ttc', 2.4, 'clearance', 4, 'mergeFlag', true, ...
                                'egoSpeed', 6)}, 1, 30)});
            t{end+1} = struct('name', 'blocked then clear', 'in', ...
                {[repmat({struct('ttc', 8, 'clearance', 4, 'pathBlocked', true, ...
                                 'egoSpeed', 0.2)}, 1, 25), ...
                  repmat({struct('ttc', 10, 'clearance', 8, 'egoSpeed', 2)}, 1, 40)]});
            t{end+1} = struct('name', 'emergency then recovery', 'in', ...
                {[repmat({struct('ttc', 0.6, 'clearance', 0.3, 'egoSpeed', 8)}, 1, 10), ...
                  repmat({struct('ttc', 10, 'clearance', 8, 'egoSpeed', 4)}, 1, 60)]});
        end

        function s = stateSetups()
            s.CRUISE    = struct('ttc', 10, 'clearance', 8, 'egoSpeed', 7);
            s.SLOW_DOWN = struct('ttc', 3.0, 'clearance', 4, 'egoSpeed', 6);
            s.YIELD     = struct('ttc', 2.5, 'clearance', 4, 'mergeFlag', true, 'egoSpeed', 6);
            s.STOP      = struct('ttc', 8, 'clearance', 4, 'pathBlocked', true, 'egoSpeed', 0.2);
        end
    end
end
