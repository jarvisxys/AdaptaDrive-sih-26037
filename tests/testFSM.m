classdef testFSM < matlab.unittest.TestCase
    %TESTFSM  Behaviour decision layer: scripted traces to state sequences.
    %
    %   The hysteresis and dwell tests are the ones that matter in practice.
    %   A state machine that chatters is not merely untidy: every change costs
    %   a replan, and a planner replanning at 10 Hz because TTC is hovering on
    %   a threshold will miss its budget for a reason that never appears in
    %   any single-cycle test.

    properties
        cfg
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig('scenario', 'village');
        end
    end

    methods (Test)

        % ---------------- basics ----------------

        function startsInCruise(tc)
            f = BehaviorFSM(tc.cfg);
            tc.verifyEqual(f.state, BehaviorState.CRUISE);
            d = f.decision();
            tc.verifyEqual(d.stateName, 'CRUISE');
            tc.verifyEqual(d.speedCap, contextParams('village').speedCap);
            tc.verifyFalse(d.emergency);
        end

        function clearRoadStaysInCruise(tc)
            f = BehaviorFSM(tc.cfg);
            for k = 1:40
                d = f.step(k * 0.1, struct('ttc', 10, 'clearance', 8));
            end
            tc.verifyEqual(d.stateName, 'CRUISE');
        end

        % ---------------- entry ladder ----------------

        function lowTTCEntersSlowDown(tc)
            ctx = contextParams('village');
            f = BehaviorFSM(tc.cfg);
            ttc = ctx.ttcSlow - 0.5;
            for k = 1:20
                d = f.step(k * 0.1, struct('ttc', ttc, 'clearance', 5));
            end
            tc.verifyEqual(d.stateName, 'SLOW_DOWN');
            tc.verifyLessThan(d.speedCap, ctx.speedCap);
        end

        function highCorridorRiskEntersSlowDown(tc)
            f = BehaviorFSM(tc.cfg);
            for k = 1:20
                d = f.step(k * 0.1, struct('ttc', 9, 'clearance', 6, 'corridorRisk', 0.8));
            end
            tc.verifyEqual(d.stateName, 'SLOW_DOWN');
        end

        function mergeWithCloseTTCEntersYield(tc)
            f = BehaviorFSM(tc.cfg);
            for k = 1:20
                d = f.step(k * 0.1, struct('ttc', 2.5, 'clearance', 4, 'mergeFlag', true));
            end
            tc.verifyEqual(d.stateName, 'YIELD');
            tc.verifySubstring(d.reason, 'merge');
        end

        function blockedPathEntersStop(tc)
            f = BehaviorFSM(tc.cfg);
            for k = 1:20
                d = f.step(k * 0.1, struct('ttc', 8, 'clearance', 4, 'pathBlocked', true));
            end
            tc.verifyEqual(d.stateName, 'STOP');
            tc.verifyEqual(d.speedCap, 0);
        end

        function wrongWayNearbyEntersSlowDown(tc)
            f = BehaviorFSM(tc.cfg);
            for k = 1:20
                d = f.step(k * 0.1, struct('ttc', 9, 'clearance', 6, 'wrongWayNear', true));
            end
            tc.verifyEqual(d.stateName, 'SLOW_DOWN');
        end

        % ---------------- emergency override ----------------

        function emergencyTriggersFromAnyState(tc)
            % From every state, a collapsing TTC must reach EMERGENCY_BRAKE on
            % the very next cycle.
            setups = { ...
                'CRUISE',          struct('ttc', 10, 'clearance', 8); ...
                'SLOW_DOWN',       struct('ttc', 3.0, 'clearance', 4); ...
                'YIELD',           struct('ttc', 2.5, 'clearance', 4, 'mergeFlag', true); ...
                'STOP',            struct('ttc', 8, 'clearance', 4, 'pathBlocked', true)};

            for i = 1:size(setups, 1)
                f = BehaviorFSM(tc.cfg);
                for k = 1:20
                    f.step(k * 0.1, setups{i, 2});
                end
                tc.assertEqual(f.stateName, setups{i, 1}, ...
                    sprintf('Failed to set up state %s.', setups{i, 1}));

                d = f.step(2.1, struct('ttc', 0.8, 'clearance', 3));
                tc.verifyEqual(d.stateName, 'EMERGENCY_BRAKE', sprintf( ...
                    'Emergency did not override %s.', setups{i, 1}));
                tc.verifyTrue(d.emergency);
                tc.verifyEqual(d.speedCap, 0);
            end
        end

        function emergencyIgnoresMinimumDwell(tc)
            % The dwell timer must never delay an emergency.
            f = BehaviorFSM(tc.cfg);
            f.step(0.1, struct('ttc', 3.0, 'clearance', 4));   % enters via ladder
            d = f.step(0.15, struct('ttc', 0.5, 'clearance', 3));
            tc.verifyEqual(d.stateName, 'EMERGENCY_BRAKE', ...
                'Emergency must not wait for the dwell timer.');
        end

        function lowClearanceAloneTriggersEmergency(tc)
            f = BehaviorFSM(tc.cfg);
            d = f.step(0.1, struct('ttc', 9, 'clearance', 0.3));
            tc.verifyEqual(d.stateName, 'EMERGENCY_BRAKE');
        end

        function emergencyRecoversThroughSlowDown(tc)
            % Recovery must be gradual: EMERGENCY -> SLOW_DOWN -> CRUISE,
            % never straight back to full speed.
            f = BehaviorFSM(tc.cfg);
            f.step(0.1, struct('ttc', 0.5, 'clearance', 3));
            tc.assertEqual(f.stateName, 'EMERGENCY_BRAKE');

            seen = {};
            for k = 2:80
                d = f.step(k * 0.1, struct('ttc', 10, 'clearance', 8));
                seen{end+1} = d.stateName; %#ok<AGROW>
            end
            tc.verifyTrue(any(strcmp(seen, 'SLOW_DOWN')), ...
                'Recovery must pass through SLOW_DOWN.');
            tc.verifyEqual(seen{end}, 'CRUISE', 'Should eventually return to CRUISE.');

            iSlow = find(strcmp(seen, 'SLOW_DOWN'), 1);
            iCruise = find(strcmp(seen, 'CRUISE'), 1);
            tc.verifyLessThan(iSlow, iCruise, ...
                'SLOW_DOWN must come before CRUISE on the way back.');
        end

        % ---------------- dwell and hysteresis ----------------

        function minimumDwellIsRespected(tc)
            f = BehaviorFSM(tc.cfg);
            for k = 1:20
                f.step(k * 0.1, struct('ttc', 3.0, 'clearance', 4));
            end
            tc.assertEqual(f.stateName, 'SLOW_DOWN');

            % Road clears, but not for long enough to satisfy the dwell.
            d = f.step(2.1, struct('ttc', 10, 'clearance', 8));
            tc.verifyEqual(d.stateName, 'SLOW_DOWN', ...
                'State changed before the minimum dwell elapsed.');
        end

        function ttcHoveringOnThresholdDoesNotChatter(tc)
            % The test this class exists for.  TTC oscillates either side of
            % ttcSlow; without hysteresis the state flips every cycle.
            ctx = contextParams('village');
            f = BehaviorFSM(tc.cfg);

            states = {};
            for k = 1:120
                ttc = ctx.ttcSlow + 0.15 * sin(k * 1.7);   % +-0.15 s around it
                d = f.step(k * 0.1, struct('ttc', ttc, 'clearance', 5));
                states{end+1} = d.stateName; %#ok<AGROW>
            end

            changes = sum(~strcmp(states(1:end-1), states(2:end)));
            tc.verifyLessThanOrEqual(changes, 3, sprintf( ...
                ['State changed %d times while TTC hovered on the threshold; ' ...
                 'hysteresis and dwell are not holding.'], changes));
        end

        function returnToCruiseNeedsSustainedClearRoad(tc)
            % A single clear cycle must not restore full speed.
            f = BehaviorFSM(tc.cfg);
            for k = 1:20
                f.step(k * 0.1, struct('ttc', 3.0, 'clearance', 4));
            end
            tc.assertEqual(f.stateName, 'SLOW_DOWN');

            d = f.step(2.5, struct('ttc', 10, 'clearance', 8));
            tc.verifyEqual(d.stateName, 'SLOW_DOWN', ...
                'One clear cycle is not enough to return to CRUISE.');

            for k = 26:60
                d = f.step(k * 0.1, struct('ttc', 10, 'clearance', 8));
            end
            tc.verifyEqual(d.stateName, 'CRUISE');
        end

        % ---------------- overtake ----------------

        function overtakeNeedsSustainedClearCorridor(tc)
            % A corridor clear for one instant must not trigger a commit.
            f = BehaviorFSM(tc.cfg);
            in = struct('ttc', 8, 'clearance', 5, 'blockerSlow', true, ...
                'altCorridorClear', true);

            d = f.step(0.6, in);
            tc.verifyNotEqual(d.stateName, 'OVERTAKE_MERGE', ...
                'Overtake committed before the corridor had stayed clear.');

            for k = 7:40
                d = f.step(k * 0.1, in);
            end
            tc.verifyEqual(d.stateName, 'OVERTAKE_MERGE');
        end

        function overtakeAbortsIfCorridorCloses(tc)
            f = BehaviorFSM(tc.cfg);
            in = struct('ttc', 8, 'clearance', 5, 'blockerSlow', true, ...
                'altCorridorClear', true);
            for k = 1:40
                f.step(k * 0.1, in);
            end
            tc.assertEqual(f.stateName, 'OVERTAKE_MERGE');

            % Record the sequence: the abort is a TRANSITION, and with a clear
            % road afterwards the FSM legitimately recovers on to CRUISE.
            % Asserting only the final state would test the recovery, not the
            % abort.
            in.altCorridorClear = false;
            seen = {};
            for k = 41:60
                d = f.step(k * 0.1, in);
                seen{end+1} = d.stateName; %#ok<AGROW>
            end
            tc.verifyFalse(any(strcmp(seen, 'OVERTAKE_MERGE')), ...
                'A closing corridor must abort the overtake.');
            tc.verifyEqual(seen{1}, 'SLOW_DOWN', ...
                'The abort must drop to SLOW_DOWN, not jump straight to CRUISE.');
        end

        function overtakeCompletesThroughRejoin(tc)
            f = BehaviorFSM(tc.cfg);
            in = struct('ttc', 8, 'clearance', 5, 'blockerSlow', true, ...
                'altCorridorClear', true);
            for k = 1:40
                f.step(k * 0.1, in);
            end
            tc.assertEqual(f.stateName, 'OVERTAKE_MERGE');

            in.rejoinDone = true;
            seen = {};
            for k = 41:120
                if k <= 50
                    d = f.step(k * 0.1, in);
                else
                    d = f.step(k * 0.1, struct('ttc', 10, 'clearance', 8));
                end
                seen{end+1} = d.stateName; %#ok<AGROW>
            end

            tc.verifyTrue(any(strcmp(seen, 'REJOIN')), ...
                'A completed overtake must pass through REJOIN.');
            tc.verifyEqual(seen{1}, 'REJOIN', ...
                'REJOIN must follow immediately once the overtake completes.');
            tc.verifyEqual(seen{end}, 'CRUISE', ...
                'A clear road after rejoining must restore CRUISE.');
            tc.verifyLessThan(find(strcmp(seen, 'REJOIN'), 1), ...
                find(strcmp(seen, 'CRUISE'), 1), ...
                'REJOIN must precede the return to CRUISE.');
        end

        % ---------------- safety of defaults ----------------

        function missingInputsTakeTheSafeValue(tc)
            % Absent information must never read as "clear".  An empty struct
            % must not authorise an overtake.
            f = BehaviorFSM(tc.cfg);
            for k = 1:40
                d = f.step(k * 0.1, struct('blockerSlow', true));
            end
            tc.verifyNotEqual(d.stateName, 'OVERTAKE_MERGE', ...
                'An unknown corridor must not be treated as clear.');
        end

        function contextChangesTheThresholds(tc)
            % B4: the same trace must produce different behaviour on a highway
            % and in a market, because the guards come from contextParams.
            ttc = 4.2;      % below highway ttcSlow (5.0), above market (3.5)

            fH = BehaviorFSM(defaultConfig('scenario', 'highway'), contextParams('highway'));
            fM = BehaviorFSM(defaultConfig('scenario', 'market'), contextParams('market'));
            for k = 1:20
                dH = fH.step(k * 0.1, struct('ttc', ttc, 'clearance', 5));
                dM = fM.step(k * 0.1, struct('ttc', ttc, 'clearance', 5));
            end
            tc.verifyEqual(dH.stateName, 'SLOW_DOWN', ...
                'Highway should slow at 4.2 s TTC (threshold 5.0 s).');
            tc.verifyEqual(dM.stateName, 'CRUISE', ...
                'Market should not slow at 4.2 s TTC (threshold 3.5 s).');
        end

        function contextAblationRemovesTheDifference(tc)
            % With cfg.context off, both contexts use one parameter set.
            cfgH = defaultConfig('scenario', 'highway', 'context', false);
            cfgM = defaultConfig('scenario', 'market', 'context', false);
            tc.verifyEqual(contextParams('highway', cfgH).speedCap, ...
                           contextParams('market', cfgM).speedCap, ...
                'The context ablation must give every scenario the same parameters.');
        end
    end
end
