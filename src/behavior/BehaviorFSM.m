classdef BehaviorFSM < handle
    %BEHAVIORFSM  The behaviour decision layer (Section 5.8).
    %
    %   States, in the deck's order:
    %       CRUISE  SLOW_DOWN  YIELD  OVERTAKE_MERGE  REJOIN  STOP
    %       EMERGENCY_BRAKE
    %
    %   THIS CLASS IS THE RUNTIME.  The generated Stateflow chart
    %   (buildStateflowChart) is a display and parity artefact: it is shown in
    %   the UI and checked against this implementation by
    %   testStateflowParity, but it never drives a run.  Keeping one
    %   authoritative implementation means the results can never come from a
    %   model that differs from the one on the slide.
    %
    %   GUARDS come from contextParams, so the same logic behaves differently
    %   on a market lane and a highway (B4).  Two mechanisms keep the output
    %   from chattering, which matters because every state change costs the
    %   planner a replan:
    %
    %     minimum dwell   a state must be held 0.5 s before anything but an
    %                     emergency can replace it.
    %     hysteresis      the condition to LEAVE a cautious state is stricter
    %                     than the one to enter it, by HysteresisFactor.
    %                     Without it, a TTC hovering at the threshold would
    %                     flip SLOW_DOWN on and off every cycle.
    %
    %   EMERGENCY_BRAKE overrides everything, from any state, ignoring dwell.
    %   It also bypasses the planner: the controller applies full deceleration
    %   immediately rather than waiting for the next trajectory, because the
    %   replanning cycle is exactly the latency you cannot afford there.
    %
    %   See also CONTEXTPARAMS, BEHAVIORSTATE, BUILDSTATEFLOWCHART.

    properties (SetAccess = private)
        cfg
        ctx                 % active context parameter row
        state
        timeInState
        reason
        lastClearTime       % how long the road has been clear
        pathClearTime       % how long the planner has had a feasible path
        overtakeClearTime   % how long an alternative corridor has been clear
        t
    end

    properties (Constant)
        MinDwell = 0.5;             % s
        HysteresisFactor = 1.25;    % leaving is 25% stricter than entering
        ClearToCruise = 1.0;        % s of clear road before returning to CRUISE
        ClearToMoveOff = 0.7;       % s of clear path before pulling away from a stop
        OvertakeClearHold = 2.0;    % s an alternative must stay clear
        EmergencyClearance = 0.5;   % m
        StoppedSpeed = 0.5;         % m/s, below this the vehicle counts as stopped
    end

    methods
        function obj = BehaviorFSM(cfg, ctx)
            obj.cfg = cfg;
            if nargin < 2 || isempty(ctx)
                ctx = contextParams(cfg.scenario, cfg);
            end
            obj.ctx = ctx;
            obj.reset();
        end

        function reset(obj)
            obj.state = BehaviorState.CRUISE;
            obj.timeInState = 0;
            obj.reason = 'initial';
            obj.lastClearTime = 0;
            obj.pathClearTime = 0;
            obj.overtakeClearTime = 0;
            obj.t = 0;
        end

        % ----------------------------------------------------------------
        function d = step(obj, t, in)
            %STEP  Advance the state machine one cycle.
            %
            %   IN fields (all optional; missing ones take safe defaults):
            %       ttc              s, minimum TTC along the planned path
            %       clearance        m, minimum predicted clearance
            %       mergeFlag        logical, a merge was detected (A3)
            %       wrongWayNear     logical, a wrong-way agent is close (A2)
            %       pathBlocked      logical, no feasible path forward
            %       altCorridorClear logical, an overtake corridor is free
            %       blockerSlow      logical, the obstacle ahead is slow
            %       corridorRisk     [0,1], mean risk along the planned path
            %       rejoinDone       logical, the overtake has completed
            %
            %   Returns the DECISION contract (Section 4):
            %       state, speedCap, clearanceMargin, reason

            dt = max(t - obj.t, 0);
            obj.t = t;
            obj.timeInState = obj.timeInState + dt;

            in = obj.fillDefaults(in);
            obj.updateClearTimers(in, dt);

            next = obj.state;
            why = obj.reason;

            % --- 1. EMERGENCY overrides everything, from any state --------
            % ...but only while the vehicle is actually MOVING.  Emergency
            % braking exists to arrest motion; commanding it on a stationary
            % vehicle achieves nothing and creates a trap.  Once stopped, a
            % track whose position error puts it on top of the ego reports
            % TTC 0 and clearance 0 for as long as it persists, and the exit
            % condition - which demands a comfortable TTC and clearance - can
            % never be met.  Measured: the ego stopped for a crossing
            % pedestrian, traffic gathered around it, and it spent 76% of the
            % run latched in EMERGENCY_BRAKE at a standstill, never moving
            % again.
            %
            % Below this speed the ladder handles it instead, and a genuinely
            % blocked path becomes STOP - which CAN be left when the path
            % clears.  Nothing is lost in safety terms: stopping distance at
            % 0.5 m/s is under 2 cm.
            moving = in.egoSpeed > obj.StoppedSpeed;
            if moving && (in.ttc < obj.ctx.ttcEmergency || in.clearance < obj.EmergencyClearance)
                next = BehaviorState.EMERGENCY_BRAKE;
                why = sprintf('TTC %.2f s < %.2f s or clearance %.2f m < %.2f m', ...
                    in.ttc, obj.ctx.ttcEmergency, in.clearance, obj.EmergencyClearance);
                obj.transition(next, why);
                d = obj.decision();
                return
            end

            % --- 2. minimum dwell ----------------------------------------
            % Below the dwell time nothing but an emergency may change state.
            if obj.timeInState < obj.MinDwell
                d = obj.decision();
                return
            end

            % --- 3. ordinary transitions ---------------------------------
            switch obj.state
                case BehaviorState.EMERGENCY_BRAKE
                    % Leave only when genuinely clear, by the stricter test.
                    if in.ttc > obj.ctx.ttcEmergency * obj.HysteresisFactor && ...
                            in.clearance > obj.EmergencyClearance * obj.HysteresisFactor
                        if in.pathBlocked
                            next = BehaviorState.STOP;
                            why = 'emergency cleared but path still blocked';
                        else
                            next = BehaviorState.SLOW_DOWN;
                            why = 'emergency cleared, resuming cautiously';
                        end
                    end

                case BehaviorState.STOP
                    % Pulling away needs the path to have been clear for a
                    % sustained period, not merely clear on this one cycle.
                    % Measured failure: a single clear cycle - produced by a
                    % dropped track rather than by the obstacle actually
                    % leaving - was enough to move off into an oncoming car.
                    % Moving off from a standstill is the one decision where
                    % a moment's stale information is least recoverable.
                    if ~in.pathBlocked && obj.pathClearTime >= obj.ClearToMoveOff
                        next = BehaviorState.SLOW_DOWN;
                        why = 'path clear, moving off';
                    end

                case BehaviorState.YIELD
                    if ~in.mergeFlag || in.ttc > 3.0 * obj.HysteresisFactor
                        next = BehaviorState.SLOW_DOWN;
                        why = 'merge conflict resolved';
                    end

                case BehaviorState.OVERTAKE_MERGE
                    if in.rejoinDone
                        next = BehaviorState.REJOIN;
                        why = 'overtake complete, rejoining';
                    elseif ~in.altCorridorClear
                        next = BehaviorState.SLOW_DOWN;
                        why = 'overtake corridor no longer clear, aborting';
                    end

                case BehaviorState.REJOIN
                    if obj.lastClearTime >= obj.ClearToCruise
                        next = BehaviorState.CRUISE;
                        why = 'rejoined and clear';
                    end

                otherwise
                    % CRUISE and SLOW_DOWN share the same entry ladder.
                    [next, why] = obj.entryLadder(in);
            end

            obj.transition(next, why);
            d = obj.decision();
        end

        % ----------------------------------------------------------------
        function d = decision(obj)
            %DECISION  The decision contract for the planner and controller.
            speedCap = obj.ctx.speedCap;
            margin = obj.ctx.lateralClearance;

            switch obj.state
                case BehaviorState.CRUISE
                    % nominal
                case BehaviorState.SLOW_DOWN
                    speedCap = obj.ctx.speedCap * 0.55;
                    margin = margin * 1.2;
                case BehaviorState.YIELD
                    speedCap = obj.ctx.speedCap * 0.30;
                    margin = margin * 1.4;
                case BehaviorState.OVERTAKE_MERGE
                    speedCap = obj.ctx.speedCap * 0.80;
                    margin = margin * 1.5;
                case BehaviorState.REJOIN
                    speedCap = obj.ctx.speedCap * 0.70;
                    margin = margin * 1.2;
                case BehaviorState.STOP
                    speedCap = 0;
                    margin = margin * 1.5;
                case BehaviorState.EMERGENCY_BRAKE
                    speedCap = 0;
                    margin = margin * 1.5;
            end

            d = struct( ...
                'state', obj.state, ...
                'stateName', char(obj.state), ...
                'speedCap', speedCap, ...
                'clearanceMargin', margin, ...
                'emergency', obj.state == BehaviorState.EMERGENCY_BRAKE, ...
                'reason', obj.reason, ...
                'timeInState', obj.timeInState);
        end

        function n = stateName(obj)
            n = char(obj.state);
        end
    end

    % ====================================================================
    methods (Access = private)

        function [next, why] = entryLadder(obj, in)
            %ENTRYLADDER  Transitions available from CRUISE and SLOW_DOWN,
            %   most severe first.
            next = obj.state;
            why = obj.reason;

            if in.pathBlocked
                next = BehaviorState.STOP;
                why = 'path blocked, no feasible alternative';
                return
            end

            if in.mergeFlag && in.ttc < 3.0
                next = BehaviorState.YIELD;
                why = sprintf('merge detected, TTC %.1f s', in.ttc);
                return
            end

            % SOFT-CONSTRAINT PROTOTYPE SETTING: 0.70 rather than 0.50.  The
            % risk field now carries an always-on wrong-side component, so a
            % corridor on the correct side of an ordinary road already sits
            % well above 0.5 and the vehicle slowed almost permanently.
            if in.ttc < obj.ctx.ttcSlow || in.corridorRisk > 0.70
                next = BehaviorState.SLOW_DOWN;
                if in.ttc < obj.ctx.ttcSlow
                    why = sprintf('TTC %.1f s < %.1f s', in.ttc, obj.ctx.ttcSlow);
                else
                    why = sprintf('corridor risk %.2f', in.corridorRisk);
                end
                return
            end

            if in.wrongWayNear
                next = BehaviorState.SLOW_DOWN;
                why = 'wrong-way vehicle nearby';
                return
            end

            % Overtake needs a slow blocker AND a corridor that has stayed
            % clear: a corridor that is merely clear right now is how you
            % commit to an overtake and then meet oncoming traffic.
            if in.blockerSlow && in.altCorridorClear && ...
                    obj.overtakeClearTime >= obj.OvertakeClearHold
                next = BehaviorState.OVERTAKE_MERGE;
                why = 'slow blocker ahead, alternative corridor clear';
                return
            end

            % Returning to CRUISE uses the stricter hysteresis threshold.
            if obj.state == BehaviorState.SLOW_DOWN && ...
                    in.ttc > obj.ctx.ttcSlow * obj.HysteresisFactor && ...
                    in.corridorRisk < 0.35 && ...
                    obj.lastClearTime >= obj.ClearToCruise
                next = BehaviorState.CRUISE;
                why = 'clear';
            end
        end

        % ----------------------------------------------------------------
        function updateClearTimers(obj, in, dt)
            clearNow = in.ttc > obj.ctx.ttcSlow * obj.HysteresisFactor && ...
                       in.corridorRisk < 0.35 && ...
                       ~in.pathBlocked && ~in.mergeFlag;
            if clearNow
                obj.lastClearTime = obj.lastClearTime + dt;
            else
                obj.lastClearTime = 0;
            end

            if in.altCorridorClear
                obj.overtakeClearTime = obj.overtakeClearTime + dt;
            else
                obj.overtakeClearTime = 0;
            end

            if in.pathBlocked
                obj.pathClearTime = 0;
            else
                obj.pathClearTime = obj.pathClearTime + dt;
            end
        end

        % ----------------------------------------------------------------
        function transition(obj, next, why)
            if next ~= obj.state
                obj.state = next;
                obj.timeInState = 0;
            end
            obj.reason = why;
        end

        % ----------------------------------------------------------------
        function in = fillDefaults(~, in)
            %FILLDEFAULTS  Missing inputs take the SAFE value, not a
            %   convenient one: absent information must never read as "clear".
            d = struct( ...
                'ttc', 10.0, ...
                'clearance', 10.0, ...
                'mergeFlag', false, ...
                'wrongWayNear', false, ...
                'pathBlocked', false, ...
                'altCorridorClear', false, ...   % no evidence = not clear
                'blockerSlow', false, ...
                'corridorRisk', 0.0, ...
                'rejoinDone', false, ...
                'egoSpeed', 99);      % unknown speed counts as moving

            f = fieldnames(d);
            for k = 1:numel(f)
                if ~isfield(in, f{k}) || isempty(in.(f{k})) || ...
                        (isnumeric(in.(f{k})) && isnan(in.(f{k})))
                    in.(f{k}) = d.(f{k});
                end
            end
        end
    end
end
