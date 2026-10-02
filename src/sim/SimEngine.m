classdef SimEngine < handle
    %SIMENGINE  The closed loop (Section 5.12).
    %
    %   Perceive -> Track -> Risk -> Predict -> Decide -> Plan -> Control ->
    %   Vehicle -> back to Perceive.  Ego motion is never scripted: the
    %   vehicle state is produced only by KinematicBicycle integrating the
    %   commands the controllers actually issued.
    %
    %   RATES
    %     cfg.sim.dt        (0.05 s) control, vehicle and agent update
    %     cfg.sim.planRate  (10 Hz)  sensing, tracking, risk, prediction, FSM, DWA
    %
    %   STEP ORDER, and why it matters
    %     1. snapshot the world at time t and log it
    %     2. test termination against that snapshot
    %     3. run the planning cycle from that snapshot (timed)
    %     4. apply control and integrate the vehicle
    %     5. advance the agents
    %   So log entry k is one consistent instant, and the planner is never
    %   allowed to see a world that has already moved in response to it.
    %
    %   CONFIGURATIONS
    %     scripted   M1 SCAFFOLD ONLY.  Fixed reference path, and longitudinal
    %                braking computed from GROUND TRUTH.  It exists to prove
    %                the loop and the vehicle model run; it is not a reported
    %                configuration and run_experiments refuses to use it.
    %     PROPOSED / BL1 / BL2 / BL3 and the ablations arrive at M4-M5.
    %
    %   See also SIMLOG, METRICSLOGGER, KINEMATICBICYCLE, BUILDSCENARIO.

    properties (SetAccess = private)
        cfg
        scenario
        ctx             % active context parameter row (B4)
        vehicle         % KinematicBicycle
        pp              % PurePursuit
        speedCtl        % SpeedController
        log             % SimLog
        sensors         % SensorSuite, when the config uses perception
        tracker         % TrackerWrapper or SimpleKFTracker
        predictor       % Predictor, when the config uses the risk stack
        riskMap         % RiskMap
        wrongWay        % WrongWayDetector (A2)
        mergeDet        % MergeDetector (A3)
        globalPlanner   % GlobalPlanner (Hybrid A* + RRT*)
        dwa             % DWAPlanner
        fsm             % BehaviorFSM
        lastTracks
        lastPreds
        lastWrongWay = []
        lastMerge = []
        globalPath      % current global path, world frame
        lastReplanTime = -inf
        lastBlocked = false
        lastDecision
        lastDwa
        emergency = false
        activePath      % path currently being tracked
        vTarget
        outcome
    end

    methods
        function obj = SimEngine(scenario, cfg)
            obj.cfg = cfg;
            obj.scenario = scenario;
            obj.ctx = contextParams(scenario.context, cfg);

            obj.vehicle  = KinematicBicycle(cfg, scenario.ego);
            obj.pp       = PurePursuit(cfg);
            obj.speedCtl = SpeedController(cfg);

            % Seed the GLOBAL stream: the toolbox sensor generators draw their
            % noise from it.  Their own InitialSeed/InitialSeedSource
            % properties warn "not relevant in this configuration" on R2026a
            % and have no effect (DEVIATIONS D17), so this is the only handle
            % on their reproducibility.  Everything AdaptaDrive owns uses
            % private RandStreams and is unaffected by global state.
            rng(cfg.seed, 'twister');

            % Perception is built only for configurations that use it, so the
            % M1 scaffold does not pay for sensors it never reads.
            if SimEngine.usesPerception(cfg.name)
                obj.sensors = SensorSuite(cfg, scenario);
                if strcmp(cfg.env.backends.tracker, 'multiObjectTracker')
                    obj.tracker = TrackerWrapper(cfg);
                else
                    obj.tracker = SimpleKFTracker(cfg);
                end
            end
            if SimEngine.usesRiskStack(cfg)
                obj.predictor = Predictor(cfg, scenario.road);
                obj.riskMap   = RiskMap(cfg, scenario.road, scenario.hazards);
                obj.wrongWay  = WrongWayDetector(cfg, scenario.road);
                obj.mergeDet  = MergeDetector(cfg);
            end

            if SimEngine.usesPlanner(cfg)
                if cfg.useGlobalPlanner
                    obj.globalPlanner = GlobalPlanner(cfg);
                end
                if cfg.useDWA
                    obj.dwa = DWAPlanner(cfg, scenario.road);
                end
                obj.fsm = BehaviorFSM(cfg, obj.ctx);
            end

            obj.lastTracks = TrackerWrapper.emptyTracks();
            obj.lastPreds  = Predictor.emptyPredictions();

            meta = struct( ...
                'scenario', scenario.name, ...
                'scenarioTitle', scenario.title, ...
                'config', cfg.name, ...
                'seed', cfg.seed, ...
                'density', cfg.density, ...
                'context', scenario.context, ...
                'contextParams', obj.ctx, ...
                'env', cfg.env, ...
                'cfg', cfg, ...
                'purePursuitBackend', obj.pp.backend);
            obj.log = SimLog(meta);

            obj.activePath = scenario.refPath;
            obj.vTarget = obj.ctx.speedCap;
        end

        % ----------------------------------------------------------------
        function result = run(obj, varargin)
            %RUN  Execute the scenario to completion.

            p = inputParser;
            p.addParameter('verbose', false, @(x) islogical(x) || isnumeric(x));
            p.parse(varargin{:});
            verbose = logical(p.Results.verbose);

            dt = obj.cfg.sim.dt;
            planEvery = max(1, round(1 / (obj.cfg.sim.planRate * dt)));
            maxTime = obj.cfg.sim.maxTimeMult * obj.scenario.nominalTime;
            maxSteps = ceil(maxTime / dt);

            obj.outcome = struct('success', false, 'reason', 'running', 'detail', '');

            for k = 0:maxSteps
                t = k * dt;
                egoState = obj.vehicle.state();

                % --- 1. snapshot and log ------------------------------
                truths = obj.agentTruths();
                obj.log.append(egoState, truths);

                % --- 2. termination -----------------------------------
                oc = obj.checkTermination(egoState, truths, t, maxTime);
                if ~strcmp(oc.reason, 'running')
                    obj.outcome = oc;
                    obj.log.addEvent(t, upper(oc.reason), oc.detail);
                    break
                end

                % --- 3. planning cycle --------------------------------
                if mod(k, planEvery) == 0
                    obj.planCycle(egoState, t, truths, k);
                end

                % --- 4. control and vehicle ---------------------------
                % EMERGENCY_BRAKE bypasses the planner entirely: full
                % deceleration is applied on the spot rather than waiting for
                % the next trajectory.  The replanning cycle is exactly the
                % latency you cannot afford in that state (Section 5.8).
                steer = obj.pp.step(egoState, obj.activePath);
                accel = obj.speedCtl.step(dt, egoState.v, obj.vTarget, obj.emergency);
                obj.vehicle.step(dt, accel, steer);

                % --- 5. agents ----------------------------------------
                world = obj.buildWorld(egoState, truths);
                for i = 1:numel(obj.scenario.agents)
                    obj.scenario.agents(i).step(dt, t, world);
                end

                if verbose && mod(k, 100) == 0
                    fprintf('  t=%6.2f s  v=%5.2f m/s  pos=(%7.2f,%7.2f)\n', ...
                        t, egoState.v, egoState.x, egoState.y);
                end
            end

            if strcmp(obj.outcome.reason, 'running')
                obj.outcome = struct('success', false, 'reason', 'timeout', ...
                    'detail', sprintf('exceeded %.1f s (3x nominal)', maxTime));
                obj.log.addEvent(obj.log.t(obj.log.n), 'TIMEOUT', obj.outcome.detail);
            end

            obj.log.finalize(obj.outcome);

            result = struct();
            result.log = obj.log;
            result.outcome = obj.outcome;
            result.metrics = MetricsLogger.compute(obj.log, obj.scenario, obj.cfg);
            if ~isempty(obj.sensors)
                result.sensorReport = obj.sensors.report();
            end
        end
    end

    % ====================================================================
    methods (Static)
        function sm = blankStageMs()
            %BLANKSTAGEMS  Per-stage timing record for one planning cycle.
            %
            %   Zero, not NaN, is the honest default here: a stage that did not
            %   run in this configuration (BL1 has no risk map, BL2 no FSM)
            %   genuinely consumed no time, and the seven fields must sum to
            %   slightly under the cycle's latencyMs.  NaN would poison that
            %   sum and hide the accounting error if one appeared.
            sm = struct('sense', 0, 'track', 0, 'predict', 0, 'detect', 0, ...
                'risk', 0, 'fsm', 0, 'global', 0, 'dwa', 0);
        end

        function names = stageNames()
            names = {'sense', 'track', 'predict', 'detect', 'risk', ...
                     'fsm', 'global', 'dwa'};
        end

        function tf = usesPerception(configName)
            %USESPERCEPTION  Which configurations build a perception stack.
            %   BL1-BL3, PROPOSED and the ablations all perceive; only the M1
            %   ground-truth scaffold does not.
            tf = ~strcmp(configName, 'scripted');
        end

        function tf = usesPlanner(cfg)
            %USESPLANNER  Does this configuration run a planner and an FSM?
            tf = ~ismember(cfg.name, {'scripted', 'perception', 'risk'});
        end

        function tf = usesRiskStack(cfg)
            %USESRISKSTACK  Does this configuration build risk/prediction/detectors?
            %   BL1 deliberately does not: no risk map and no prediction is
            %   exactly what makes it the lane-follow baseline.
            tf = ~strcmp(cfg.name, 'scripted') && ...
                 (cfg.useRiskMap || ismember(cfg.name, {'risk', 'perception'}));
        end

        function preds = freezePredictions(preds, tracks)
            %FREEZEPREDICTIONS  BL3: every agent held at its current pose.
            %
            %   The baseline that "sees" everything but assumes nothing moves.
            %   Implemented by flattening the prediction means rather than by
            %   bypassing the predictor, so BL3 differs from PROPOSED in
            %   exactly one respect: what the future is assumed to be.
            ids = [tracks.trackId];
            for i = 1:numel(preds)
                j = find(ids == preds(i).trackId, 1);
                if isempty(j)
                    continue
                end
                here = [tracks(j).x, tracks(j).y];
                for k = 1:numel(preds(i).modes)
                    n = size(preds(i).modes(k).mu, 1);
                    preds(i).modes(k).mu = repmat(here, n, 1);
                    preds(i).modes(k).heading = repmat( ...
                        headingOrZero(tracks(j)), n, 1);
                end
                % A frozen world has one future, not several.
                preds(i).modes = preds(i).modes(1);
                preds(i).modes(1).prob = 1.0;
            end
        end

        function q = quantiseRisk(R)
            %QUANTISERISK  Store the risk field as uint8 for replay.
            %
            %   A 360x120 double field at 10 Hz is 35 MB per simulated minute,
            %   which makes replay logs unusable.  uint8 costs 8.6 MB/min and
            %   quantises to 1/255 - far below the resolution at which the
            %   field is ever read.  DISPLAY AND REPLAY ONLY: every metric is
            %   computed from the full-precision field during the run.
            q = uint8(min(max(R, 0), 1) * 255);
        end
    end

    % ====================================================================
    methods (Access = private)

        function planCycle(obj, egoState, t, truths, k)
            %PLANCYCLE  Decide the path and target speed for the next 0.1 s.

            switch obj.cfg.name
                case 'scripted'
                    % M1 scaffold.  Fixed path, and a gap kept using GROUND
                    % TRUTH rather than perception.  Deliberately not timed:
                    % reporting a latency here would invite comparison with
                    % the 200 ms budget for a cycle that does no planning.
                    obj.activePath = obj.scenario.refPath;
                    obj.vTarget = obj.scriptedSpeedTarget(egoState, truths);

                case 'perception'
                    % M2 scaffold.  Real sensing and tracking run every cycle
                    % and are timed, but the DRIVING is still the scripted
                    % ground-truth scaffold - there is no planner yet.  The
                    % timing recorded here is perception only and is labelled
                    % as such, so it is never compared with the 200 ms
                    % full-cycle budget.
                    tCycle = tic;
                    obj.syncContainer(egoState, truths);
                    [dets, raw] = obj.sensors.step(t, egoState, obj.scenario);
                    obj.lastTracks = obj.tracker.step(t, dets, raw);
                    latencyMs = toc(tCycle) * 1000;

                    obj.log.appendCycle(struct( ...
                        't', t, 'step', obj.log.n, 'latencyMs', latencyMs, ...
                        'stage', 'perception', 'tracks', obj.lastTracks, ...
                        'nDets', numel(dets)));

                    obj.activePath = obj.scenario.refPath;
                    obj.vTarget = obj.scriptedSpeedTarget(egoState, truths);

                case 'risk'
                    % M3 scaffold.  The full perception -> prediction -> risk
                    % chain runs and is timed; the detectors write to the event
                    % log.  Driving is still the scripted ground-truth
                    % scaffold, because there is no planner until M4.  So this
                    % configuration exercises and measures everything the
                    % planner will consume, without pretending to be it.
                    tCycle = tic;
                    obj.syncContainer(egoState, truths);
                    [dets, raw] = obj.sensors.step(t, egoState, obj.scenario);
                    obj.lastTracks = obj.tracker.step(t, dets, raw);
                    obj.lastPreds  = obj.predictor.predict(obj.lastTracks);

                    [wwIds, wwEvents] = obj.wrongWay.step(t, obj.lastTracks);
                    obj.lastWrongWay = wwIds;

                    [mgIds, mgEvents, mgTTC] = obj.mergeDet.step(t, ...
                        obj.lastTracks, obj.lastPreds, egoState, obj.activePath);
                    obj.lastMerge = mgIds;

                    obj.riskMap.update(egoState, obj.lastPreds, obj.lastTracks, wwIds);
                    latencyMs = toc(tCycle) * 1000;

                    for e = wwEvents
                        obj.log.addEvent(t, 'WRONG_WAY', e.text, e);
                    end
                    for e = mgEvents
                        obj.log.addEvent(t, 'MERGE', e.text, e);
                    end

                    obj.log.appendCycle(struct( ...
                        't', t, 'step', obj.log.n, 'latencyMs', latencyMs, ...
                        'stage', 'risk', 'tracks', obj.lastTracks, ...
                        'nDets', numel(dets), ...
                        'risk', SimEngine.quantiseRisk(obj.riskMap.R), ...
                        'riskMeta', obj.riskMeta(), ...
                        'preds', obj.lastPreds, ...
                        'wrongWayIds', wwIds, 'mergeIds', mgIds, 'mergeTTC', mgTTC));

                    obj.activePath = obj.scenario.refPath;
                    obj.vTarget = obj.scriptedSpeedTarget(egoState, truths);

                % STRESS-* run the full proposed loop unchanged; the stress is
                % applied inside SensorSuite, not by taking a different path
                % through the loop.  It is listed explicitly rather than caught
                % by a wildcard so an unknown config name still errors instead
                % of silently running as PROPOSED.
                case {'PROPOSED', 'BL1', 'BL2', 'BL3', '-riskmap', ...
                      '-classpriors', '-uncertainty', '-context', ...
                      '-wrongside', 'STRESS-sensor'}
                    obj.proposedCycle(egoState, t, truths);

                otherwise
                    error('SimEngine:configNotImplemented', ...
                        ['Configuration "%s" is not implemented. Available: ' ...
                         '"scripted" and "perception" scaffolds, PROPOSED, ' ...
                         'BL1-BL3, the five ablations, and STRESS-sensor.'], ...
                        obj.cfg.name);
            end
        end

        % ----------------------------------------------------------------
        function proposedCycle(obj, egoState, t, truths)
            %PROPOSEDCYCLE  The full closed loop, timed end to end.
            %
            %   Everything inside the tic/toc is what the 200 ms budget covers:
            %   sensing, tracking, prediction, detectors, risk map, behaviour,
            %   global replanning when triggered, and the local planner.
            %
            %   Each stage is timed separately as well.  A single end-to-end
            %   number tells you a cycle missed the budget; it does not tell you
            %   which stage to go and fix.  The per-stage split is what makes
            %   the latency table actionable, and it is summarised per run into
            %   runs.csv as stage_<name>_ms.

            tCycle = tic;
            sm = SimEngine.blankStageMs();

            % --- perceive ------------------------------------------------
            tS = tic;
            obj.syncContainer(egoState, truths);
            [dets, raw] = obj.sensors.step(t, egoState, obj.scenario);
            sm.sense = toc(tS) * 1000;

            tS = tic;
            obj.lastTracks = obj.tracker.step(t, dets, raw);
            sm.track = toc(tS) * 1000;

            wwIds = []; mgIds = []; mgTTC = NaN;
            wwEvents = []; mgEvents = [];

            if ~isempty(obj.predictor)
                tS = tic;
                obj.lastPreds = obj.predictor.predict(obj.lastTracks);
                if obj.cfg.freezeAgents
                    % BL3: no motion prediction.  Every agent is assumed to
                    % stay exactly where it is for the whole horizon.
                    obj.lastPreds = SimEngine.freezePredictions( ...
                        obj.lastPreds, obj.lastTracks);
                end
                sm.predict = toc(tS) * 1000;

                % --- named detectors ----------------------------------
                tS = tic;
                [wwIds, wwEvents] = obj.wrongWay.step(t, obj.lastTracks);
                [mgIds, mgEvents, mgTTC] = obj.mergeDet.step(t, obj.lastTracks, ...
                    obj.lastPreds, egoState, obj.activePath);
                sm.detect = toc(tS) * 1000;

                % --- risk ---------------------------------------------
                tS = tic;
                obj.riskMap.update(egoState, obj.lastPreds, obj.lastTracks, wwIds);
                sm.risk = toc(tS) * 1000;
            else
                obj.lastPreds = Predictor.emptyPredictions();
            end
            obj.lastWrongWay = wwIds;
            obj.lastMerge = mgIds;

            % --- decide --------------------------------------------------
            % pathBlocked comes from the PREVIOUS cycle's local planner: the
            % FSM sets the speed cap the planner then plans under, so the
            % dependency has to be broken somewhere.  One cycle (0.1 s) of lag
            % on a blocked-path flag is the cheapest place to break it.
            tS = tic;
            fsmIn = obj.behaviourInputs(egoState, wwIds, mgIds, mgTTC);
            if obj.cfg.useFSM
                decision = obj.fsm.step(t, fsmIn);
            else
                % BL2: no behaviour layer.  Always CRUISE, so the only thing
                % slowing the vehicle is the local planner's own cost.
                decision = obj.fsm.decision();
            end
            obj.lastDecision = decision;
            obj.emergency = decision.emergency;
            sm.fsm = toc(tS) * 1000;

            % --- global replan, periodic or event triggered ---------------
            tS = tic;
            replanReason = '';
            plannerUsed = 'none';
            if obj.cfg.useGlobalPlanner
                replanReason = obj.replanTrigger(t, decision);
                plannerUsed = 'cached';
            end
            if ~isempty(replanReason)
                obj.globalPlanner.notePathHeld(~isempty(obj.globalPath));
                g = obj.globalPlanner.plan(obj.riskMap, egoState, ...
                    obj.localGoal(egoState), decision);
                plannerUsed = g.plannerUsed;
                if g.success
                    obj.globalPath = g.path;
                    obj.lastReplanTime = t;
                    obj.log.addEvent(t, 'REPLAN', sprintf( ...
                        'replan (%s) via %s in %.0f ms', replanReason, ...
                        g.plannerUsed, g.latencyMs));
                else
                    obj.log.addEvent(t, 'REPLAN_FAIL', sprintf( ...
                        'replan (%s) failed: %s', replanReason, g.reason));
                end
            end
            sm.global = toc(tS) * 1000;

            % --- local plan ----------------------------------------------
            tS = tic;
            if obj.cfg.useDWA
                d = obj.dwa.plan(egoState, obj.riskMap, obj.globalPath, ...
                    obj.lastPreds, decision, obj.ctx, obj.localGoal(egoState));
            else
                % BL1: no local planner.  Track the road centreline and
                % regulate speed with an IDM-style gap law on the nearest
                % tracked obstacle in a narrow corridor.  No lateral
                % avoidance whatsoever - that absence is the baseline.
                d = obj.laneFollowPlan(egoState, decision);
            end
            sm.dwa = toc(tS) * 1000;
            obj.lastDwa = d;
            obj.lastBlocked = d.blocked;

            if d.feasible
                if obj.cfg.useDWA
                    obj.activePath = d.traj(:, 1:2);
                end
                obj.vTarget = min(d.chosenV, decision.speedCap);

                % CAR FOLLOWING.  The behaviour states cap speed as a fraction
                % of the context limit, which says nothing about how fast the
                % vehicle in front is going.  On the highway SLOW_DOWN caps at
                % 9.2 m/s while the truck ahead does 6.9, so the ego closed
                % the gap under a state that believed it was being cautious
                % and rear-ended it at 378 m.  A following term is what
                % actually matches a leader's speed.
                obj.vTarget = min(obj.vTarget, ...
                    obj.followingSpeed(fsmIn.leadGap, fsmIn.leadSpeed, egoState));
            else
                % Nothing feasible.  Hold the last path for steering continuity
                % and let the controller brake; the FSM sees pathBlocked next
                % cycle and commands STOP.
                obj.vTarget = 0;
            end

            if decision.emergency
                obj.vTarget = 0;
            end

            latencyMs = toc(tCycle) * 1000;

            % --- log ------------------------------------------------------
            for e = wwEvents
                obj.log.addEvent(t, 'WRONG_WAY', e.text, e);
            end
            for e = mgEvents
                obj.log.addEvent(t, 'MERGE', e.text, e);
            end
            if decision.emergency && obj.fsm.timeInState == 0
                obj.log.addEvent(t, 'EMERGENCY_BRAKE', decision.reason);
            end

            % BL1 has no risk map, so there is no field to store for it.  An
            % empty entry replays as "no risk layer", which is the truth,
            % rather than a zero field that would look like "no risk".
            %
            % Heavy visualisation payloads (risk field, candidate fan,
            % predictions) are stored on a stride in 'light' mode.  They are
            % for DISPLAY: every metric is computed from the full-rate data.
            keepDetail = strcmp(obj.cfg.io.logDetail, 'full') || ...
                mod(obj.log.nCycles, obj.cfg.io.detailStride) == 0;

            if isempty(obj.riskMap) || ~keepDetail
                riskQ = []; riskM = [];
            else
                riskQ = SimEngine.quantiseRisk(obj.riskMap.R);
                riskM = obj.riskMeta();
            end
            if keepDetail
                candStore = d.candidates;
                predStore = obj.lastPreds;
            else
                candStore = {};
                predStore = [];
            end

            obj.log.appendCycle(struct( ...
                't', t, 'step', obj.log.n, 'latencyMs', latencyMs, ...
                'stage', 'plan', 'tracks', obj.lastTracks, ...
                'nDets', numel(dets), ...
                'risk', riskQ, 'riskMeta', riskM, ...
                'preds', predStore, ...
                'wrongWayIds', wwIds, 'mergeIds', mgIds, 'mergeTTC', mgTTC, ...
                'state', decision.stateName, 'reason', decision.reason, ...
                'plannerUsed', plannerUsed, 'replanReason', replanReason, ...
                'globalPath', obj.globalPath, 'localTraj', d.traj, ...
                'candidates', {candStore}, 'nRejected', d.nRejected, ...
                'speedCap', decision.speedCap, ...
                'vTarget', obj.vTarget, 'corridorRisk', fsmIn.corridorRisk, ...
                'minTTC', fsmIn.ttc, 'minClear', fsmIn.clearance, ...
                'stageMs', sm, ...
                'antiStall', isfield(d, 'antiStall') && d.antiStall));
        end

        % ----------------------------------------------------------------
        function vT = scriptedSpeedTarget(obj, egoState, truths)
            %SCRIPTEDSPEEDTARGET  Ground-truth gap keeping for the scaffold.
            %
            %   IDM-style: full speed when clear, proportional slow-down
            %   inside the context following gap, stop at 2 m.  Uses truth
            %   because M1 has no perception yet - which is exactly why this
            %   configuration is not reportable.
            vT = obj.ctx.speedCap;

            egoBox = obj.vehicle.footprint(egoState);
            heading = [cos(egoState.yaw), sin(egoState.yaw)];
            stopGap = 2.0;
            gap = inf;

            for i = 1:numel(truths)
                a = truths(i);
                if ~a.active
                    continue
                end
                rel = [a.x - egoBox.x, a.y - egoBox.y];
                along = rel * heading.';
                if along <= 0
                    continue    % behind us
                end
                lateral = abs(rel(1) * -heading(2) + rel(2) * heading(1));
                if lateral > 0.5 * (obj.cfg.vehicle.width + a.width) + 0.3
                    continue    % not in our corridor
                end
                gap = min(gap, along - 0.5 * (obj.cfg.vehicle.length + a.length));
            end

            % Solid obstacles too.
            statics = obj.scenario.hazards.staticOBBs();
            for i = 1:numel(statics)
                rel = [statics(i).x - egoBox.x, statics(i).y - egoBox.y];
                along = rel * heading.';
                if along <= 0, continue, end
                lateral = abs(rel(1) * -heading(2) + rel(2) * heading(1));
                if lateral > 0.5 * (obj.cfg.vehicle.width + statics(i).W) + 0.3
                    continue
                end
                gap = min(gap, along - 0.5 * (obj.cfg.vehicle.length + statics(i).L));
            end

            if isfinite(gap)
                if gap < stopGap
                    vT = 0;
                elseif gap < obj.ctx.followingGap
                    vT = obj.ctx.speedCap * (gap - stopGap) / ...
                        (obj.ctx.followingGap - stopGap);
                end
            end

            % Ease off approaching the goal so the run ends on the marker
            % rather than overshooting it.
            dGoal = hypot(obj.scenario.ego.goal(1) - egoState.x, ...
                          obj.scenario.ego.goal(2) - egoState.y);
            if dGoal < 12
                vT = min(vT, max(1.5, obj.ctx.speedCap * dGoal / 12));
            end
        end

        % ----------------------------------------------------------------
        function oc = checkTermination(obj, egoState, truths, t, maxTime)
            %CHECKTERMINATION  Goal, collision, off-road or timeout.
            oc = struct('success', false, 'reason', 'running', 'detail', '');

            egoBox = obj.vehicle.footprint(egoState);

            % --- collision with an agent ---------------------------------
            for i = 1:numel(truths)
                a = truths(i);
                if ~a.active
                    continue
                end
                if obbOverlap(egoBox, makeOBB(a.x, a.y, a.yaw, a.length, a.width))
                    oc.reason = 'collision';
                    oc.detail = sprintf('ego hit %s #%d at t=%.2f s', ...
                        a.className, a.id, t);
                    return
                end
            end

            % --- collision with a solid obstacle -------------------------
            statics = obj.scenario.hazards.staticOBBs();
            for i = 1:numel(statics)
                if obbOverlap(egoBox, statics(i))
                    oc.reason = 'collision';
                    oc.detail = sprintf('ego hit static obstacle #%d at t=%.2f s', i, t);
                    return
                end
            end

            % --- left the drivable area ----------------------------------
            % Defined on the BODY CENTRE.  A corner overhanging the edge is
            % not a failure - it is penalised by the edge-risk layer - but the
            % vehicle leaving the carriageway is.
            if ~obj.scenario.road.isDrivable(egoBox.x, egoBox.y)
                oc.reason = 'offroad';
                oc.detail = sprintf('body centre left the drivable area at t=%.2f s', t);
                return
            end

            % --- goal -----------------------------------------------------
            dGoal = hypot(obj.scenario.ego.goal(1) - egoState.x, ...
                          obj.scenario.ego.goal(2) - egoState.y);
            if dGoal <= obj.scenario.ego.goalTol
                oc.success = true;
                oc.reason = 'goal';
                oc.detail = sprintf('reached goal at t=%.2f s (%.2f m from marker)', t, dGoal);
                return
            end

            % --- timeout --------------------------------------------------
            if t >= maxTime
                oc.reason = 'timeout';
                oc.detail = sprintf('exceeded %.1f s (3x nominal travel time)', maxTime);
                return
            end
        end

        % ----------------------------------------------------------------
        function in = behaviourInputs(obj, egoState, wwIds, mgIds, mgTTC)
            %BEHAVIOURINPUTS  Turn perception into the FSM's guard variables.
            %
            %   Everything here is computed from TRACKS, never from ground
            %   truth.  A behaviour layer fed truth would make the whole
            %   comparison against the baselines meaningless.

            veh = obj.cfg.vehicle;
            off = veh.length / 2 - veh.rearOverhang;
            egoBox = makeOBB(egoState.x + off*cos(egoState.yaw), ...
                             egoState.y + off*sin(egoState.yaw), ...
                             egoState.yaw, veh.length, veh.width);
            egoVel = [egoState.v * cos(egoState.yaw), egoState.v * sin(egoState.yaw)];

            minTTC = 10.0;
            minClear = 10.0;
            blockerSlow = false;
            wrongWayNear = false;
            leadGap = inf;
            leadSpeed = inf;

            heading = [cos(egoState.yaw), sin(egoState.yaw)];
            normal = [-heading(2), heading(1)];

            for k = 1:numel(obj.lastTracks)
                tr = obj.lastTracks(k);
                prior = classPriors(tr.classId);
                box = makeOBB(tr.x, tr.y, headingOrZero(tr), prior.length, prior.width);

                rel = [tr.x - egoBox.x, tr.y - egoBox.y];
                along = rel * heading.';
                lat = abs(rel * normal.');

                % TTC only for what is ACTIONABLE: ahead of us, and near
                % enough laterally that we could ever meet it.  Taking the
                % minimum over every track meant oncoming traffic passing
                % safely a metre to the side - plus its duplicate tracks -
                % held minTTC near the emergency threshold for whole runs, so
                % the vehicle drove the entire scenario in SLOW_DOWN.  The
                % clearance metric below is still measured over everything,
                % because that is a safety OUTCOME rather than a decision
                % input.
                if along > -2.0 && lat < 4.0
                    minTTC = min(minTTC, ttc(egoBox, egoVel, box, [tr.vx, tr.vy]));
                end
                minClear = min(minClear, obbDistance(egoBox, box));

                % A slow blocker: ahead, in our corridor, and appreciably
                % slower than our cap.
                if along > 0 && along < 35 && lat < 2.0 && ...
                        tr.speed < 0.6 * obj.ctx.speedCap
                    blockerSlow = true;
                end

                % Nearest leader in our own corridor, for car following.
                if along > 0 && lat < 0.5 * (veh.width + prior.width) + 0.4
                    g = along - 0.5 * (veh.length + prior.length);
                    if g < leadGap
                        leadGap = g;
                        leadSpeed = tr.speed;
                    end
                end

                if ismember(tr.trackId, wwIds) && hypot(rel(1), rel(2)) < 45
                    wrongWayNear = true;
                end
            end

            % Mean risk along the corridor we intend to drive.
            % BL1 has no risk map at all - that absence is what makes it the
            % lane-follow baseline - so the term is simply absent for it
            % rather than faked with a zero that would read as "clear".
            corridorRisk = 0;
            if ~isempty(obj.riskMap) && ~isempty(obj.activePath)
                rv = obj.riskMap.sampleWorld(obj.activePath(:,1), obj.activePath(:,2));
                corridorRisk = mean(rv);
            end

            % Is there a clear alternative corridor to overtake into?  Sample
            % risk along a laterally offset lane ahead.  Offsetting to the
            % RIGHT of our heading means moving toward oncoming traffic under
            % keep-left, which is what an overtake on these roads involves.
            altClear = obj.alternativeCorridorClear(egoState);

            in = struct( ...
                'ttc', minTTC, ...
                'clearance', minClear, ...
                'mergeFlag', ~isempty(mgIds), ...
                'wrongWayNear', wrongWayNear, ...
                'pathBlocked', obj.lastBlocked, ...
                'altCorridorClear', altClear, ...
                'blockerSlow', blockerSlow, ...
                'corridorRisk', corridorRisk, ...
                'rejoinDone', false, ...
                'egoSpeed', egoState.v, ...
                'leadGap', leadGap, ...
                'leadSpeed', leadSpeed, ...
                'mergeTTC', mgTTC);
        end

        % ----------------------------------------------------------------
        function v = followingSpeed(obj, gap, leadV, egoState)
            %FOLLOWINGSPEED  Speed that keeps a safe gap behind a leader.
            %
            %   Two constraints, whichever is tighter:
            %     1. do not exceed the leader's speed once inside the desired
            %        following gap, so the gap stops shrinking;
            %     2. stay able to stop in the distance available, which is
            %        what matters when the leader is much slower or stopped.
            v = inf;
            if ~isfinite(gap) || ~isfinite(leadV)
                return
            end

            desired = obj.ctx.followingGap;
            stopGap = 2.0;

            if gap <= stopGap
                v = 0;
                return
            end

            % Stopping-distance bound: v^2 / (2a) <= usable gap.
            usable = gap - stopGap;
            vStop = sqrt(2 * obj.cfg.vehicle.comfortDecel * usable) + max(leadV, 0);

            if gap < desired
                % Inside the desired gap: match the leader, easing in.
                frac = (gap - stopGap) / max(desired - stopGap, 0.1);
                v = max(leadV, 0) + frac * max(egoState.v - leadV, 0) * 0.5;
            end

            v = min(v, vStop);
        end

        % ----------------------------------------------------------------
        function d = laneFollowPlan(obj, egoState, decision)
            %LANEFOLLOWPLAN  BL1: centreline tracking with IDM gap keeping.
            %
            %   The lateral path is the road centreline, handed to this
            %   baseline directly.  The proposed planner never receives it -
            %   that asymmetry is deliberate and generous to the baseline, and
            %   it is recorded in ARCHITECTURE.md so the comparison cannot be
            %   read as rigged in the proposed system's favour.
            %
            %   Longitudinally it is a plain IDM-style law on the nearest
            %   TRACKED obstacle inside a narrow corridor.  It uses tracks,
            %   not ground truth, so it faces the same perception it would in
            %   service.
            obj.activePath = obj.scenario.refPath;

            heading = [cos(egoState.yaw), sin(egoState.yaw)];
            normal = [-heading(2), heading(1)];
            corridorHalf = 1.2;

            gap = inf;
            leadV = inf;
            for k = 1:numel(obj.lastTracks)
                tr = obj.lastTracks(k);
                rel = [tr.x - egoState.x, tr.y - egoState.y];
                along = rel * heading.';
                lat = abs(rel * normal.');
                if along <= 0 || lat > corridorHalf
                    continue
                end
                prior = classPriors(tr.classId);
                g = along - 0.5 * (obj.cfg.vehicle.length + prior.length);
                if g < gap
                    gap = g;
                    leadV = tr.speed;
                end
            end

            vCap = decision.speedCap;
            if isfinite(gap)
                % Intelligent-driver-style: desired gap grows with speed.
                sStar = 2.0 + max(egoState.v * obj.ctx.followingGap / ...
                    max(vCap, 0.1), 0);
                if gap < 1.0
                    vT = 0;
                elseif gap < sStar
                    vT = min(vCap, max(leadV, 0) * gap / max(sStar, 0.1));
                else
                    vT = vCap;
                end
            else
                vT = vCap;
            end

            d = struct('traj', [], 'candidates', {{}}, 'cost', 0, 'terms', [], ...
                'feasible', true, 'blocked', false, 'nRejected', 0, ...
                'chosenV', vT, 'chosenKappa', 0);
        end

        % ----------------------------------------------------------------
        function g = localGoal(obj, egoState)
            %LOCALGOAL  A waypoint ALONG THE ROAD inside the planning window.
            %
            %   The route goal is up to 250 m away, far outside the 70 m
            %   window.  Clamping it to the window boundary in a straight line
            %   fails as soon as the road curves: the clamped point lands off
            %   the carriageway, and the planner reports "no reachable goal
            %   cell" - measured 474 times in one run, which is most of why
            %   the ego crawled.
            %
            %   Following the road's own arc length instead keeps the local
            %   goal on the drivable surface whatever the geometry does.  This
            %   is not lane following: the goal is a point to reach, and the
            %   planner remains free to route around it however the risk field
            %   dictates.
            lookahead = 0.75 * obj.cfg.risk.aheadM;

            info = obj.scenario.road.nearest(egoState.x, egoState.y);
            seg = max(info.seg, 1);
            sGoal = min(info.s + lookahead, obj.scenario.ego.goalS);

            xy = obj.scenario.road.pointAt(seg, sGoal, obj.scenario.ego.laneOffset);
            g = [xy(1), xy(2)];

            % Near the end of the route, aim at the actual goal marker.
            if obj.scenario.ego.goalS - info.s < lookahead
                g = obj.scenario.ego.goal;
            end
        end

        % ----------------------------------------------------------------
        function tf = alternativeCorridorClear(obj, egoState)
            %ALTERNATIVECORRIDORCLEAR  Risk along an offset overtaking lane.
            tf = false;
            if isempty(obj.riskMap)
                return      % no risk map (BL1): never authorise an overtake
            end
            lateral = 2.6;      % m to the right of the current heading
            ahead = linspace(4, 30, 14)';
            c = cos(egoState.yaw); s = sin(egoState.yaw);

            xs = egoState.x + ahead * c - (-lateral) * s;
            ys = egoState.y + ahead * s + (-lateral) * c;

            % Ask whether the corridor is PHYSICALLY free, not whether it is
            % the side we would prefer.  Overtaking on an unmarked two-way
            % road means using the oncoming half; judging that corridor
            % against the wrong-side preference would refuse every overtake
            % by construction.
            rv = obj.riskMap.sampleWorldNoSide(xs, ys);
            onRoad = obj.scenario.road.isDrivable(xs, ys);

            tf = all(onRoad) && max(rv) < 0.35;
        end

        % ----------------------------------------------------------------
        function reason = replanTrigger(obj, t, decision)
            %REPLANTRIGGER  Periodic plus event-driven global replanning.
            reason = '';

            if isempty(obj.globalPath)
                reason = 'no path';
                return
            end
            if t - obj.lastReplanTime >= obj.ctx.replanPeriod
                reason = 'periodic';
                return
            end
            if obj.lastBlocked
                reason = 'path blocked';
                return
            end
            if ~isempty(obj.lastDecision) && ...
                    ~strcmp(decision.stateName, obj.lastDecision.stateName) && ...
                    ~strcmp(decision.stateName, 'CRUISE')
                reason = sprintf('FSM entered %s', decision.stateName);
                return
            end
        end

        % ----------------------------------------------------------------
        function m = riskMeta(obj)
            %RISKMETA  Everything needed to place the quantised field in world
            %   coordinates on replay, without storing the field twice.
            m = struct('res', obj.riskMap.res, 'originXY', obj.riskMap.originXY, ...
                'yaw', obj.riskMap.yaw, 'nx', obj.riskMap.nx, 'ny', obj.riskMap.ny, ...
                'xe', obj.riskMap.xe([1 end]), 'ye', obj.riskMap.ye([1 end]));
        end

        % ----------------------------------------------------------------
        function truths = agentTruths(obj)
            agents = obj.scenario.agents;
            if isempty(agents)
                truths = struct([]);
                return
            end
            cells = cell(1, numel(agents));
            for i = 1:numel(agents)
                cells{i} = agents(i).truth();
            end
            truths = [cells{:}];
        end

        % ----------------------------------------------------------------
        function world = buildWorld(obj, egoState, truths)
            %BUILDWORLD  Interaction table the agents use for gap keeping.
            egoBox = obj.vehicle.footprint(egoState);
            info = obj.scenario.road.nearest(egoBox.x, egoBox.y);

            egoEnt = struct('id', 0, 'seg', info.seg, 's', info.s, 'd', info.d, ...
                'L', obj.cfg.vehicle.length, 'W', obj.cfg.vehicle.width, ...
                'isEgo', true, 'active', true);

            agents = obj.scenario.agents;
            ents = cell(1, numel(agents) + 1);
            ents{1} = egoEnt;
            for i = 1:numel(agents)
                ents{i+1} = agents(i).entity();
            end

            world = struct();
            world.entities = [ents{:}];
            world.ego = egoState;
            world.road = obj.scenario.road;
            world.hazards = obj.scenario.hazards;
            world.truths = truths;
        end

        % ----------------------------------------------------------------
        function syncContainer(obj, egoState, truths)
            %SYNCCONTAINER  Mirror poses into the drivingScenario container.
            %
            %   Only needed so the toolbox sensor models have something to
            %   look at (M2).  Yaw crosses into DEGREES here - the container
            %   is the only place in the project that uses them.
            sc = obj.scenario.scenarioObj;
            if isempty(sc)
                return
            end

            % Position must be the actor's ROTATIONAL centre:
            %   Position = bodyCentre + R(yaw) * OriginOffset
            % Writing a body centre straight in displaces every vehicle by its
            % rear-axle offset (D19).
            egoBox = obj.vehicle.footprint(egoState);
            eo = obj.scenario.egoOriginOffset;
            obj.scenario.egoActor.Position = ...
                [egoBox.x + eo * cos(egoState.yaw), egoBox.y + eo * sin(egoState.yaw), 0];
            obj.scenario.egoActor.Yaw = rad2deg(egoState.yaw);
            obj.scenario.egoActor.Velocity = ...
                [egoState.v * cos(egoState.yaw), egoState.v * sin(egoState.yaw), 0];

            % An agent that has left the modelled stretch must leave PERCEPTION
            % too.  A drivingScenario actor cannot be removed mid-run, and
            % simply not updating it leaves it frozen wherever it stopped being
            % simulated - where the sensors go on detecting it.  Four highway
            % agents ran off the end of the ribbon within a few metres of each
            % other and sat 16 m in front of the ego as a stack of coincident
            % targets: the tracker spawned ~30 phantom tracks on them, the risk
            % map painted the corridor solid, and the ego stopped at 384 m of
            % 392 and never resumed.  That was the highway stall.
            %
            % Parking them 1e4 m away is the toolbox idiom for "not present":
            % the longest sensor here reaches 150 m, so they are out of range of
            % everything, by four orders of magnitude, and cannot come back.
            PARK = 1e4;

            acts = obj.scenario.agentActors;
            offs = obj.scenario.actorOriginOffset;
            for i = 1:min(numel(acts), numel(truths))
                if ~truths(i).active
                    acts(i).Position = [PARK, PARK, 0];
                    acts(i).Velocity = [0, 0, 0];
                    continue
                end
                o = offs(i);
                acts(i).Position = [truths(i).x + o * cos(truths(i).yaw), ...
                                    truths(i).y + o * sin(truths(i).yaw), 0];
                acts(i).Yaw = rad2deg(truths(i).yaw);
                acts(i).Velocity = [truths(i).vx, truths(i).vy, 0];
            end
        end
    end
end

% ========================================================================
function h = headingOrZero(tr)
%HEADINGORZERO  A track's heading, or its velocity direction, or 0.
%   A stationary track has no observable heading; falling back to 0 keeps the
%   footprint axis-aligned rather than inventing an orientation.
if isfinite(tr.heading)
    h = tr.heading;
elseif hypot(tr.vx, tr.vy) > 1e-6
    h = atan2(tr.vy, tr.vx);
else
    h = 0;
end
end
