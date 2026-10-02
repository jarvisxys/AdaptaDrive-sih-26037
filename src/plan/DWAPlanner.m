classdef DWAPlanner < handle
    %DWAPLANNER  Dynamic-window local planner (Section 5.10).
    %
    %   Samples terminal speeds and curvatures the vehicle can actually reach,
    %   rolls each out for 2.5 s with the same bicycle model the vehicle uses,
    %   and scores with FIVE NORMALISED TERMS whose weights sum to 1:
    %
    %     cRisk   = mean R along the rollout                          [0,1]
    %     cClear  = clamp((dSafe - dMin)/dSafe, 0, 1)                 [0,1]
    %     cGoal   = 0.5*min(lateralDev/1.5, 1)
    %             + 0.5*(1 - progress/(vCap*Troll))                   [0,1]
    %     cSmooth = min(|dKappa|/dKappaMax, 1)                        [0,1]
    %     cSpeed  = (vCap - vEnd)/vCap                                [0,1]
    %
    %   WHY EVERY TERM IS BOUNDED, and why this is not cosmetic.  Four separate
    %   defects in this planner had the same shape: a term left in its natural
    %   units inside a weighted sum, quietly overriding every other term.
    %     - 1/clearance was unbounded; a rollout grazing risk 0.99 scored 99,
    %       more than any achievable progress, so standing still won.
    %     - the goal term was in metres (up to ~17) against terms of order 1,
    %       so progress outweighed risk and the ego cut corners off the road.
    %     - the smoothness term contained 0.5*|v - v0|, which reached 2.5 once
    %       the window widened; on a COMPLETELY EMPTY road the cost minimum sat
    %       below the current speed, so the vehicle coasted down and never
    %       recovered.
    %     - the window itself was one cycle wide (+-0.25 m/s), making speed
    %       differences worth 0.03 against safety terms worth up to 4.8.
    %   None of these raised an error. Each produced a plausible-looking
    %   vehicle that was simply wrong, and each took a full diagnostic run to
    %   find. Bounding every term and making the weights a partition of 1 is
    %   what makes the weights in contextParams mean what they say.
    %
    %   HARD REJECTS (a rollout is discarded outright, never merely penalised)
    %     1. peak R >= cfg.plan.rejectRisk  (0.85)
    %     2. predicted footprint overlap with any mode above the probability floor
    %     3. body centre leaves the drivable area
    %   (3) is redundant with (1) while the edge layer weighs 1.0 - off-road is
    %   R = 1 - but it is kept explicit so that the guarantee "the planner never
    %   proposes leaving the road" does not silently depend on a tunable risk
    %   weight. It is the same definition the termination check uses.
    %
    %   If EVERY rollout is rejected, the planner reports BLOCKED and returns no
    %   trajectory. It never picks the least-bad rejected candidate: the whole
    %   point of a hard constraint is that violating it is not on the menu. The
    %   FSM turns BLOCKED into STOP or EMERGENCY_BRAKE, and the controller keeps
    %   steering along the last trajectory that WAS valid while it brakes.
    %
    %   This is where the continuous risk field earns its keep. Hybrid A* sees
    %   only free/blocked (see GlobalPlanner), so the difference between risk
    %   0.2 and 0.55 exists solely in cRisk here.
    %
    %   See also GLOBALPLANNER, RISKMAP, CONTEXTPARAMS, KINEMATICBICYCLE.

    properties (Constant)
        % Below this a candidate is standing still rather than creeping.  Well
        % under BehaviorFSM.StoppedSpeed: this asks "did the planner choose to
        % move at all", which is a smaller question than "is the vehicle under
        % way", and the two are used for different halves of the anti-stall rule.
        MovingSpeed = 0.05      % m/s
    end

    properties (SetAccess = private)
        cfg
        road            % for the explicit drivable-area reject
        K               % rollout steps
        skipSteps = 2   % steps exempt from the overlap reject (see collides)
        dtRoll
        lastCandidates  % for the UI fan
        lastBest
        lastCost
        stoppedSince = NaN    % sim time the current involuntary stop began
        breakingStall = false % latched while overriding a deadlocked stop
    end

    methods
        function obj = DWAPlanner(cfg, road)
            obj.cfg = cfg;
            if nargin >= 2
                obj.road = road;
            end
            obj.dtRoll = cfg.plan.dwaDt;
            obj.K = round(cfg.plan.dwaHorizon / obj.dtRoll);
            obj.lastCandidates = {};
        end

        % ----------------------------------------------------------------
        function out = plan(obj, egoState, riskMap, globalPath, preds, decision, ctx, goalXY)
            %PLAN  Choose a local trajectory.
            %
            %   OUT fields
            %       traj        K x 4 [x y yaw v] in WORLD coordinates ([] if blocked)
            %       candidates  cell of K x 2 rollouts, for the UI fan
            %       cost        cost of the chosen trajectory
            %       terms       its five normalised cost terms
            %       feasible    logical
            %       blocked     logical: nothing survived the hard rejects
            %       nRejected   how many candidates were rejected

            out = struct('traj', [], 'candidates', {{}}, 'cost', inf, ...
                'terms', [], 'feasible', false, 'blocked', true, ...
                'nRejected', 0, 'chosenV', 0, 'chosenKappa', 0, ...
                'antiStall', false, 'escape', false);

            if nargin < 8, goalXY = []; end
            speedCap = max(decision.speedCap, 0);
            [vs, kappas, kappa0, dKappaMax] = obj.dynamicWindow(egoState, speedCap);
            obst = obj.flattenPredictions(preds);

            best = []; bestCost = inf; bestTerms = [];
            bestV = 0; bestK = 0;

            % Cheapest candidate that actually MOVES, tracked separately.  See
            % the anti-stall rule below for why standing still cannot be left
            % to win on cost alone.
            mvBest = []; mvCost = inf; mvTerms = [];
            mvV = 0; mvK = 0;

            % Least-bad REJECTED candidate, by peak risk then by speed.  Used
            % only to escape a state the ego is already standing in.
            escBest = []; escPeak = inf; escMean = inf; escV = inf; escK = 0;

            % Risk where the ego IS, at the same body-centre reference REJECTED
            % samples.  This is the yardstick the escape rule compares against:
            % an escape must be strictly better than staying put.
            offB = obj.cfg.vehicle.length / 2 - obj.cfg.vehicle.rearOverhang;
            holdPeak = riskMap.sampleWorld( ...
                egoState.x + offB * cos(egoState.yaw), ...
                egoState.y + offB * sin(egoState.yaw));

            nCand = numel(vs) * numel(kappas);
            cands = cell(1, nCand);
            n = 0; nRej = 0;

            for iv = 1:numel(vs)
                for ik = 1:numel(kappas)
                    traj = obj.rollout(egoState, vs(iv), kappas(ik));
                    n = n + 1;
                    cands{n} = traj(:, 1:2);

                    [rej, rv] = obj.rejected(traj, riskMap, obst);
                    if rej
                        nRej = nRej + 1;
                        % Remember the least-bad rejected candidate in case the
                        % ego is ALREADY in violation - see the escape rule.
                        % Ordered by peak risk, then MEAN risk, then speed.  The
                        % mean is what makes this usable where it is needed: deep
                        % in an off-road region every peak saturates at 1.0, so
                        % peak alone gives no gradient and the vehicle would sit
                        % in the violation it is supposed to be leaving.  The
                        % mean still points downhill.
                        pk = max(rv);
                        mn = mean(rv);
                        better = pk < escPeak || ...
                            (pk == escPeak && mn < escMean) || ...
                            (pk == escPeak && mn == escMean && vs(iv) < escV);
                        if better
                            escPeak = pk;
                            escMean = mn;
                            escBest = traj;
                            escV = vs(iv);
                            escK = kappas(ik);
                        end
                        continue
                    end

                    terms = obj.costTerms(traj, rv, kappas(ik), kappa0, dKappaMax, ...
                        obst, globalPath, egoState, speedCap, goalXY);
                    c = obj.combine(terms, ctx.dwaWeights);

                    if c < bestCost
                        bestCost = c;
                        best = traj;
                        bestTerms = terms;
                        bestV = vs(iv);
                        bestK = kappas(ik);
                    end
                    if vs(iv) > obj.MovingSpeed && c < mvCost
                        mvCost = c;
                        mvBest = traj;
                        mvTerms = terms;
                        mvV = vs(iv);
                        mvK = kappas(ik);
                    end
                end
            end

            out.candidates = cands(1:n);
            out.nRejected = nRej;
            obj.lastCandidates = out.candidates;

            if isempty(best)
                % Every candidate violated a hard constraint.  Report BLOCKED:
                % the least-bad violation of a hard constraint is still a
                % violation, and the FSM turns this into STOP or
                % EMERGENCY_BRAKE while the controller brakes along the last
                % trajectory that was valid.
                out.blocked = true;

                % ESCAPE.  There is one case where refusing to move is not the
                % safe answer, and it is the case where the ego's OWN footprint
                % is already on a cell at or above rejectRisk.  Then even the
                % zero-speed candidate is rejected, so "every candidate was
                % rejected" is not information about where the ego may go - it
                % is information about where the ego already is.  Holding
                % position preserves the violation instead of ending it, the FSM
                % latches STOP, and nothing can ever change: measured on village
                % seed 1, the ego sat at (95.0, 1.5) in STOP with 77 of 77
                % candidates rejected from t = 55 s to the timeout at 177.5 s.
                %
                % The rejection rule governs ENTERING a violating state.  It
                % cannot also govern LEAVING one, so leaving needs its own rule:
                % take the candidate with the lowest peak risk, provided it is
                % strictly lower than what the ego is already sitting in, and
                % slowest first among equals.  This never lets the planner
                % choose risk it could have avoided - it only applies when every
                % alternative, including standing still, is already rejected.
                % Strictly better than holding, on peak OR on mean.  Holding is
                % a single point, so its peak and mean are the same number.
                escapeHelps = escPeak < holdPeak || ...
                    (escPeak <= holdPeak && escMean < holdPeak);
                if ~isempty(escBest) && escapeHelps && escV > obj.MovingSpeed
                    out.traj = escBest;
                    out.cost = inf;            % no cost is claimed for an escape
                    out.terms = [];
                    out.feasible = true;
                    out.chosenV = min(escV, obj.cfg.plan.escapeSpeed);
                    out.chosenKappa = escK;
                    out.escape = true;
                    obj.lastBest = escBest;
                    return
                end

                obj.lastBest = [];
                return
            end

            % ---- anti-stall: stopping is the FSM's decision, not a side
            %      effect of the cost function -------------------------------
            %
            % Standing still occupies one cell, and if that cell is clear its
            % risk term is exactly 0.  Any trajectory that moves enters cells
            % further out and scores above 0.  So whenever
            %
            %     w_risk * R(moving)  >  w_speed * (speed gain)
            %                            + w_goal * (progress gain)
            %
            % the cheapest candidate is v = 0 - and because the ego then does
            % not move, the same comparison holds on the next cycle, and every
            % cycle after it.  The vehicle is deadlocked by arithmetic, on a
            % road it could drive.
            %
            % Measured on village seed 1 at t = 100.1 s, stopped 30 m from the
            % goal with a static hazard 3.5 m ahead and a 1.1 m gap beside it:
            %
            %     v = 0.00  cost 0.3668   (risk 0.000, speed 1.000)
            %     v = 0.42  cost 0.3853   (risk 0.150, speed 0.940)
            %
            % The ego held that position until the run timed out.
            %
            % Re-weighting the terms cannot fix this in general: for ANY
            % positive risk weight there is a risk level at which paralysis
            % wins, and lowering it far enough to prevent that would stop the
            % risk field mattering, which is the whole contribution.  The fix
            % is architectural instead.  The behaviour layer decides WHETHER to
            % proceed and expresses it as speedCap; STOP and EMERGENCY_BRAKE
            % set speedCap = 0.  A non-zero cap is therefore an explicit
            % decision to move, and the local planner's job is to choose HOW,
            % not to quietly veto it on cost.  So: when the FSM has decided to
            % proceed and a feasible moving trajectory exists, take the
            % cheapest moving one.
            %
            % This never overrides a hard constraint.  mvBest passed the same
            % rejection test as every other candidate; if nothing moving
            % survived, mvBest is empty and the vehicle still stops.
            %
            % It is also not immediate, and that distinction matters.  Waiting
            % is often exactly right: in the cattle scenario the ego stops for a
            % cow, the cow wanders off, and the ego proceeds.  Forcing motion the
            % instant v = 0 wins on cost made the ego creep at the cow until the
            % FSM escalated to STOP, and that run went from reaching the goal to
            % timing out at 146 m.  Waiting is allowed; waiting FOREVER is not.
            % So the override arms only after the vehicle has been stationary
            % for stallBreakTime while the FSM was asking it to proceed - by
            % which point whatever it was waiting for is not going to clear.
            % Once armed the override LATCHES until the vehicle is genuinely
            % rolling.  Firing for a single cycle achieves nothing: a 0.42 m/s
            % target held for one 0.1 s cycle moves the ego a few centimetres,
            % v = 0 wins again immediately, and the vehicle ratchets forward
            % about a metre every stallBreakTime.  Village advanced 0.9 m in
            % 77 s that way before timing out.  The latch clears at the FSM's
            % own StoppedSpeed, so both layers agree on when the stop is over.
            wantsToGo = speedCap > obj.MovingSpeed;
            egoStopped = egoState.v <= BehaviorFSM.StoppedSpeed;

            if ~wantsToGo || ~egoStopped
                obj.stoppedSince = NaN;
                obj.breakingStall = false;
            elseif bestV <= obj.MovingSpeed || obj.breakingStall
                if isnan(obj.stoppedSince)
                    obj.stoppedSince = egoState.t;
                end
                if egoState.t - obj.stoppedSince >= obj.cfg.plan.stallBreakTime
                    obj.breakingStall = true;
                end
            end

            stalled = false;
            if obj.breakingStall && ~isempty(mvBest) && bestV <= mvV
                best = mvBest;
                bestCost = mvCost;
                bestTerms = mvTerms;
                bestV = mvV;
                bestK = mvK;
                stalled = true;
            end

            out.traj = best;
            out.cost = bestCost;
            out.terms = bestTerms;
            out.feasible = true;
            out.blocked = false;
            out.chosenV = bestV;
            out.chosenKappa = bestK;
            out.antiStall = stalled;
            obj.lastBest = best;
            obj.lastCost = bestCost;
        end

        % ----------------------------------------------------------------
        function [vs, kappas, kappa0, dKappaMax] = dynamicWindow(obj, egoState, speedCap)
            %DYNAMICWINDOW  Terminal speeds and curvatures reachable in 1 s.
            %
            %   The window spans what the vehicle can reach over dwaWindow
            %   seconds, and the chosen speed is handed to the controller as a
            %   TARGET rather than as the next instant's speed.  A one-cycle
            %   window makes every candidate nearly identical in speed, and
            %   then any term with a slope - clearance and risk both have one -
            %   decides the speed instead of the speed terms.

            W = obj.cfg.plan.dwaWindow;
            v0 = egoState.v;

            vMin = max(0, v0 - obj.cfg.plan.dwaDecel * W);
            vMax = min(speedCap, v0 + obj.cfg.plan.dwaAccel * W);
            if vMax < vMin
                vMax = vMin;
            end
            vs = linspace(vMin, vMax, obj.cfg.plan.dwaNv);

            L = obj.cfg.vehicle.wheelbase;
            steerMax = min(obj.cfg.vehicle.maxSteer, ...
                abs(egoState.steer) + obj.cfg.vehicle.maxSteerRate * W);
            kSteer = tan(steerMax) / L;

            % Physical limit as well as a steering limit.  A dynamic window is
            % the set of FEASIBLE commands, and a curvature the tyres cannot
            % hold is not feasible: at 11 m/s the steering limit alone would
            % admit 26 m/s^2 of lateral acceleration.  Capping here keeps it
            % in the window definition rather than adding another reject.
            kLat = obj.cfg.plan.maxLateralAccel / max(v0, 1.0)^2;
            kMax = min(kSteer, kLat);

            kappas = linspace(-kMax, kMax, obj.cfg.plan.dwaNk);
            kappa0 = tan(egoState.steer) / L;
            kappa0 = min(max(kappa0, -kMax), kMax);
            dKappaMax = max(2 * kMax, 1e-6);
        end

        % ----------------------------------------------------------------
        function traj = rollout(obj, egoState, vTarget, kappa)
            %ROLLOUT  Forward-simulate the bicycle model toward a speed target.
            %
            %   The speed RAMPS toward vTarget under the real acceleration
            %   limits rather than jumping to it, so a rollout describes a
            %   trajectory the vehicle can actually execute.  Snapping to the
            %   target would overstate the progress of every accelerating
            %   candidate and understate the stopping distance of every braking
            %   one - exactly the two cases the choice turns on.
            x = egoState.x; y = egoState.y; yaw = egoState.yaw;
            v = egoState.v;

            aUp = obj.cfg.vehicle.maxAccel;
            aDn = obj.cfg.vehicle.maxDecel;
            dt = obj.dtRoll;

            traj = zeros(obj.K, 4);
            for k = 1:obj.K
                dv = vTarget - v;
                v = max(v + max(min(dv, aUp * dt), -aDn * dt), 0);

                % Midpoint integration: at 0.1 s steps plain Euler visibly
                % bends the heading on tight turns, which would make tight
                % rollouts look feasible when they are not.
                yawMid = yaw + 0.5 * dt * v * kappa;
                x = x + v * cos(yawMid) * dt;
                y = y + v * sin(yawMid) * dt;
                yaw = yaw + dt * v * kappa;
                traj(k, :) = [x, y, yaw, v];
            end
        end

        % ----------------------------------------------------------------
        function terms = costTerms(obj, traj, rv, kappa, kappa0, dKappaMax, ...
                obst, globalPath, egoState, speedCap, goalXY)
            %COSTTERMS  The five normalised cost terms, each in [0,1].
            %
            %   Public so the tests can assert the bound directly rather than
            %   inferring it from behaviour - the bound is the property that
            %   four separate defects violated.

            % --- risk ----------------------------------------------------
            terms.risk = clamp01(mean(rv));

            % --- clearance ------------------------------------------------
            terms.clear = obj.clearancePenalty(traj, obst);

            % --- goal: lateral deviation and progress shortfall ------------
            Troll = obj.cfg.plan.dwaHorizon;
            maxProgress = max(speedCap * Troll, 1e-6);

            % PROGRESS = how much closer to the goal this rollout gets us.
            %
            % Measured directly, not as arc length between the nearest path
            % indices.  The index version silently stops discriminating
            % whenever the ego sits at the end of a short global path: every
            % candidate then maps to the same index, progress is identical for
            % all of them, and the choice falls to whatever term is left.
            % Measured consequence: on a completely clear road with every
            % agent gone, TTC 10 s and clearance 10 m, the planner selected
            % v = 0 and the vehicle stood still for the rest of the run.
            %
            % Closing distance to the goal is always defined, always
            % monotonic in useful motion, and independent of how whichever
            % planner happened to sample the path.
            if isempty(goalXY)
                progress = hypot(traj(end,1) - egoState.x, traj(end,2) - egoState.y);
            else
                dNow = hypot(egoState.x - goalXY(1), egoState.y - goalXY(2));
                dEnd = hypot(traj(end,1) - goalXY(1), traj(end,2) - goalXY(2));
                progress = dNow - dEnd;
            end

            % Lateral deviation is still measured against the global path,
            % which is what keeps the vehicle in the corridor the global
            % planner chose rather than cutting straight at the goal.
            lateralDev = 0;
            if ~isempty(globalPath) && size(globalPath, 1) >= 2
                % Ignore a global path the vehicle is no longer ON.  A stale
                % path - one the ego has drifted away from, or one left over
                % from a replan that has since failed - drags the deviation
                % term toward wherever it happens to lie, and the cheapest way
                % to avoid deviating from it is to not move at all.
                %
                % Measured: with every agent gone, TTC 10 s, clearance 10 m
                % and CRUISE at a 6.94 m/s cap, the ego chose v = 0 and stood
                % still. The identical DWA with NO global path (BL2) drove the
                % same scenario at 5.50 m/s. The path was the difference.
                dEgo = min(hypot(globalPath(:,1) - egoState.x, ...
                                 globalPath(:,2) - egoState.y));
                if dEgo <= obj.cfg.plan.pathValidRadius
                    endPt = traj(end, 1:2);
                    lateralDev = min(hypot(globalPath(:,1) - endPt(1), ...
                                           globalPath(:,2) - endPt(2)));
                end
            end

            terms.goal = 0.5 * clamp01(lateralDev / obj.cfg.plan.dwaLateralScale) + ...
                         0.5 * (1 - clamp01(progress / maxProgress));

            % --- smoothness: curvature change only -------------------------
            % Longitudinal comfort is already enforced twice, by the vehicle's
            % acceleration limits and by the jerk limiter in SpeedController.
            % Penalising it a third time here only fights the speed term.
            terms.smooth = clamp01(abs(kappa - kappa0) / dKappaMax);

            % --- speed ------------------------------------------------------
            vEnd = traj(end, 4);
            terms.speed = clamp01((speedCap - vEnd) / max(speedCap, 1e-6));
        end

        % ----------------------------------------------------------------
        function c = combine(~, terms, w)
            %COMBINE  Weighted sum.  Weights are a partition of 1 (contextParams).
            c = w.goal   * terms.goal + ...
                w.risk   * terms.risk + ...
                w.clear  * terms.clear + ...
                w.smooth * terms.smooth + ...
                w.speed  * terms.speed;
        end
    end

    % ====================================================================
    methods (Access = private)

        function [rej, rv] = rejected(obj, traj, riskMap, obst)
            %REJECTED  The hard constraints, cheapest test first.

            % Risk is sampled at the BODY CENTRE, the same reference point the
            % termination check and the metrics use.
            off = obj.cfg.vehicle.length / 2 - obj.cfg.vehicle.rearOverhang;
            yaw = traj(:, 3);
            bx = traj(:, 1) + off * cos(yaw);
            by = traj(:, 2) + off * sin(yaw);

            rv = riskMap.sampleWorld(bx, by);

            rej = true;
            if max(rv) >= obj.cfg.plan.rejectRisk
                return
            end
            if ~isempty(obj.road) && ~all(obj.road.isDrivable(bx, by))
                return
            end
            if obj.collides(traj, obst)
                return
            end
            rej = false;
        end

        % ----------------------------------------------------------------
        function obst = flattenPredictions(obj, preds)
            %FLATTENPREDICTIONS  Modes above the probability floor, as arrays.
            obst = struct('a', {}, 'b', {}, 'radius', {}, 'prob', {});
            minP = obj.cfg.plan.dwaMinModeProb;

            for i = 1:numel(preds)
                prior = classPriors(preds(i).classId);
                halfLen = prior.length / 2;
                rAgent = prior.width / 2;

                for k = 1:numel(preds(i).modes)
                    m = preds(i).modes(k);
                    if m.prob < minP
                        continue    % too unlikely to be a hard constraint
                    end

                    % Body capsule: the agent's centreline, placed along its
                    % predicted heading, swollen by its half-width.
                    h = m.heading;
                    if ~isfield(m, 'heading') || all(isnan(h))
                        % Never moves and no heading is known.  Fall back to a
                        % disc that covers the body in any orientation -
                        % conservative, which is the right way to be wrong
                        % about something whose orientation is unknown.
                        a = m.mu;
                        b = m.mu;
                        r = 0.5 * hypot(prior.length, prior.width);
                    else
                        h(isnan(h)) = 0;
                        off = halfLen * [cos(h), sin(h)];
                        a = m.mu - off;
                        b = m.mu + off;
                        r = rAgent;
                    end

                    obst(end+1) = struct('a', a, 'b', b, 'radius', r, ...
                        'prob', m.prob); %#ok<AGROW>
                end
            end
        end

        % ----------------------------------------------------------------
        function hit = collides(obj, traj, obst)
            %COLLIDES  Time-indexed overlap against predicted footprints.
            %
            %   The ego is three circles along its length, not one bounding
            %   circle: the bounding circle of a 4.5 x 1.8 m car has radius
            %   2.42 m, which claims the car is 4.8 m WIDE.  On a 6 m road that
            %   rejects nearly every rollout whenever a pedestrian is near, and
            %   since a stationary vehicle's rollouts all sit on one point, the
            %   rejection never clears and the ego deadlocks permanently.
            %
            %   The first few steps are exempt.  A track whose position error
            %   places it on top of the stationary ego would otherwise veto
            %   every candidate including "stay exactly where you are".  You
            %   cannot steer away from something you are already beside; what
            %   you can do is not drive further INTO it.
            hit = false;
            if isempty(obst)
                return
            end

            [ea, eb, rEgo] = obj.egoCapsule(traj);
            k0 = obj.skipSteps + 1;

            for i = 1:numel(obst)
                n = min(size(obst(i).a, 1), size(ea, 1));
                if n < k0
                    continue
                end
                gap = capsuleGap(ea(k0:n,:), eb(k0:n,:), ...
                                 obst(i).a(k0:n,:), obst(i).b(k0:n,:), ...
                                 rEgo + obst(i).radius);
                if any(gap < 0)
                    hit = true;
                    return
                end
            end
        end

        % ----------------------------------------------------------------
        function p = clearancePenalty(obj, traj, obst)
            %CLEARANCEPENALTY  Proximity cost in [0,1], WEIGHTED BY MODE PROBABILITY.
            %
            %   penalty = max over modes of  prob * clamp((dSafe - gap)/dSafe)
            %
            %   The probability weighting is the point.  A cow's "steps into
            %   the road" mode carries probability 0.15, and without weighting
            %   it produced exactly the same full penalty as a certainty: the
            %   term saturated for every candidate that moved at all, leaving
            %   no gradient to distinguish "ease off" from "crawl".  Measured:
            %   10.5 m/s on an empty road but 2.1 m/s on the same road with
            %   one cow grazing on the verge, and the ego still collided
            %   because crawling made the event arrive later, not safer.
            %
            %   A 15%-likely hazard should cost about a sixth of a certain
            %   one. The HARD reject is unchanged and still treats any mode
            %   above the probability floor as a constraint - unlikely is not
            %   the same as ignorable, it is the same as cheaper.
            p = 0;
            if isempty(obst)
                return
            end

            dSafe = obj.cfg.plan.dwaSafeGap;
            [ea, eb, rEgo] = obj.egoCapsule(traj);

            for i = 1:numel(obst)
                n = min(size(obst(i).a, 1), size(ea, 1));
                if n < 1
                    continue
                end
                gap = max(min(capsuleGap(ea(1:n,:), eb(1:n,:), ...
                    obst(i).a(1:n,:), obst(i).b(1:n,:), ...
                    rEgo + obst(i).radius)), 0);
                p = max(p, obst(i).prob * clamp01((dSafe - gap) / dSafe));
            end
            p = clamp01(p);
        end

        % ----------------------------------------------------------------
        function [a, b, r] = egoCapsule(obj, traj)
            %EGOCAPSULE  The ego body as a capsule: centreline plus half-width.
            %
            %   Exactly contains the L x W rectangle, rounding only its
            %   corners, so it is tight where a bounding circle is not: the
            %   circle for a 4.5 x 1.8 m car has radius 2.42 m and asserts the
            %   car is 4.8 m wide, which on a 6 m road makes every oncoming
            %   vehicle read as a collision.
            L = obj.cfg.vehicle.length;
            r = obj.cfg.vehicle.width / 2;

            yaw = traj(:, 3);
            off = L / 2 - obj.cfg.vehicle.rearOverhang;   % rear axle -> centre
            bx = traj(:, 1) + off * cos(yaw);
            by = traj(:, 2) + off * sin(yaw);

            half = (L / 2) * [cos(yaw), sin(yaw)];
            a = [bx, by] - half;
            b = [bx, by] + half;
        end
    end
end

% ========================================================================
function v = clamp01(v)
v = min(max(v, 0), 1);
end
