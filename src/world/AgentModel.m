classdef AgentModel < handle
    %AGENTMODEL  One simulated road user with class-specific behaviour.
    %
    %   Agents are parameterised ON THE ROAD, not in world coordinates: the
    %   state is (segment, arc length s, lateral offset d, direction).  World
    %   position follows from RoadModel.pointAt.  Two things fall out of that
    %   choice: agents stay on the carriageway of a curved road without any
    %   path-tracking controller of their own, and "lateral drift" and
    %   "crossing" become one-line statements about d rather than geometry.
    %
    %   BEHAVIOUR BY CLASS (Section 5.1)
    %     car / bus     follow the road, mild drift, brake for what is ahead
    %     auto          larger drift (0.3-0.8 m), can merge informally
    %     two-wheeler   largest lateral variance, filters through gaps
    %     pedestrian    walks the edge, then crosses on a seeded trigger
    %     pushcart      very slow, long dwells, occupies the edge
    %     cow           near-stationary grazing, then an abrupt entry
    %
    %   WHO YIELDS TO THE EGO
    %   Vehicle classes brake for whatever is directly ahead in their own
    %   corridor, including the ego - that is ordinary traffic behaviour and
    %   without it agents would drive through each other.  Pedestrians, carts
    %   and cows do NOT defer to the ego: they are the hazard the planner
    %   exists to handle, and making them polite would quietly convert every
    %   avoidance test into a test of someone else's caution.  This asymmetry
    %   is recorded in ARCHITECTURE.md because it materially affects results.
    %
    %   DETERMINISM
    %   Each agent owns a private mrg32k3a substream, so its random draws do
    %   not depend on how many other agents exist or in what order they are
    %   updated.  Same seed, same scenario, same trajectories.
    %
    %   See also ROADMODEL, CLASSPRIORS, SCENARIO1.

    properties (SetAccess = private)
        id
        classId
        className
        priors                  % row from classPriors

        road                    % handle to the RoadModel
        seg                     % which ribbon this agent travels on
        s                       % arc length along that ribbon (m)
        d                       % signed lateral offset, + = left of tangent
        dir                     % +1 or -1 along the ribbon tangent

        vs                      % longitudinal speed along dir (m/s, >= 0)
        vd                      % lateral speed along +normal (m/s, signed)
        targetSpeed             % free-flow speed (m/s)
        dNominal                % lateral offset this agent aims to hold

        mode                    % cruise | along | crossing | dwell | graze | enter | done
        modeTime                % s spent in the current mode
        active                  % false once it leaves the ribbon

        % world-frame cache, refreshed every step
        x, y, yaw, vx, vy
        L, W, H

        trigger                 % event trigger, see evaluateTrigger
        triggered
        triggerTime

        rs                      % private RandStream substream
        speedNoise              % Ornstein-Uhlenbeck state
        driftPhase, driftAmp, driftTau
        crossSpeed, crossTarget
        dwellUntil
    end

    methods
        function obj = AgentModel(spec, road, seed)
            %AGENTMODEL  Build one agent.
            %
            %   spec fields: id, classId, seg, s, d, dir, targetSpeed (opt),
            %                trigger (opt), driftAmp (opt)

            obj.id        = spec.id;
            obj.classId   = spec.classId;
            obj.priors    = classPriors(spec.classId);
            obj.className = obj.priors.name;
            obj.road      = road;

            obj.L = obj.priors.length;
            obj.W = obj.priors.width;
            obj.H = obj.priors.height;

            obj.seg = getOr(spec, 'seg', 1);
            obj.s   = getOr(spec, 's', 0);
            obj.d   = getOr(spec, 'd', 0);
            obj.dir = sign(getOr(spec, 'dir', 1));
            if obj.dir == 0, obj.dir = 1; end
            obj.dNominal = obj.d;

            % --- private random stream -----------------------------------
            % Substream index is derived from the agent id, so adding or
            % removing an agent does not change any other agent's draws.
            obj.rs = RandStream('mrg32k3a', 'Seed', seed);
            obj.rs.Substream = 100 + obj.id;

            % --- speed ---------------------------------------------------
            rng0 = obj.priors.speedRange;
            if isfield(spec, 'targetSpeed') && ~isempty(spec.targetSpeed)
                obj.targetSpeed = spec.targetSpeed;
            else
                obj.targetSpeed = rng0(1) + rand(obj.rs) * (rng0(2) - rng0(1));
            end
            obj.vs = obj.targetSpeed;
            obj.vd = 0;
            obj.speedNoise = 0;

            % --- lateral drift -------------------------------------------
            b = obj.priors.behavior;
            if isfield(spec, 'driftAmp') && ~isempty(spec.driftAmp)
                obj.driftAmp = spec.driftAmp;
            else
                % Draw per agent around the class value, so two autos in the
                % same scene do not wander in lockstep.
                obj.driftAmp = b.lateralDriftAmp * (0.6 + 0.8 * rand(obj.rs));
            end
            obj.driftTau   = b.lateralDriftTau * (0.7 + 0.6 * rand(obj.rs));
            obj.driftPhase = 2 * pi * rand(obj.rs);

            % --- class-specific initial mode -----------------------------
            switch obj.priors.motionModel
                case 'pedestrian'
                    obj.mode = 'along';
                    obj.crossSpeed = 1.0 + 0.5 * rand(obj.rs);
                    obj.crossTarget = -sign(obj.d) * abs(obj.d);
                    if obj.crossTarget == 0
                        obj.crossTarget = 3.0;
                    end
                case 'cow'
                    obj.mode = 'graze';
                    obj.vs = 0;
                    obj.crossSpeed = 0.8 + 0.6 * rand(obj.rs);
                    obj.crossTarget = -sign(obj.d) * abs(obj.d);
                    if obj.crossTarget == 0
                        obj.crossTarget = 3.0;
                    end
                case 'pushcart'
                    obj.mode = 'cruise';
                otherwise
                    obj.mode = 'cruise';
            end
            obj.modeTime = 0;
            obj.dwellUntil = -inf;

            obj.trigger     = getOr(spec, 'trigger', struct('type', 'none'));
            obj.triggered   = false;
            obj.triggerTime = NaN;
            obj.active      = true;

            obj.refreshWorldState();
        end

        % ----------------------------------------------------------------
        function step(obj, dt, t, world)
            %STEP  Advance this agent by dt seconds.
            if ~obj.active
                return
            end

            obj.modeTime = obj.modeTime + dt;
            obj.evaluateTrigger(t, world);

            switch obj.priors.motionModel
                case {'vehicle', 'auto', 'twowheeler'}
                    obj.stepVehicle(dt, t, world);
                case 'pedestrian'
                    obj.stepPedestrian(dt, t, world);
                case 'pushcart'
                    obj.stepPushcart(dt, t, world);
                case 'cow'
                    obj.stepCow(dt, t, world);
                otherwise
                    obj.stepVehicle(dt, t, world);
            end

            % --- integrate along the ribbon ------------------------------
            obj.s = obj.s + obj.dir * obj.vs * dt;
            obj.d = obj.d + obj.vd * dt;

            L = obj.road.segmentLength(obj.seg);
            if obj.s < -1 || obj.s > L + 1
                obj.active = false;      % left the modelled stretch
                % Stop claiming to move.  STEP returns early once inactive, so
                % vs/vd freeze at whatever they held on the last live step, and
                % TRUTH reports that as the agent's speed forever.  On the
                % highway four agents ran off the end of the ribbon and sat at
                % s = L+1 still reporting 7-16 m/s: a constant-velocity tracker
                % predicted them forward every step, the measurement snapped
                % them back, association never settled, and the track list grew
                % to 35 tracks for 4 objects (90% false).  A thing that is not
                % moving has speed zero.
                obj.vs = 0;
                obj.vd = 0;
            end

            obj.refreshWorldState();
        end

        % ----------------------------------------------------------------
        function o = footprint(obj)
            %FOOTPRINT  Oriented bounding box in world coordinates.
            o = makeOBB(obj.x, obj.y, obj.yaw, obj.L, obj.W);
        end

        % ----------------------------------------------------------------
        function st = truth(obj)
            %TRUTH  The agentTruth contract (Section 4).
            %
            %   Ground truth, for EVALUATION ONLY.  Nothing in the perception
            %   or planning path is allowed to read this struct; the pipeline
            %   sees detections and tracks.
            st = struct( ...
                'id',         obj.id, ...
                'classId',    obj.classId, ...
                'className',  obj.className, ...
                'x',          obj.x, ...
                'y',          obj.y, ...
                'yaw',        obj.yaw, ...
                'v',          hypot(obj.vx, obj.vy), ...
                'vx',         obj.vx, ...
                'vy',         obj.vy, ...
                'length',     obj.L, ...
                'width',      obj.W, ...
                'isWrongWay', obj.isWrongWay(), ...
                'mode',       obj.mode, ...
                'active',     obj.active);
        end

        % ----------------------------------------------------------------
        function tf = isWrongWay(obj)
            %ISWRONGWAY  Ground-truth wrong-way flag (A2 evaluation).
            %
            %   Derived from the road's expected-direction field and the
            %   agent's ACTUAL velocity, not from a label set at spawn time.
            %   That way the truth cannot drift away from what the agent is
            %   really doing, and time-to-flag is measured against something
            %   real.
            tf = false;
            sp = hypot(obj.vx, obj.vy);
            if sp < 1.5 || ~obj.active
                return   % too slow to attribute a travel direction
            end
            [c, s, defined] = obj.road.expectedDirection(obj.x, obj.y);
            if ~defined
                return   % junctions have no expected direction
            end
            % Angle between actual heading and expected heading.
            dotp = (obj.vx * c + obj.vy * s) / sp;
            tf = dotp < cosd(120);
        end

        % ----------------------------------------------------------------
        function e = entity(obj)
            %ENTITY  Compact record for the gap-following interaction table.
            e = struct('id', obj.id, 'seg', obj.seg, 's', obj.s, 'd', obj.d, ...
                'L', obj.L, 'W', obj.W, 'isEgo', false, 'active', obj.active);
        end
    end

    % ====================================================================
    methods (Access = private)

        function stepVehicle(obj, dt, t, world)
            %STEPVEHICLE  Car, bus, auto and two-wheeler.
            b = obj.priors.behavior;

            % --- Ornstein-Uhlenbeck speed jitter -------------------------
            % Correlated noise, not white: real drivers vary their speed over
            % seconds, and white noise would be filtered out by the tracker
            % and never reach the planner.
            tau = max(b.speedNoiseTau, 1e-3);
            obj.speedNoise = obj.speedNoise + ...
                (-obj.speedNoise / tau) * dt + ...
                b.speedNoiseSigma * sqrt(2 * dt / tau) * randn(obj.rs);

            vTarget = max(obj.targetSpeed + obj.speedNoise, 0);

            % --- brake for whatever is ahead in this agent's corridor -----
            if b.reactsToObstacles
                gap = obj.gapAhead(world);
                stopGap = 1.5;
                if gap < stopGap
                    vTarget = 0;
                elseif gap < b.followGap
                    vTarget = vTarget * (gap - stopGap) / (b.followGap - stopGap);
                end
            end

            aMax = obj.priors.accelMax;
            obj.vs = obj.vs + max(min(vTarget - obj.vs, aMax * dt), -2.5 * aMax * dt);
            obj.vs = max(obj.vs, 0);

            % --- lateral drift about the nominal offset ------------------
            % Analytic derivative, so the lateral velocity that feeds the
            % predictor is exact rather than a finite difference.
            w = 2 * pi / max(obj.driftTau, 1e-3);
            obj.vd = obj.driftAmp * w * cos(w * t + obj.driftPhase);
            targetD = obj.dNominal + obj.driftAmp * sin(w * t + obj.driftPhase);

            % Drift stays on the agent's OWN HALF of the road.  Real drivers
            % wander within their side; they do not wander across the
            % centreline into oncoming traffic.  Unclamped, an auto's 0.77 m
            % drift amplitude around a -1.38 m offset reached -0.6 m, and on a
            % 7 m market street that was enough to close the gap on an ego
            % that was correctly keeping left - the ego was struck while
            % nearly stationary, having done nothing wrong.
            if obj.dNominal ~= 0
                minSide = 0.6;   % m, closest approach to the centreline
                if obj.dNominal > 0
                    targetD = max(targetD, minSide);
                else
                    targetD = min(targetD, -minSide);
                end
            end

            % Pull back toward the intended offset so drift cannot integrate
            % the agent off the road over a long run.
            obj.vd = obj.vd + 0.8 * (targetD - obj.d);
        end

        % ----------------------------------------------------------------
        function tf = egoIsBlockingCrossing(obj, world)
            %EGOISBLOCKINGCROSSING  Is a near-stationary ego right in the way?
            tf = false;
            if isempty(world) || ~isfield(world, 'ego')
                return
            end
            e = world.ego;
            if e.v > 1.5
                return   % still moving: the pedestrian does not defer to it
            end
            % Deliberately SMALL.  This exists only to stop a pedestrian
            % walking into the flank of a car that has already stopped for it;
            % it is not a general right-of-way rule.
            %
            % At 10 m it produced a mutual deadlock: the ego stopped for the
            % crossing pedestrian, the pedestrian stopped for the ego, and
            % neither ever moved again - village collapsed from 244 m to 39 m.
            % Two agents each waiting for the other is a worse failure than
            % the collision it was meant to prevent, and it is the failure
            % mode that any "politeness" rule risks creating.
            d = hypot(e.x - obj.x, e.y - obj.y);
            tf = d < 3.0;
        end

        function stepPedestrian(obj, dt, ~, world)
            b = obj.priors.behavior;
            obj.speedNoise = obj.speedNoise * 0.9 + ...
                b.speedNoiseSigma * sqrt(dt) * randn(obj.rs);

            switch obj.mode
                case 'along'
                    obj.vs = max(obj.targetSpeed + obj.speedNoise, 0.2);
                    obj.vd = 0;
                    if obj.triggered
                        obj.mode = 'crossing';
                        obj.modeTime = 0;
                    end

                case 'crossing'
                    % Walk across; slow forward drift while crossing.
                    obj.vs = 0.25 * obj.targetSpeed;
                    obj.vd = sign(obj.crossTarget - obj.d) * obj.crossSpeed;

                    % Stop short of a vehicle that is already stopped in the
                    % way.  Pedestrians here do NOT yield to moving traffic -
                    % that asymmetry is deliberate and is what makes them a
                    % hazard - but walking into the flank of a stationary car
                    % is not hazard modelling, it is the pedestrian causing a
                    % collision the planner had already avoided.  Measured: the
                    % ego stopped correctly for a crossing pedestrian in the
                    % market and was then walked into at 0.6 m/s.
                    if obj.egoIsBlockingCrossing(world)
                        obj.vs = 0;
                        obj.vd = 0;
                    end
                    if abs(obj.d - obj.crossTarget) < 0.3
                        obj.mode = 'along';
                        obj.modeTime = 0;
                        obj.dNominal = obj.crossTarget;
                        obj.vd = 0;
                    end

                otherwise
                    obj.vs = 0;
                    obj.vd = 0;
            end
        end

        % ----------------------------------------------------------------
        function stepPushcart(obj, dt, t, ~)
            b = obj.priors.behavior;

            if t < obj.dwellUntil
                obj.vs = 0;
                obj.vd = 0;
                return
            end

            % Long dwells at the edge: a Poisson-style stop.
            if rand(obj.rs) < b.dwellProb * dt
                obj.dwellUntil = t + 3 + 6 * rand(obj.rs);
                obj.vs = 0;
                obj.vd = 0;
                return
            end

            obj.speedNoise = obj.speedNoise * 0.95 + ...
                b.speedNoiseSigma * sqrt(dt) * randn(obj.rs);
            obj.vs = max(obj.targetSpeed + obj.speedNoise, 0.1);
            obj.vd = 0.3 * (obj.dNominal - obj.d);
        end

        % ----------------------------------------------------------------
        function stepCow(obj, dt, ~, world)
            switch obj.mode
                case 'graze'
                    % Near-stationary random walk: minutes of nothing, then a
                    % metre of movement with no intent cue beforehand.
                    obj.vs = 0.15 * randn(obj.rs);
                    obj.vd = 0.10 * randn(obj.rs);
                    obj.vs = max(min(obj.vs, 0.4), -0.4);
                    if obj.triggered
                        obj.mode = 'enter';
                        obj.modeTime = 0;
                    end

                case 'enter'
                    % Abrupt entry across the carriageway.
                    obj.vs = 0.2;
                    obj.vd = sign(obj.crossTarget - obj.d) * obj.crossSpeed;
                    if obj.egoIsBlockingCrossing(world)
                        obj.vs = 0; obj.vd = 0;
                    end
                    if abs(obj.d - obj.crossTarget) < 0.3
                        obj.mode = 'graze';
                        obj.modeTime = 0;
                        obj.dNominal = obj.crossTarget;
                    end

                otherwise
                    obj.vs = 0;
                    obj.vd = 0;
            end
        end

        % ----------------------------------------------------------------
        function evaluateTrigger(obj, t, world)
            %EVALUATETRIGGER  Decide when a scripted event fires.
            %
            %   'egoBrakingDistance' is the one that matters for Scenario 5:
            %   the cow enters when the ego is between 1.2 and 2.0 braking
            %   distances away, so the event is physically avoidable but only
            %   with a timely reaction.  The factor is drawn per seed, which
            %   is why the difficulty varies across seeds instead of being
            %   pinned to one convenient value.
            if obj.triggered || ~isfield(obj.trigger, 'type')
                return
            end

            switch obj.trigger.type
                case 'none'
                    return

                case 'time'
                    if t >= obj.trigger.at
                        obj.fire(t);
                    end

                case 'egoDistance'
                    if isempty(world) || ~isfield(world, 'ego'), return, end
                    dist = hypot(world.ego.x - obj.x, world.ego.y - obj.y);
                    if dist <= obj.trigger.at
                        obj.fire(t);
                    end

                case 'egoBrakingDistance'
                    if isempty(world) || ~isfield(world, 'ego'), return, end
                    v = world.ego.v;
                    decel = obj.trigger.decel;
                    dBrake = v^2 / (2 * decel);

                    % Floor the trigger distance so the event stays PHYSICALLY
                    % AVOIDABLE at any ego speed.  Braking distance goes to
                    % zero as the ego slows, and the distance is measured from
                    % the ego's REAR AXLE - so at 2 m/s the raw formula fired
                    % when the animal was 1.75 m from the rear axle, i.e.
                    % already beside the front of a 4.5 m vehicle. That is not
                    % a test of avoidance, it is an unavoidable collision, and
                    % it would have been scored against the planner.
                    %
                    % The floor is well clear of the vehicle's own length plus
                    % a reaction margin; at the scenario's nominal speed the
                    % braking term dominates and the floor never binds.
                    dTrigger = max(obj.trigger.factor * dBrake, ...
                                   obj.triggerField('minDistance', 12.0));

                    dist = hypot(world.ego.x - obj.x, world.ego.y - obj.y);
                    if dist <= dTrigger
                        obj.fire(t);
                    end

                otherwise
                    error('AgentModel:badTrigger', ...
                        'Unknown trigger type "%s".', obj.trigger.type);
            end
        end

        function v = triggerField(obj, name, dflt)
            if isfield(obj.trigger, name) && ~isempty(obj.trigger.(name))
                v = obj.trigger.(name);
            else
                v = dflt;
            end
        end

        function fire(obj, t)
            obj.triggered = true;
            obj.triggerTime = t;
        end

        % ----------------------------------------------------------------
        function gap = gapAhead(obj, world)
            %GAPAHEAD  Clear distance to the nearest thing in front.
            gap = inf;
            if isempty(world) || ~isfield(world, 'entities')
                return
            end

            ents = world.entities;
            for k = 1:numel(ents)
                e = ents(k);
                if e.id == obj.id || ~e.active || e.seg ~= obj.seg
                    continue
                end
                % Same corridor?  Half-widths plus a little room.
                if abs(e.d - obj.d) > 0.5 * (obj.W + e.W) + 0.6
                    continue
                end
                ahead = (e.s - obj.s) * obj.dir;
                if ahead <= 0
                    continue
                end
                gap = min(gap, ahead - 0.5 * (obj.L + e.L));
            end
            gap = max(gap, 0);
        end

        % ----------------------------------------------------------------
        function refreshWorldState(obj)
            %REFRESHWORLDSTATE  Map (seg, s, d) back to world coordinates.
            [xy, T, N] = obj.road.pointAt(obj.seg, obj.s, obj.d);
            obj.x = xy(1);
            obj.y = xy(2);

            vWorld = obj.dir * obj.vs * T + obj.vd * N;
            obj.vx = vWorld(1);
            obj.vy = vWorld(2);

            sp = hypot(obj.vx, obj.vy);
            if sp > 1e-3
                obj.yaw = atan2(obj.vy, obj.vx);
            else
                % Stationary: face along the travel direction rather than
                % snapping to an arbitrary heading.
                hdg = obj.dir * T;
                obj.yaw = atan2(hdg(2), hdg(1));
            end
        end
    end
end

% ========================================================================
function v = getOr(s, f, dflt)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f))
    v = s.(f);
else
    v = dflt;
end
end
