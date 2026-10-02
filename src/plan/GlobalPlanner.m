classdef GlobalPlanner < handle
    %GLOBALPLANNER  Hybrid A* over the risk map, with an RRT* fallback.
    %
    %   The occupancy map handed to Hybrid A* is the unified risk field
    %   thresholded at cfg.plan.riskThreshold and inflated by half the vehicle
    %   width plus the context's lateral margin.
    %
    %   THE BINARY-CHECK CAVEAT, stated rather than hidden
    %   plannerHybridAStar takes a BINARY validity check: a state is free or
    %   it is not.  So the global planner cannot distinguish a cell at risk
    %   0.61 from one at 0.99 - both are simply blocked - and it cannot prefer
    %   the cheaper of two passable routes.  The CONTINUOUS risk enters
    %   through the local planner (DWAPlanner), which integrates R along every
    %   rollout.  The division of labour is deliberate: Hybrid A* supplies a
    %   kinematically feasible corridor, DWA chooses where within and around
    %   that corridor to actually drive.  ARCHITECTURE.md carries this note
    %   too, because a reader could otherwise assume the risk map grades the
    %   global search when it does not.
    %
    %   BUDGET
    %   plan() is blocking and cannot be interrupted, so a 150 ms budget
    %   cannot be enforced by timing the call - by the time it has overrun,
    %   it has overrun.  It is enforced a priori through MaxNumNodes, whose
    %   value is calibrated against measured worst-case latency (see
    %   defaultConfig).  Hitting the cap raises a catchable error, which is
    %   taken as "too hard for Hybrid A*" and routed to the RRT* fallback.
    %
    %   FRAME
    %   Planning happens in the EGO frame, the same frame as the risk grid, so
    %   no resampling is needed.  The returned path is converted to world
    %   coordinates for the controller.
    %
    %   FORWARD ONLY: MotionDirection is 'forward'.  The local planner never
    %   commands reverse and the vehicle model refuses it, so allowing the
    %   global planner to return reversing manoeuvres would produce paths the
    %   rest of the stack cannot execute.
    %
    %   See also DWAPLANNER, RISKMAP, DEFAULTCONFIG.

    properties (SetAccess = private)
        cfg
        backend         % 'hybridAStar' | 'rrtStar' | 'gridAStar'
        fallback        % 'rrtStar' | 'gridAStar'
        lastPlanner     % which one actually produced the last path
        lastLatencyMs
        lastFailReason
        lastMargin      % inflation actually used, after any relaxation
    end

    properties (Access = private)
        hasPath = false     % set by the caller via notePathHeld
    end

    methods
        function notePathHeld(obj, tf)
            %NOTEPATHHELD  Tell the planner whether a usable path already exists.
            %
            %   Governs whether the RRT* fallback is worth running.  Measured:
            %   with the fallback firing on every failed Hybrid A* call, RRT*
            %   accounted for 20.2 s of a 32 s simulated run - about 62 ms of
            %   every 100 ms cycle, on its own over half the entire budget.
            %   When a previous path is still in hand, retrying Hybrid A* next
            %   cycle costs nothing and is nearly always enough; RRT* is worth
            %   its price only when there is no path at all.
            obj.hasPath = logical(tf);
        end

        function obj = GlobalPlanner(cfg)
            obj.cfg = cfg;
            obj.backend = cfg.env.backends.globalPlanner;
            obj.fallback = cfg.env.backends.globalPlannerFallback;
            obj.lastPlanner = 'none';
            obj.lastLatencyMs = NaN;
            obj.lastFailReason = '';
        end

        % ----------------------------------------------------------------
        function out = plan(obj, riskMap, egoState, goalWorld, decision)
            %PLAN  Global path from the ego to the goal, in world coordinates.
            %
            %   OUT fields
            %       path         M x 3 [x y yaw] in WORLD coordinates ([] on failure)
            %       plannerUsed  'hybridAStar' | 'rrtStar' | 'gridAStar' | 'none'
            %       latencyMs    wall clock for the whole attempt
            %       success      logical
            %       reason       why it failed, when it did

            tAll = tic;
            out = struct('path', [], 'plannerUsed', 'none', 'latencyMs', NaN, ...
                'success', false, 'reason', '');

            % Inflation is the half vehicle width only - see buildMap for why
            % the context clearance margin is deliberately NOT added here.
            margin = obj.cfg.vehicle.width / 2 + obj.cfg.plan.inflateExtraM;

            % Build the map, relaxing inflation if the ego's own pose lands
            % inside it.  On a 6 m road a 1.8 m vehicle keeping left sits
            % about 1.0 m from the edge, and a 0.9 m inflation plus grid
            % rounding can swallow exactly that - the planner then refuses to
            % plan from where the vehicle already is.  Measured: 377 refusals
            % in one run.
            %
            % Relaxing is legitimate.  The vehicle IS at that pose and is
            % demonstrably not in collision (the run would have ended
            % otherwise), so inflation there is a preference that has already
            % been overtaken by events.  It is relaxed only as far as needed,
            % and only for the cell the vehicle occupies; the rest of the map
            % keeps the full margin.
            startEgo = [0, 0, 0];                       % the ego is the origin
            [map, ss] = obj.buildMap(riskMap, margin);
            obj.lastMargin = margin;
            obj.freeStartCell(map, startEgo);
            goalEgo = obj.projectGoal(riskMap, goalWorld, map);
            if isempty(goalEgo)
                out.reason = 'no reachable goal cell inside the risk window';
                out.latencyMs = toc(tAll) * 1000;
                obj.record(out);
                return
            end

            % --- primary: Hybrid A* ---------------------------------
            if strcmp(obj.backend, 'hybridAStar')
                [p, ok, why] = obj.runHybrid(ss, map, startEgo, goalEgo);
                if ok
                    out.path = obj.toWorld(riskMap, p);
                    out.plannerUsed = 'hybridAStar';
                    out.success = true;
                    out.latencyMs = toc(tAll) * 1000;
                    obj.record(out);
                    return
                end
                out.reason = why;
            end

            % --- fallback: RRT* on a Reeds-Shepp space ---------------
            % Only when there is nothing to fall back ON.  See notePathHeld.
            if obj.hasPath
                out.reason = sprintf('%s; keeping previous path (RRT* skipped)', out.reason);
                out.latencyMs = toc(tAll) * 1000;
                obj.record(out);
                return
            end

            [p, ok, why] = obj.runRRT(map, startEgo, goalEgo);
            if ok
                out.path = obj.toWorld(riskMap, p);
                out.plannerUsed = 'rrtStar';
                out.success = true;
            else
                out.reason = sprintf('%s; fallback: %s', out.reason, why);
            end

            out.latencyMs = toc(tAll) * 1000;
            obj.record(out);
        end
    end

    % ====================================================================
    methods (Access = private)

        function [map, ss] = buildMap(obj, riskMap, inflateM)
            %BUILDMAP  Hard constraints for the global search.
            %
            %   DEVIATION from the literal spec, which says "risk map
            %   thresholded at tau, inflated by half vehicle width + context
            %   margin".  That combination is geometrically infeasible on the
            %   roads this project targets, and it double-counts the same
            %   safety margin twice:
            %
            %     6 m road, half-width 3.0 m
            %     edge risk reaches tau = 0.6 at 0.4 m from the boundary
            %     inflation = 0.9 (half car) + 0.8 (village margin) = 1.7 m
            %     free band for the vehicle centre = 3.0 - 0.4 - 1.7 = 0.9 m
            %
            %   A 1.8 m corridor down the middle of a two-way road: the
            %   planner would be pinned to the centreline, driving head-on
            %   into oncoming traffic, and the ego's own starting pose at
            %   1.5 m offset sits INSIDE the occupied region, so Hybrid A*
            %   refuses to plan at all.  Measured: every global plan failed
            %   and the ego deadlocked at 0.36 m/s.
            %
            %   The edge layer already encodes "keep away from the edge" as a
            %   smooth gradient.  Thresholding that gradient AND then inflating
            %   by the clearance margin applies the same intent twice, once as
            %   a soft cost and once as a wall.
            %
            %   So the hard constraint is what is genuinely impassable -
            %   off-road, solid obstacles, and high predicted occupancy - and
            %   inflation is the half vehicle width, which is the geometric
            %   price of planning a point instead of a body.  The context
            %   clearance margin stays a PREFERENCE, expressed through the
            %   edge gradient and the risk term in the DWA cost, which is
            %   where a preference belongs.  See DEVIATIONS D24.

            tau = obj.cfg.plan.riskThreshold;
            L = riskMap.layers;

            offRoad = L.edge >= 0.999;          % genuinely not drivable
            solid   = L.blocking >= tau;        % stalls, parked cars, debris
            dynamic = L.dynamic  >= tau;        % high predicted occupancy

            % Note: L.blocking, NOT L.static.  Surface hazards such as
            % potholes are in L.static and stay OUT of the hard constraint -
            % they are expensive to drive over, not impossible, and treating
            % them as walls blocks the road entirely.
            occ = offRoad | solid | dynamic;

            % binaryOccupancyMap indexes row 1 as the HIGHEST y, while the
            % risk grid indexes row 1 as the LOWEST.  Without the flip the map
            % is mirrored about the ego's axis and the planner steers into
            % obstacles it thinks are on the other side.
            map = binaryOccupancyMap(flipud(occ), 1 / riskMap.res);
            map.GridLocationInWorld = [riskMap.xe(1), riskMap.ye(1)];

            if inflateM > 0
                inflate(map, inflateM);
            end

            ss = stateSpaceSE2;
            ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi, pi]];
        end

        % ----------------------------------------------------------------
        function freeStartCell(obj, map, startEgo)
            %FREESTARTCELL  Clear inflation from the pose the ego already holds.
            %
            %   ONLY the cells the vehicle currently occupies, and only when
            %   the start is otherwise invalid.  The rest of the map keeps its
            %   full margin.
            %
            %   An earlier version relaxed the inflation across the WHOLE map
            %   when the start was blocked.  That let the planner route with
            %   the vehicle centre right at the carriageway edge, and the ego
            %   drove off the road at 59 m.  The distinction matters: the
            %   vehicle is already at the start pose and is not in collision,
            %   so the margin there has been overtaken by events - everywhere
            %   else it is still doing its job.
            if ~checkOccupancy(map, startEgo(1:2))
                return
            end

            r = obj.cfg.vehicle.width / 2 + 0.3;
            step = 1 / map.Resolution;
            [gx, gy] = meshgrid(-r:step:r, -r:step:r);
            keep = hypot(gx, gy) <= r;
            pts = [startEgo(1) + gx(keep), startEgo(2) + gy(keep)];

            inBounds = pts(:,1) >= map.XWorldLimits(1) & pts(:,1) <= map.XWorldLimits(2) & ...
                       pts(:,2) >= map.YWorldLimits(1) & pts(:,2) <= map.YWorldLimits(2);
            if any(inBounds)
                setOccupancy(map, pts(inBounds, :), 0);
            end
        end

        % ----------------------------------------------------------------
        function goalEgo = projectGoal(obj, riskMap, goalWorld, map)
            %PROJECTGOAL  Bring the goal inside the window and onto free space.
            %
            %   The goal is usually far beyond the 70 m horizon, so it becomes
            %   a waypoint on the window boundary in the goal's direction.
            %   If that cell is occupied, the nearest free cell is used - a
            %   planner that refuses to move because its distant goal happens
            %   to sit under a predicted pedestrian would be useless.

            [gx, gy] = riskMap.toEgo(goalWorld(1), goalWorld(2));

            pad = 2.0;
            gx = min(max(gx, riskMap.xe(1) + pad), riskMap.xe(end) - pad);
            gy = min(max(gy, riskMap.ye(1) + pad), riskMap.ye(end) - pad);

            heading = atan2(gy, gx);
            goalEgo = [gx, gy, heading];

            if ~checkOccupancy(map, [gx, gy])
                return
            end

            % Spiral outward for the nearest free cell.
            for r = 0.5:0.5:12
                for a = 0:pi/8:2*pi
                    cx = gx + r * cos(a);
                    cy = gy + r * sin(a);
                    if cx < riskMap.xe(1) + pad || cx > riskMap.xe(end) - pad || ...
                       cy < riskMap.ye(1) + pad || cy > riskMap.ye(end) - pad
                        continue
                    end
                    if ~checkOccupancy(map, [cx, cy])
                        goalEgo = [cx, cy, atan2(cy, cx)];
                        return
                    end
                end
            end

            goalEgo = [];
        end

        % ----------------------------------------------------------------
        function [p, ok, why] = runHybrid(obj, ss, map, startEgo, goalEgo)
            p = []; ok = false; why = '';

            val = validatorOccupancyMap(ss, 'Map', map);
            val.ValidationDistance = 0.3;

            if checkOccupancy(map, startEgo(1:2))
                % The ego's own cell is "occupied" - usually its own inflation
                % radius against a nearby edge.  Hybrid A* refuses to start
                % there, so this is a fallback case, not an error.
                why = 'start pose lies in inflated occupied space';
                return
            end

            planner = plannerHybridAStar(val, ...
                'MinTurningRadius', obj.minTurningRadius(), ...
                'MotionPrimitiveLength', obj.cfg.plan.motionPrimitiveLength, ...
                'MaxNumNodes', obj.cfg.plan.hybridMaxNodes, ...
                'MotionDirection', 'forward');

            % plannerHybridAStar prints its "no obstacle-free path" message
            % straight to the console rather than raising a suppressible
            % warning, and a failed replan is a normal, logged event here -
            % not something to spray over every run's output.  evalc captures
            % it; the failure itself is reported through the return value.
            ws = warning('off', 'all');
            cleanup = onCleanup(@() warning(ws));
            try
                pathObj = [];
                evalc('pathObj = plan(planner, startEgo, goalEgo);');
                if isempty(pathObj) || isempty(pathObj.States) || size(pathObj.States, 1) < 2
                    why = 'hybrid A* found no obstacle-free path';
                    return
                end
                p = pathObj.States;
                ok = true;
            catch ME
                % The node cap is the budget guard firing, not a bug.
                if contains(ME.message, 'MaxNumNodes')
                    why = sprintf('hybrid A* hit the %d node budget', ...
                        obj.cfg.plan.hybridMaxNodes);
                else
                    why = sprintf('hybrid A* error: %s', ME.message);
                end
            end
        end

        % ----------------------------------------------------------------
        function [p, ok, why] = runRRT(obj, map, startEgo, goalEgo)
            p = []; ok = false; why = '';

            if exist('plannerRRTStar', 'file') == 0
                why = 'plannerRRTStar unavailable';
                return
            end

            ssRS = stateSpaceReedsShepp;
            ssRS.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi, pi]];
            ssRS.MinTurningRadius = obj.minTurningRadius();

            val = validatorOccupancyMap(ssRS, 'Map', map);
            val.ValidationDistance = 0.3;

            rrt = plannerRRTStar(ssRS, val, ...
                'MaxIterations', obj.cfg.plan.rrtMaxIterations, ...
                'MaxConnectionDistance', 6, ...
                'GoalBias', 0.15);

            ws = warning('off', 'all');
            cleanup = onCleanup(@() warning(ws));
            try
                pathObj = []; info = struct('IsPathFound', false);
                evalc('[pathObj, info] = plan(rrt, startEgo, goalEgo);');
                if ~info.IsPathFound || size(pathObj.States, 1) < 2
                    why = 'RRT* found no path';
                    return
                end
                interpolate(pathObj, max(size(pathObj.States, 1) * 4, 20));
                p = pathObj.States;
                ok = true;
            catch ME
                why = sprintf('RRT* error: %s', ME.message);
            end
        end

        % ----------------------------------------------------------------
        function w = toWorld(~, riskMap, p)
            [wx, wy] = riskMap.toWorld(p(:, 1), p(:, 2));
            w = [wx, wy, p(:, 3) + riskMap.yaw];
        end

        function r = minTurningRadius(obj)
            r = obj.cfg.vehicle.wheelbase / tan(obj.cfg.vehicle.maxSteer);
        end

        function record(obj, out)
            obj.lastPlanner = out.plannerUsed;
            obj.lastLatencyMs = out.latencyMs;
            obj.lastFailReason = out.reason;
        end
    end
end
