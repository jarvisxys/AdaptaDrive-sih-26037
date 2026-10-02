classdef MetricsLogger
    %METRICSLOGGER  Compute the reported metrics from a finished log.
    %
    %   Every metric is a PURE FUNCTION OF THE LOG.  Nothing is accumulated
    %   inside the simulation loop and nothing is passed in from the planner,
    %   so the UI, the CSV and the figures cannot disagree, and a metric can
    %   never be "helped" by the component it is supposed to be judging.
    %
    %   Metrics that a milestone has not built yet are reported as NaN with a
    %   matching entry in .notImplemented, never as 0 and never omitted.  A
    %   zero would read as a good result.
    %
    %   Definitions (Section 6)
    %     collision        ego OBB overlaps any agent or solid obstacle (SAT)
    %     minClearance     min OBB-to-OBB distance over the whole run
    %     minTTC           min over steps and agents of the constant-velocity
    %                      time to footprint overlap, capped at 10 s
    %     jerk             d(a)/dt from the logged acceleration
    %     lateral accel    v^2 * tan(steer) / wheelbase
    %     curvature        of the DRIVEN path, from the logged positions
    %     hazardTraversals number of distinct pothole entries (A1)
    %
    %   See also SIMLOG, SIMENGINE.

    methods (Static)

        function m = compute(log, scenario, cfg)
            %COMPUTE  All metrics for one run.

            n = log.n;
            m = MetricsLogger.blank();

            m.scenario   = scenario.name;
            m.config     = cfg.name;
            m.seed       = cfg.seed;
            m.density    = cfg.density;
            m.outcome    = log.meta.outcome.reason;
            m.completed  = log.meta.outcome.success;
            m.collision  = strcmp(log.meta.outcome.reason, 'collision');
            m.timeoutFail = strcmp(log.meta.outcome.reason, 'timeout');
            m.offroadFail = strcmp(log.meta.outcome.reason, 'offroad');
            m.completionTime = log.t(max(n, 1));

            if n < 2
                return
            end

            dt = cfg.sim.dt;
            v     = log.ego(:, 4);
            a     = log.ego(:, 5);
            steer = log.ego(:, 6);

            % --- smoothness ------------------------------------------------
            jerk = diff(a) / dt;
            m.jerkRMS = sqrt(mean(jerk.^2));
            m.jerkMax = max(abs(jerk));

            latAcc = v.^2 .* tan(steer) / cfg.vehicle.wheelbase;
            m.latAccRMS = sqrt(mean(latAcc.^2));
            m.latAccMax = max(abs(latAcc));

            [kappa, dkds] = MetricsLogger.pathCurvature(log.ego(:, 1), log.ego(:, 2));
            m.curvatureMean = mean(abs(kappa));
            m.curvatureMax  = max(abs(kappa));
            m.dKappaDsMax   = max(abs(dkds));

            m.meanSpeed = mean(v);
            m.maxSpeed  = max(v);
            m.pathLength = sum(hypot(diff(log.ego(:,1)), diff(log.ego(:,2))));

            % --- safety: clearance, TTC, hazards --------------------------
            [m.minClearance, m.minTTC, m.minClearanceTime, m.minTTCTime] = ...
                MetricsLogger.safetySweep(log, scenario, cfg);

            [m.hazardTraversals, m.minPotholeClearance] = ...
                MetricsLogger.hazardSweep(log, scenario, cfg);

            % --- events ----------------------------------------------------
            m.emergencyBrakes = MetricsLogger.countEvents(log, 'EMERGENCY_BRAKE');
            m.wrongWayFlags   = MetricsLogger.countEvents(log, 'WRONG_WAY');
            m.mergeDetections = MetricsLogger.countEvents(log, 'MERGE');
            m.replanCount     = MetricsLogger.countEvents(log, 'REPLAN');

            % --- latency ---------------------------------------------------
            if log.nCycles > 0
                lat = [log.cycles.latencyMs];
                lat = lat(~isnan(lat));
                if ~isempty(lat)
                    m.latencyP50 = median(lat);
                    m.latencyP95 = prctile(lat, 95);
                    m.latencyMax = max(lat);
                    m.latencyUnder200 = mean(lat < cfg.sim.latencyBudgetMs);
                    m.cycleCount = numel(lat);
                    stages = unique(string({log.cycles.stage}));
                    m.latencyStage = strjoin(stages, '+');
                end
                m.stageMs = MetricsLogger.stageBreakdown(log);
            end

            % --- perception ---------------------------------------------
            if log.nCycles > 0 && isfield(log.cycles, 'tracks')
                m.perception = MetricsLogger.perceptionStats(log, scenario, cfg);
            end

            % --- provenance -------------------------------------------------
            m.matlabRelease = cfg.env.platform.matlabRelease;
            m.cpu           = cfg.env.platform.cpu;
            m.gitCommit     = cfg.env.gitCommit;
            m.timestamp     = datetime('now');
        end

        % ----------------------------------------------------------------
        function m = blank()
            %BLANK  Every metric, pre-set to NaN.
            %
            %   NaN is the honest default: it prints as NaN, it does not
            %   average into a summary, and it cannot be mistaken for a good
            %   score the way 0 can.
            m = struct( ...
                'scenario', "", 'config', "", 'seed', NaN, 'density', NaN, ...
                'outcome', "", 'completed', false, 'collision', false, ...
                'timeoutFail', false, 'offroadFail', false, ...
                'completionTime', NaN, 'pathLength', NaN, ...
                'minClearance', NaN, 'minClearanceTime', NaN, ...
                'minTTC', NaN, 'minTTCTime', NaN, ...
                'hazardTraversals', NaN, 'minPotholeClearance', NaN, ...
                'jerkRMS', NaN, 'jerkMax', NaN, ...
                'latAccRMS', NaN, 'latAccMax', NaN, ...
                'curvatureMean', NaN, 'curvatureMax', NaN, 'dKappaDsMax', NaN, ...
                'meanSpeed', NaN, 'maxSpeed', NaN, ...
                'emergencyBrakes', NaN, 'wrongWayFlags', NaN, ...
                'mergeDetections', NaN, 'replanCount', NaN, ...
                'latencyP50', NaN, 'latencyP95', NaN, 'latencyMax', NaN, ...
                'latencyUnder200', NaN, 'cycleCount', 0, 'latencyStage', "", ...
                'stageMs', MetricsLogger.blankStageBreakdown(), ...
                'perception', MetricsLogger.blankPerception(), ...
                'matlabRelease', "", 'cpu', "", 'gitCommit', "", ...
                'timestamp', NaT);
        end

        % ----------------------------------------------------------------
        function print(m, extraNote)
            %PRINT  Console summary for run_demo.
            line = repmat('-', 1, 72);
            fprintf('\n%s\n  RUN METRICS  (simulation)\n%s\n', line, line);
            fprintf('  scenario / config / seed : %s / %s / %d\n', ...
                m.scenario, m.config, m.seed);
            fprintf('  outcome                  : %s\n', upper(string(m.outcome)));
            fprintf('  completed                : %s\n', yn(m.completed));
            fprintf('  completion time          : %s\n', fmt(m.completionTime, 's', 2));
            fprintf('  path length              : %s\n', fmt(m.pathLength, 'm', 1));
            fprintf('  mean / max speed         : %s / %s\n', ...
                fmt(m.meanSpeed, 'm/s', 2), fmt(m.maxSpeed, 'm/s', 2));
            fprintf('%s\n', line);
            fprintf('  min clearance            : %s   (at t = %s)\n', ...
                fmt(m.minClearance, 'm', 3), fmt(m.minClearanceTime, 's', 2));
            fprintf('  min TTC                  : %s   (at t = %s)\n', ...
                fmt(m.minTTC, 's', 2), fmt(m.minTTCTime, 's', 2));
            fprintf('  pothole traversals (A1)  : %s\n', fmtInt(m.hazardTraversals));
            fprintf('  min pothole clearance    : %s\n', fmt(m.minPotholeClearance, 'm', 3));
            fprintf('%s\n', line);
            fprintf('  jerk RMS / max           : %s / %s\n', ...
                fmt(m.jerkRMS, 'm/s^3', 3), fmt(m.jerkMax, 'm/s^3', 3));
            fprintf('  lateral accel RMS / max  : %s / %s\n', ...
                fmt(m.latAccRMS, 'm/s^2', 3), fmt(m.latAccMax, 'm/s^2', 3));
            fprintf('  curvature mean / max     : %s / %s\n', ...
                fmt(m.curvatureMean, '1/m', 4), fmt(m.curvatureMax, '1/m', 4));
            fprintf('%s\n', line);
            fprintf('  emergency brakes         : %s\n', fmtInt(m.emergencyBrakes));
            fprintf('  wrong-way flags (A2)     : %s\n', fmtInt(m.wrongWayFlags));
            fprintf('  merge detections (A3)    : %s\n', fmtInt(m.mergeDetections));
            fprintf('  replans                  : %s\n', fmtInt(m.replanCount));
            fprintf('%s\n', line);
            if m.cycleCount > 0
                fprintf('  cycle latency p50/p95    : %s / %s      [stage: %s]\n', ...
                    fmt(m.latencyP50, 'ms', 1), fmt(m.latencyP95, 'ms', 1), m.latencyStage);
                fprintf('  latency max              : %s   over %d cycles\n', ...
                    fmt(m.latencyMax, 'ms', 1), m.cycleCount);
                fprintf('  fraction under 200 ms    : %s\n', fmtPct(m.latencyUnder200));
                if ~strcmp(m.latencyStage, "plan")
                    fprintf('  NB: this is NOT the full replanning cycle - stage is "%s".\n', ...
                        m.latencyStage);
                end
            else
                fprintf('  replan latency           : not measured (no planning cycles in this config)\n');
            end

            p = m.perception;
            if p.cycles > 0
                fprintf('%s\n  PERCEPTION  (tracks vs ground truth)\n%s\n', line, line);
                fprintf('  mean tracks / cycle      : %s\n', fmt(p.meanTracks, '', 2));
                fprintf('  mean truths in view      : %s   (geometry only, ignores occlusion)\n', ...
                    fmt(p.meanInView, '', 2));
                fprintf('  mean matched             : %s\n', fmt(p.meanMatched, '', 2));
                fprintf('  recall (matched/in view) : %s\n', fmtPct(p.recall));
                fprintf('  duplicate tracks         : %s   (2nd track on a real object)\n', ...
                    fmtPct(p.duplicateTrackRate));
                fprintf('  false tracks             : %s   (no truth within 3.5 m)\n', ...
                    fmtPct(p.falseTrackRate));
                fprintf('  track position RMSE      : %s\n', fmt(p.posRMSE, 'm', 3));
                fprintf('  track position max error : %s\n', fmt(p.posMaxErr, 'm', 3));
                fprintf('  track velocity RMSE      : %s\n', fmt(p.velRMSE, 'm/s', 3));
                fprintf('  class accuracy           : %s   (of tracks with a class)\n', ...
                    fmtPct(p.classAccuracy));
                fprintf('  class "unknown" rate     : %s\n', fmtPct(p.classUnknownRate));
                fprintf('  mean matched range       : %s   (max %s)\n', ...
                    fmt(p.meanMatchRange, 'm', 1), fmt(p.maxMatchRange, 'm', 1));
                fprintf('  mean range when missed   : %s\n', fmt(p.meanMissRange, 'm', 1));

                if any(p.byClassInView > 0)
                    T = classPriors();
                    fprintf('\n  recall by class (matched / in geometric view):\n');
                    for c = 1:7
                        if p.byClassInView(c) == 0
                            continue
                        end
                        fprintf('     %-11s %5d / %5d   %s\n', T(c).name, ...
                            p.byClassMatched(c), p.byClassInView(c), ...
                            fmtPct(p.byClassRecall(c)));
                    end
                    fprintf(['  "in view" is FOV and range only. It ignores occlusion and the\n' ...
                             '  camera''s MinObjectImageSize, so small distant objects counted\n' ...
                             '  here are not physically detectable. Recall against it is a\n' ...
                             '  LOWER bound on true recall.\n']);
                end
            end
            fprintf('%s\n', line);
            fprintf('  MATLAB R%s on %s\n', m.matlabRelease, m.cpu);
            if nargin > 1 && ~isempty(extraNote)
                fprintf('  NOTE: %s\n', extraNote);
            end
            fprintf('%s\n\n', line);
        end
    end

    % ====================================================================
    methods (Static, Access = private)

        function [minClear, minTTCv, tClear, tTTC] = safetySweep(log, scenario, cfg)
            %SAFETYSWEEP  Clearance and TTC over the whole run.
            minClear = inf;   tClear = NaN;
            minTTCv  = 10.0;  tTTC   = NaN;

            veh = cfg.vehicle;
            off = veh.length / 2 - veh.rearOverhang;
            statics = scenario.hazards.staticOBBs();

            for k = 1:log.n
                st = log.egoStateAt(k);
                egoBox = makeOBB(st.x + off*cos(st.yaw), st.y + off*sin(st.yaw), ...
                    st.yaw, veh.length, veh.width);
                egoVel = [st.v * cos(st.yaw), st.v * sin(st.yaw)];

                truths = log.agents{k};
                for i = 1:numel(truths)
                    a = truths(i);
                    if ~a.active
                        continue
                    end
                    box = makeOBB(a.x, a.y, a.yaw, a.length, a.width);

                    d = obbDistance(egoBox, box);
                    if d < minClear
                        minClear = d;
                        tClear = st.t;
                    end

                    tc = ttc(egoBox, egoVel, box, [a.vx, a.vy]);
                    if tc < minTTCv
                        minTTCv = tc;
                        tTTC = st.t;
                    end
                end

                for i = 1:numel(statics)
                    d = obbDistance(egoBox, statics(i));
                    if d < minClear
                        minClear = d;
                        tClear = st.t;
                    end
                end
            end

            if isinf(minClear)
                minClear = NaN;   % nothing to measure against
            end
        end

        % ----------------------------------------------------------------
        function [traversals, minPotholeClear] = hazardSweep(log, scenario, cfg)
            %HAZARDSWEEP  Pothole entries and clearance (requirement A1).
            %
            %   Traversals count DISTINCT entries: a vehicle sitting over one
            %   pothole for twenty steps has traversed one pothole, not twenty.
            hz = scenario.hazards;
            traversals = 0;
            minPotholeClear = inf;

            if hz.numPotholes() == 0
                minPotholeClear = NaN;
                return
            end

            veh = cfg.vehicle;
            off = veh.length / 2 - veh.rearOverhang;
            wasIn = false(1, hz.numPotholes());

            for k = 1:log.n
                st = log.egoStateAt(k);
                box = makeOBB(st.x + off*cos(st.yaw), st.y + off*sin(st.yaw), ...
                    st.yaw, veh.length, veh.width);

                [~, ids] = hz.potholesUnderFootprint(box);
                nowIn = false(1, hz.numPotholes());
                nowIn(ids) = true;
                traversals = traversals + sum(nowIn & ~wasIn);
                wasIn = nowIn;

                minPotholeClear = min(minPotholeClear, hz.clearanceToPotholes(box));
            end
        end

        % ----------------------------------------------------------------
        function ps = perceptionStats(log, scenario, cfg)
            %PERCEPTIONSTATS  Track quality against ground truth (M2 report).
            %
            %   Tracks are matched to truth greedily by nearest position
            %   within MatchGate.  Deliberately simple and stated, rather than
            %   an optimal assignment that would flatter recall.
            %
            %   "In view" counts truth agents inside the camera or radar
            %   footprint by pure geometry.  It IGNORES OCCLUSION, so it is an
            %   UPPER BOUND on what is detectable: recall measured against it
            %   is therefore a lower bound on true recall, never the reverse.

            ps = MetricsLogger.blankPerception();
            matchGate = 3.5;    % m

            nCyc = log.nCycles;
            if nCyc == 0
                return
            end

            sqErr = [];
            nTracks = zeros(1, nCyc);
            nInView = zeros(1, nCyc);
            nMatched = zeros(1, nCyc);
            nFalse = zeros(1, nCyc);
            nDuplicate = zeros(1, nCyc);
            classRight = 0; classSeen = 0; classUnknown = 0;
            byClassInView = zeros(1, 7);
            byClassMatched = zeros(1, 7);
            matchRange = [];
            missRange = [];
            sqVelErr = [];

            for c = 1:nCyc
                cyc = log.cycles(c);
                tr = cyc.tracks;
                k = cyc.step;
                if isempty(k) || k < 1 || k > log.n
                    continue
                end
                truths = log.agents{k};
                st = log.egoStateAt(k);

                inView = false(1, numel(truths));
                for i = 1:numel(truths)
                    inView(i) = truths(i).active && ...
                        MetricsLogger.isInSensorFootprint(st, truths(i), cfg);
                    if inView(i)
                        cid = truths(i).classId;
                        if cid >= 1 && cid <= 7
                            byClassInView(cid) = byClassInView(cid) + 1;
                        end
                    end
                end

                nTracks(c) = numel(tr);
                nInView(c) = sum(inView);

                matchedTruth = false(1, numel(truths));
                used = false(1, numel(truths));
                for j = 1:numel(tr)
                    best = 0; bestD = inf;
                    for i = 1:numel(truths)
                        if used(i) || ~truths(i).active
                            continue
                        end
                        d = hypot(tr(j).x - truths(i).x, tr(j).y - truths(i).y);
                        if d < bestD
                            bestD = d; best = i;
                        end
                    end
                    if best > 0 && bestD <= matchGate
                        used(best) = true;
                        matchedTruth(best) = true;
                        nMatched(c) = nMatched(c) + 1;
                        sqErr(end+1) = bestD^2; %#ok<AGROW>
                        % Recall is counted ONLY over truths that were in the
                        % geometric view.  A track matched to an out-of-view
                        % truth (a coasting track, or an object just outside
                        % the cone) is a legitimate track, but counting it in
                        % the numerator against an in-view denominator
                        % produced recall above 100%.
                        cid = truths(best).classId;
                        if inView(best) && cid >= 1 && cid <= 7
                            byClassMatched(cid) = byClassMatched(cid) + 1;
                        end
                        matchRange(end+1) = hypot(truths(best).x - st.x, ...
                                                  truths(best).y - st.y); %#ok<AGROW>
                        % Velocity accuracy matters because prediction (B2/B3)
                        % is built on it, and the tracker deliberately does not
                        % use the radar's range rate.  Reported, not assumed.
                        sqVelErr(end+1) = (tr(j).vx - truths(best).vx)^2 + ...
                                          (tr(j).vy - truths(best).vy)^2; %#ok<AGROW>
                        if tr(j).classId == 0
                            classUnknown = classUnknown + 1;
                        else
                            classSeen = classSeen + 1;
                            if tr(j).classId == truths(best).classId
                                classRight = classRight + 1;
                            end
                        end
                    else
                        % Distinguish the two failure modes.  A DUPLICATE sits
                        % on a real object that another track already claimed
                        % (track fragmentation); a FALSE track sits on nothing.
                        % They call for opposite fixes, so reporting them as
                        % one number would hide which problem exists.
                        anyD = inf;
                        for i = 1:numel(truths)
                            if ~truths(i).active, continue, end
                            anyD = min(anyD, hypot(tr(j).x - truths(i).x, ...
                                                   tr(j).y - truths(i).y));
                        end
                        if anyD <= matchGate
                            nDuplicate(c) = nDuplicate(c) + 1;
                        else
                            nFalse(c) = nFalse(c) + 1;
                        end
                    end
                end

                % Range at which an in-view truth went unseen: this is what
                % separates "the tracker failed" from "the camera physically
                % cannot resolve a 0.6 m pedestrian at 40 m".
                for i = find(inView & ~matchedTruth)
                    missRange(end+1) = hypot(truths(i).x - st.x, ...
                                             truths(i).y - st.y); %#ok<AGROW>
                end
            end

            ps.cycles = nCyc;
            ps.meanTracks = mean(nTracks);
            ps.meanInView = mean(nInView);
            ps.meanMatched = mean(nMatched);
            if sum(byClassInView) > 0
                % Same numerator/denominator population as the per-class rows.
                ps.recall = sum(byClassMatched) / sum(byClassInView);
            end
            if sum(nTracks) > 0
                ps.falseTrackRate = sum(nFalse) / sum(nTracks);
                ps.duplicateTrackRate = sum(nDuplicate) / sum(nTracks);
            end
            if ~isempty(sqErr)
                ps.posRMSE = sqrt(mean(sqErr));
                ps.posMaxErr = sqrt(max(sqErr));
            end
            if ~isempty(sqVelErr)
                ps.velRMSE = sqrt(mean(sqVelErr));
            end
            if classSeen > 0
                ps.classAccuracy = classRight / classSeen;
            end
            total = classSeen + classUnknown;
            if total > 0
                ps.classUnknownRate = classUnknown / total;
            end

            ps.byClassInView = byClassInView;
            ps.byClassMatched = byClassMatched;
            ps.byClassRecall = nan(1, 7);
            seen = byClassInView > 0;
            ps.byClassRecall(seen) = byClassMatched(seen) ./ byClassInView(seen);
            if ~isempty(matchRange)
                ps.meanMatchRange = mean(matchRange);
                ps.maxMatchRange = max(matchRange);
            end
            if ~isempty(missRange)
                ps.meanMissRange = mean(missRange);
            end
        end

        % ----------------------------------------------------------------
        % ----------------------------------------------------------------
        function sb = stageBreakdown(log)
            %STAGEBREAKDOWN  Median and p95 per pipeline stage, in ms.
            %
            %   Answers the question a single end-to-end latency cannot: WHICH
            %   stage to fix.  Medians are reported alongside p95 because they
            %   answer different questions - the median is where the time
            %   normally goes, the p95 is what breaks the budget, and on this
            %   pipeline they do not point at the same stage.
            %
            %   'unaccounted' is deliberately reported.  It is the cycle total
            %   minus the sum of the stages, i.e. logging, the stage timers
            %   themselves and anything not yet instrumented.  A large value
            %   would mean this table is lying about where the time goes, so it
            %   is shown rather than silently absorbed into a stage.
            sb = MetricsLogger.blankStageBreakdown();
            if log.nCycles == 0 || ~isfield(log.cycles, 'stageMs')
                return
            end
            keep = ~cellfun(@isempty, {log.cycles.stageMs});
            if ~any(keep)
                return
            end
            S = [log.cycles(keep).stageMs];
            names = SimEngine.stageNames();
            total = zeros(numel(S), 1);
            for i = 1:numel(names)
                v = [S.(names{i})]';
                sb.([names{i} '_p50']) = median(v);
                sb.([names{i} '_p95']) = prctile(v, 95);
                total = total + v;
            end
            lat = [log.cycles(keep).latencyMs]';
            sb.unaccounted_p50 = median(max(lat - total, 0));
            sb.cycles = numel(S);
        end

        function sb = blankStageBreakdown()
            %BLANKSTAGEBREAKDOWN  NaN per stage: no run means not measured.
            %
            %   NaN here, zero in SimEngine.blankStageMs, and the difference
            %   matters: a stage that ran and took no measurable time is 0, a
            %   stage nobody has measured is NaN.
            names = SimEngine.stageNames();
            sb = struct('cycles', 0, 'unaccounted_p50', NaN);
            for i = 1:numel(names)
                sb.([names{i} '_p50']) = NaN;
                sb.([names{i} '_p95']) = NaN;
            end
        end

        % ----------------------------------------------------------------
        function ps = blankPerception()
            ps = struct('cycles', 0, 'meanTracks', NaN, 'meanInView', NaN, ...
                'meanMatched', NaN, 'recall', NaN, 'falseTrackRate', NaN, ...
                'duplicateTrackRate', NaN, ...
                'posRMSE', NaN, 'posMaxErr', NaN, 'velRMSE', NaN, ...
                'classAccuracy', NaN, 'classUnknownRate', NaN, ...
                'byClassInView', zeros(1, 7), 'byClassMatched', zeros(1, 7), ...
                'byClassRecall', nan(1, 7), ...
                'meanMatchRange', NaN, 'maxMatchRange', NaN, 'meanMissRange', NaN);
        end

        % ----------------------------------------------------------------
        function tf = isInSensorFootprint(st, truth, cfg)
            %ISINSENSORFOOTPRINT  Geometry only: FOV and range, no occlusion.
            c = cfg.sensors.camera;
            r = cfg.sensors.radar;

            camO = [st.x, st.y] + cfg.vehicle.wheelbase * [cos(st.yaw), sin(st.yaw)];
            radO = [st.x, st.y] + (cfg.vehicle.length - cfg.vehicle.rearOverhang) * ...
                [cos(st.yaw), sin(st.yaw)];

            tf = inCone(camO, st.yaw, [truth.x truth.y], c.fov, c.range) || ...
                 inCone(radO, st.yaw, [truth.x truth.y], r.fovLong, r.rangeLong) || ...
                 inCone(radO, st.yaw, [truth.x truth.y], r.fovShort, r.rangeShort);
        end

        % ----------------------------------------------------------------
        function [kappa, dkds] = pathCurvature(x, y)
            %PATHCURVATURE  Curvature of the driven path and its rate.
            dx = gradient(x);
            dy = gradient(y);
            ddx = gradient(dx);
            ddy = gradient(dy);
            denom = (dx.^2 + dy.^2).^1.5;

            kappa = zeros(size(x));
            ok = denom > 1e-9;     % undefined while stationary
            kappa(ok) = (dx(ok).*ddy(ok) - dy(ok).*ddx(ok)) ./ denom(ok);

            ds = hypot(dx, dy);
            dkds = zeros(size(kappa));
            good = ds > 1e-6;
            dk = gradient(kappa);
            dkds(good) = dk(good) ./ ds(good);
        end

        % ----------------------------------------------------------------
        function c = countEvents(log, type)
            if isempty(log.eventLog)
                c = 0;
                return
            end
            c = sum(strcmp({log.eventLog.type}, type));
        end
    end
end

% ========================================================================
function s = fmt(v, unit, dec)
if isnan(v)
    s = 'not measured';
else
    s = sprintf(['%.' num2str(dec) 'f %s'], v, unit);
end
end

function s = fmtInt(v)
if isnan(v)
    s = 'not measured';
else
    s = sprintf('%d', round(v));
end
end

function s = fmtPct(v)
if isnan(v)
    s = 'not measured';
else
    s = sprintf('%.1f %%', 100 * v);
end
end

function s = yn(tf)
if tf
    s = 'YES';
else
    s = 'NO';
end
end

function tf = inCone(origin, yaw, target, fov, range)
%INCONE  Is a point inside a sensor's field of view and range?
rel = target - origin;
d = hypot(rel(1), rel(2));
if d > range
    tf = false;
    return
end
if d < 1e-6
    tf = true;
    return
end
ang = mod(atan2(rel(2), rel(1)) - yaw + pi, 2*pi) - pi;
tf = abs(ang) <= fov / 2;
end
