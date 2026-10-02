function summary = run_experiments(varargin)
%RUN_EXPERIMENTS  Run the experiment matrix and write results/*.csv.
%
%   RUN_EXPERIMENTS()                      the default matrix
%   RUN_EXPERIMENTS('seeds', 1:10)         choose the seeds
%   RUN_EXPERIMENTS('scenarios', {'village'}, 'configs', {'PROPOSED','BL1'})
%   RUN_EXPERIMENTS('ablations', true)     include the ablation configurations
%
%   Writes
%     results/runs.csv     one row per run, every metric, with provenance
%     results/summary.csv  grouped mean +- std, n, and a Wilson 95% interval
%                          on the collision-free completion rate
%     logs/<scenario>_<config>_seed<logSeed>.mat   for replay and video
%
%   HONESTY
%   Every row is produced by a simulation that actually ran.  A run that
%   errors is recorded with outcome "error" and its metrics as NaN - never
%   dropped, because silently dropping failures is how a completion rate
%   becomes a lie.  The Wilson interval is reported because N is small and a
%   bare percentage of ten runs invites more confidence than it has earned.
%
%   Runs SERIALLY by default.  This machine has 7.7 GB of RAM and each worker
%   carries its own scenario, risk grids and sensor objects; measured memory
%   is printed at the start and every 20 runs so the choice can be revisited
%   against evidence rather than assumption.
%
%   See also RUN_DEMO, MAKE_FIGURES, CONFIGPRESET.

p = inputParser;
p.addParameter('scenarios', {'village', 'intersection', 'highway', 'market', 'cattle'});
p.addParameter('configs',   {'PROPOSED', 'BL1', 'BL2', 'BL3'});
p.addParameter('seeds',     1:10);
p.addParameter('density',   1.0);
p.addParameter('ablations', false);
p.addParameter('logSeed',   1);
p.addParameter('outDir',    adRoot('results'));
% Where the seed-logSeed replay logs are written. Explicit, and separate from
% outDir, because these two must never be assumed to travel together: a caller
% that redirects outDir to keep its CSVs apart (run_stress does, per condition)
% would otherwise still write its logs over the main matrix's, and the saved
% run would stop matching the row in results/runs.csv that names it. That
% happened: a density x2 intersection run overwrote the nominal one, and the
% video renderer then replayed a stressed run labelled PROPOSED.
p.addParameter('logDir',    adRoot('logs'));
p.parse(varargin{:});
opt = p.Results;

configs = opt.configs;
if opt.ablations
    configs = [configs, {'-riskmap', '-classpriors', '-uncertainty', ...
                         '-context', '-wrongside'}];
end

if ~isfolder(opt.outDir), mkdir(opt.outDir); end
logDir = opt.logDir;
if ~isfolder(logDir), mkdir(logDir); end

env = check_env('print', false);
nRuns = numel(opt.scenarios) * numel(configs) * numel(opt.seeds);

line = repmat('=', 1, 78);
fprintf('\n%s\n  AdaptaDrive experiment matrix  (SIMULATION)\n%s\n', line, line);
fprintf('  scenarios : %s\n', strjoin(opt.scenarios, ', '));
fprintf('  configs   : %s\n', strjoin(configs, ', '));
fprintf('  seeds     : %s   (density %.1f)\n', mat2str(opt.seeds), opt.density);
fprintf('  total     : %d runs, serial\n', nRuns);
fprintf('  machine   : %s, %s\n', env.platform.cpu, memString());
fprintf('%s\n\n', line);

rows = cell(nRuns, 1);
k = 0;
tStart = tic;

for iS = 1:numel(opt.scenarios)
    for iC = 1:numel(configs)
        for iD = 1:numel(opt.seeds)
            k = k + 1;
            scenario = opt.scenarios{iS};
            config   = configs{iC};
            seed     = opt.seeds(iD);

            tRun = tic;
            cpu0 = cputime;
            try
                cfg = configPreset(config, 'scenario', scenario, ...
                    'seed', seed, 'density', opt.density);
                s = buildScenario(scenario, seed, opt.density, cfg);
                res = SimEngine(s, cfg).run();
                m = res.metrics;
                ok = true;

                if seed == opt.logSeed
                    res.log.save(fullfile(logDir, ...
                        sprintf('%s_%s_seed%d.mat', scenario, config, seed)));
                end
            catch ME
                % Recorded, never dropped.
                m = MetricsLogger.blank();
                m.scenario = scenario; m.config = config; m.seed = seed;
                m.density = opt.density;
                m.outcome = "error";
                ok = false;
                fprintf('    !! %s / %s / seed %d errored: %s\n', ...
                    scenario, config, seed, ME.message);
            end

            wall = toc(tRun);
            cpuUsed = cputime - cpu0;

            % A stall is wall-clock time with no matching CPU time: the
            % process was descheduled, not working.  Flagged rather than
            % averaged into the latency figures.
            stall = wall > 5 && cpuUsed > 0 && (wall / max(cpuUsed, 1e-6)) > 10;

            rows{k} = toRow(m, env, wall, cpuUsed, stall, ok);

            pct = 100 * k / nRuns;
            elapsed = toc(tStart);
            eta = elapsed / k * (nRuns - k);
            fprintf('  [%3.0f%%] %-13s %-13s seed %2d  %-9s %6.1fs  (ETA %4.1f min)%s\n', ...
                pct, scenario, config, seed, string(m.outcome), wall, eta/60, ...
                stallTag(stall));

            % Flush after every run.  A matrix takes hours on this machine, and
            % writing only at the end means an interruption at run 130 of 135
            % loses every measurement taken.  The partial CSV is also the only
            % way to see progress: MATLAB block-buffers stdout when it is
            % redirected to a file, so the console log can lag by many runs.
            %
            % Cost is one writetable per run against a ~100 s run. It is not
            % measurable, and it is inside neither the latency timing nor the
            % cycle budget.
            flushRuns(rows(1:k), opt.outDir);

            if mod(k, 20) == 0
                fprintf('         memory: %s\n', memString());
            end
        end
    end
end

T = vertcat(rows{:});
runsFile = fullfile(opt.outDir, 'runs.csv');
writetable(T, runsFile);

% The matrix completed, so the partial file has no further purpose and would
% only invite someone to read a superseded table.
partial = fullfile(opt.outDir, 'runs_partial.csv');
if isfile(partial)
    delete(partial);
end

S = summarise(T);
summaryFile = fullfile(opt.outDir, 'summary.csv');
writetable(S, summaryFile);

fprintf('\n%s\n', line);
fprintf('  %d runs in %.1f min\n', nRuns, toc(tStart)/60);
fprintf('  runs    -> %s\n', runsFile);
fprintf('  summary -> %s\n', summaryFile);
if any(T.stallFlag)
    fprintf('  !! %d run(s) flagged as machine stalls; their latency is suspect.\n', ...
        sum(T.stallFlag));
end
fprintf('%s\n\n', line);

printSummary(S);

if nargout > 0
    summary = S;
end
end

% ========================================================================
function flushRuns(rowsSoFar, outDir)
%FLUSHRUNS  Write the runs completed so far, so an interruption keeps them.
%
%   Summary statistics are NOT written here: a Wilson interval over a partial
%   cell would be a real number computed from an incomplete sample, and someone
%   reading summary.csv would have no way to tell. The partial file is
%   deliberately runs-only, and `summary.csv` appears when the matrix finishes.
T = vertcat(rowsSoFar{:});
try
    writetable(T, fullfile(outDir, 'runs_partial.csv'));
catch ME
    % A failed flush must never kill the matrix it is protecting.
    fprintf('         (flush failed: %s)\n', ME.message);
end
end

% ========================================================================
function row = toRow(m, env, wall, cpuUsed, stall, ok)
row = table();
row.scenario   = string(m.scenario);
row.config     = string(m.config);
row.seed       = m.seed;
row.density    = m.density;
row.outcome    = string(m.outcome);
row.completed  = logical(m.completed);
row.collision  = logical(m.collision);
row.timeoutFail = logical(m.timeoutFail);
row.offroadFail = logical(m.offroadFail);
row.completionTime = m.completionTime;
row.pathLength     = m.pathLength;
row.meanSpeed      = m.meanSpeed;
row.minClearance   = m.minClearance;
row.minTTC         = m.minTTC;
row.hazardTraversals    = m.hazardTraversals;
row.minPotholeClearance = m.minPotholeClearance;
row.jerkRMS    = m.jerkRMS;
row.jerkMax    = m.jerkMax;
row.latAccRMS  = m.latAccRMS;
row.curvatureMean = m.curvatureMean;
row.emergencyBrakes = m.emergencyBrakes;
row.wrongWayFlags   = m.wrongWayFlags;
row.mergeDetections = m.mergeDetections;
row.replanCount     = m.replanCount;
row.latencyP50 = m.latencyP50;
row.latencyP95 = m.latencyP95;
row.latencyMax = m.latencyMax;
row.latencyUnder200 = m.latencyUnder200;

% Per-stage latency (user decision 5).  One end-to-end number says a cycle
% missed the budget; these say which stage to fix.  Written flat as
% stage_<name>_p50/p95 so the CSV stays a plain table.
if isfield(m, 'stageMs') && isstruct(m.stageMs)
    for nm = string(SimEngine.stageNames())
        row.("stage_" + nm + "_p50") = m.stageMs.(nm + "_p50");
        row.("stage_" + nm + "_p95") = m.stageMs.(nm + "_p95");
    end
    row.stage_unaccounted_p50 = m.stageMs.unaccounted_p50;
end

row.wallSeconds = wall;
row.cpuSeconds  = cpuUsed;
row.stallFlag   = stall;
row.rerunCount  = 0;
row.ranOk       = ok;
row.matlabRelease = string(env.platform.matlabRelease);
row.cpu         = string(env.platform.cpu);
row.gitCommit   = string(env.gitCommit);
row.timestamp   = string(datetime('now'), 'yyyy-MM-dd HH:mm:ss');
end

% ========================================================================
function S = summarise(T)
%SUMMARISE  Grouped statistics with a Wilson interval on completion.
[g, sc, cf] = findgroups(T.scenario, T.config);

S = table();
S.scenario = sc;
S.config   = cf;
S.n        = splitapply(@numel, T.seed, g);

success = T.completed & ~T.collision;
S.completionRate = splitapply(@mean, double(success), g);
[lo, hi] = arrayfun(@(i) wilson(sum(success(g == i)), sum(g == i)), ...
    (1:max(g))');
S.wilsonLo = lo;
S.wilsonHi = hi;

S.collisions = splitapply(@sum, double(T.collision), g);
S.timeouts   = splitapply(@sum, double(T.timeoutFail), g);
S.offroad    = splitapply(@sum, double(T.offroadFail), g);

for f = ["completionTime", "meanSpeed", "minClearance", "minTTC", ...
         "jerkRMS", "emergencyBrakes", "latencyP50", "latencyP95"]
    S.(f + "_mean") = splitapply(@(x) mean(x, 'omitnan'), T.(f), g);
    S.(f + "_std")  = splitapply(@(x) std(x, 'omitnan'), T.(f), g);
end
end

% ========================================================================
function [lo, hi] = wilson(k, n)
%WILSON  95% Wilson score interval for a binomial proportion.
%   Reported because N is small: a bare "80%" from ten runs implies a
%   precision the sample cannot support.
if n == 0
    lo = NaN; hi = NaN; return
end
z = 1.959963984540054;
phat = k / n;
denom = 1 + z^2/n;
centre = (phat + z^2/(2*n)) / denom;
half = z * sqrt(phat*(1-phat)/n + z^2/(4*n^2)) / denom;
lo = max(centre - half, 0);
hi = min(centre + half, 1);
end

% ========================================================================
function printSummary(S)
fprintf('  %-13s %-13s %3s  %-22s %9s %9s\n', ...
    'scenario', 'config', 'n', 'collision-free (95% CI)', 'meanTime', 'minClear');
fprintf('  %s\n', repmat('-', 1, 76));
for k = 1:height(S)
    fprintf('  %-13s %-13s %3d  %5.0f%%  [%3.0f%% - %3.0f%%]      %8.1fs %9.2f\n', ...
        S.scenario(k), S.config(k), S.n(k), 100*S.completionRate(k), ...
        100*S.wilsonLo(k), 100*S.wilsonHi(k), ...
        S.completionTime_mean(k), S.minClearance_mean(k));
end
fprintf('\n  Simulation results. Architectural coverage is not validated performance.\n\n');
end

% ========================================================================
function s = memString()
try
    [~, sv] = memory; %#ok<MEMOR>
    s = sprintf('%.1f GB free of %.1f GB', ...
        sv.PhysicalMemory.Available/2^30, sv.PhysicalMemory.Total/2^30);
catch
    s = 'memory unavailable';
end
end

function s = stallTag(stall)
if stall
    s = '  [STALL]';
else
    s = '';
end
end
