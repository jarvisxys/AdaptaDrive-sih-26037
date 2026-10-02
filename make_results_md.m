function file = make_results_md(varargin)
%MAKE_RESULTS_MD  Write docs/RESULTS.md from the measured CSVs.
%
%   The results table is GENERATED from results/summary.csv, never typed.  A
%   table a human retypes is a table a human can mistype, and the one number
%   nobody checks is the one on the slide.
%
%   MAKE_RESULTS_MD() reads results/summary.csv and results/runs.csv.
%
%   See also RUN_EXPERIMENTS, MAKE_FIGURES.

p = inputParser;
p.addParameter('resultsDir', adRoot('results'));
p.addParameter('out', adRoot('RESULTS.md'));
p.parse(varargin{:});
opt = p.Results;

sumFile = fullfile(opt.resultsDir, 'summary.csv');
runsFile = fullfile(opt.resultsDir, 'runs.csv');
if ~isfile(sumFile) || ~isfile(runsFile)
    error('make_results_md:noResults', ...
        'No results at %s. Run run_experiments first.', opt.resultsDir);
end

S = readtable(sumFile);
T = readtable(runsFile);
% Normalise text columns: readtable returns cellstr by default while
% run_experiments writes them from string arrays.
S = convertvars(S, vartype('cellstr'), 'string');
T = convertvars(T, vartype('cellstr'), 'string');

% UTF-8 explicitly: fopen defaults to the system encoding on Windows, which
% turns every em-dash and plus-minus in this file into mojibake.
fid = fopen(opt.out, 'w', 'n', 'UTF-8');
closer = onCleanup(@() fclose(fid));

w = @(varargin) fprintf(fid, varargin{:});

w('# AdaptaDrive -- measured results\n\n');
w('**Generated from `results/summary.csv` by `make_results_md`.**\n');
w('Not typed by hand. Re-run `run_experiments` then `make_results_md` to refresh.\n\n');

w('> Simulation results. Architectural coverage is not validated performance.\n\n');

w('- runs: **%d**    seeds: **%s**    generated: %s\n', ...
    height(T), mat2str(unique(T.seed)'), string(datetime('now'), 'yyyy-MM-dd HH:mm'));
w('- machine: %s\n', string(T.cpu(1)));
w('- MATLAB: R%s\n', string(T.matlabRelease(1)));
if any(T.stallFlag)
    w('- **%d run(s) flagged as machine stalls** (wall time with no matching CPU time); their latency is suspect.\n', ...
        sum(T.stallFlag));
end
w('\n---\n\n');

% ---------------------------------------------------------------- headline
w('## Collision-free completion\n\n');
w('Wilson 95%% intervals, because N is small: a bare percentage from a\n');
w('handful of runs implies precision the sample cannot support.\n\n');
w('| scenario | config | n | completion | 95%% CI | collisions | timeouts | off-road |\n');
w('|---|---|---|---|---|---|---|---|\n');
for k = 1:height(S)
    w('| %s | %s | %d | %.0f%% | %.0f-%.0f%% | %d | %d | %d |\n', ...
        S.scenario(k), S.config(k), S.n(k), 100*S.completionRate(k), ...
        100*S.wilsonLo(k), 100*S.wilsonHi(k), ...
        S.collisions(k), S.timeouts(k), S.offroad(k));
end

% ---------------------------------------------------------------- safety
w('\n## Safety and comfort (mean +/- std)\n\n');
w('| scenario | config | completion time (s) | mean speed (m/s) | min clearance (m) | min TTC (s) | jerk RMS (m/s^3) |\n');
w('|---|---|---|---|---|---|---|\n');
for k = 1:height(S)
    w('| %s | %s | %s | %s | %s | %s | %s |\n', ...
        S.scenario(k), S.config(k), ...
        pm(S.completionTime_mean(k), S.completionTime_std(k)), ...
        pm(S.meanSpeed_mean(k), S.meanSpeed_std(k)), ...
        pm(S.minClearance_mean(k), S.minClearance_std(k)), ...
        pm(S.minTTC_mean(k), S.minTTC_std(k)), ...
        pm(S.jerkRMS_mean(k), S.jerkRMS_std(k)));
end

% ---------------------------------------------------------------- latency
w('\n## Planning-cycle latency vs the 200 ms target\n\n');
w('Interpreted MATLAB on %s. Not an embedded target.\n\n', string(T.cpu(1)));

sc = unique(T.scenario, 'stable');

% Prefer the NAMED file. Reading the bare matrix means matching it to the
% scenario list by row order, and the two files are written by different
% scripts - reorder one and every figure below silently attaches to the wrong
% scenario. The named file carries its own labels.
isoNamed = fullfile(opt.resultsDir, 'latency_isolated_named.csv');
isoFile = fullfile(opt.resultsDir, 'latency_isolated.csv');
isoRows = [];
isoSpread = [];
isoReps = 1;
if isfile(isoNamed)
    In = readtable(isoNamed);
    In = convertvars(In, vartype('cellstr'), 'string');
    isoRows = [In.p50_ms, In.p95_ms, In.max_ms, In.pct_under_200];
    isoNames = In.scenario;
    if all(ismember({'p95_lo', 'p95_hi'}, In.Properties.VariableNames))
        isoSpread = [In.p95_lo, In.p95_hi];
    end
    if ismember('repeats', In.Properties.VariableNames)
        isoReps = In.repeats(1);
    end
elseif isfile(isoFile)
    isoRows = readmatrix(isoFile);
    isoNames = sc(1:min(numel(sc), size(isoRows, 1)));
    isoRows = isoRows(1:numel(isoNames), :);
end

if ~isempty(isoRows)
    w('**Measured in isolation** -- one scenario at a time with nothing else\n');
    w('running. These are the figures to quote.\n\n');
    if isoReps > 1
        w('Median of **%d repeats** per scenario, with the p95 range across\n', isoReps);
        w('them. The run is deterministic - same path every time - so the\n');
        w('spread is measurement noise, not different driving. It is shown\n');
        w('because a single pass put highway at 178 ms and then 217 ms, either\n');
        w('side of the target, and a verdict from one pass would be noise\n');
        w('presented as a result.\n\n');
        w('| scenario | p50 (ms) | p95 (ms) | p95 range | max (ms) | under 200 ms | meets target? |\n');
        w('|---|---|---|---|---|---|---|\n');
    else
        w('| scenario | p50 (ms) | p95 (ms) | max (ms) | under 200 ms | meets target? |\n');
        w('|---|---|---|---|---|---|\n');
    end
    nMet = 0; nBorder = 0;
    for k = 1:numel(isoNames)
        if isoRows(k,2) < 200
            verdict = 'YES'; nMet = nMet + 1;
        else
            verdict = '**NO**';
        end
        % A scenario whose range straddles 200 ms has no stable verdict, and
        % saying so is more honest than reporting whichever side the median
        % happened to land on.
        if ~isempty(isoSpread) && isoSpread(k,1) < 200 && isoSpread(k,2) >= 200
            verdict = [verdict ' (borderline)']; %#ok<AGROW>
            nBorder = nBorder + 1;
        end
        if isoReps > 1
            w('| %s | %.0f | %.0f | %.0f-%.0f | %.0f | %.1f%% | %s |\n', ...
                isoNames(k), isoRows(k,1), isoRows(k,2), ...
                isoSpread(k,1), isoSpread(k,2), isoRows(k,3), ...
                isoRows(k,4), verdict);
        else
            w('| %s | %.0f | %.0f | %.0f | %.1f%% | %s |\n', ...
                isoNames(k), isoRows(k,1), isoRows(k,2), isoRows(k,3), ...
                isoRows(k,4), verdict);
        end
    end
    w('\n**%d of %d scenarios meet the 200 ms p95 target.**\n', ...
        nMet, numel(isoNames));
    if nBorder > 0
        w('%d of the verdicts is borderline: its p95 range crosses 200 ms, so\n', ...
            nBorder);
        w('which side it lands on depends on the pass rather than on the code.\n');
    end
    w('The target is not relaxed to improve that count; the misses are\n');
    w('reported as misses.\n\n');
end

stgFile = fullfile(opt.resultsDir, 'latency_stages.csv');
if isfile(stgFile)
    St = readtable(stgFile);
    St = convertvars(St, vartype('cellstr'), 'string');
    names = SimEngine.stageNames();
    w('**Where the time goes**, per stage, same isolated runs. A single\n');
    w('end-to-end number says a cycle missed the budget; it does not say\n');
    w('which stage to fix. Median milliseconds.\n\n');
    w('| scenario |');
    for i = 1:numel(names), w(' %s |', names{i}); end
    w(' unacct | total |\n|---|');
    w(repmat('---|', 1, numel(names) + 2));
    w('\n');
    for k = 1:height(St)
        w('| %s |', St.scenario(k));
        tot = 0;
        for i = 1:numel(names)
            v = St.([names{i} '_p50'])(k);
            w(' %.1f |', v);
            tot = tot + v;
        end
        w(' %.1f | %.1f |\n', St.unaccounted_p50(k), tot);
    end
    w('\nThe unaccounted column is the cycle total minus the sum of the\n');
    w('stages, per cycle. It is reported rather than absorbed into a stage,\n');
    w('so this table cannot quietly stop adding up.\n\n');
end

w('**Measured during the batch** -- shown for completeness, and higher.\n');
w('This machine has 7.7 GB of RAM and under 1 GB free during a run of the\n');
w('matrix, so batch figures include time the process spent contending for\n');
w('memory rather than planning. They are reported rather than dropped, but\n');
w('the isolated numbers above are the honest measure of the planner.\n\n');
w('| scenario | p50 (ms) | p95 (ms) |\n');
w('|---|---|---|\n');
for k = 1:numel(sc)
    sel = T.scenario == sc(k) & T.config == "PROPOSED" & ~T.stallFlag;
    w('| %s | %.0f | %.0f |\n', sc(k), ...
        mean(T.latencyP50(sel), 'omitnan'), mean(T.latencyP95(sel), 'omitnan'));
end

% ---------------------------------------------------------------- A1
w('\n## Requirement A1 -- surface hazards\n\n');
w('| scenario | pothole traversals (total) | min pothole clearance (m) |\n');
w('|---|---|---|\n');
for k = 1:numel(sc)
    sel = T.scenario == sc(k) & T.config == "PROPOSED";
    tr = sum(T.hazardTraversals(sel), 'omitnan');
    mc = min(T.minPotholeClearance(sel), [], 'omitnan');
    w('| %s | %d | %s |\n', sc(k), tr, num2strOrDash(mc));
end
w('\nA1 asks for zero traversals and at least 0.5 m clearance. The numbers\n');
w('above are reported as measured, pass or fail.\n');

% ---------------------------------------------------------------- A2/A3
w('\n## Requirements A2 and A3 -- detectors\n\n');
w('| scenario | wrong-way flags | merge detections |\n');
w('|---|---|---|\n');
for k = 1:numel(sc)
    sel = T.scenario == sc(k) & T.config == "PROPOSED";
    w('| %s | %d | %d |\n', sc(k), ...
        sum(T.wrongWayFlags(sel), 'omitnan'), ...
        sum(T.mergeDetections(sel), 'omitnan'));
end
w('\nVillage and cattle contain no wrong-way vehicle and no merging vehicle:\n');
w('zero there is the correct answer, and is the false-positive check.\n');

% ---------------------------------------------------------------- honesty
w('\n---\n\n## How to reproduce\n\n');
w('```matlab\nstartup\nrun_experiments(''seeds'', %s)\nmake_figures\nmake_results_md\n```\n', ...
    mat2str(unique(T.seed)'));
w('\nEvery row above came from a simulation that ran. Runs that errored are\n');
w('recorded with outcome `error` and NaN metrics rather than dropped.\n');

fprintf('  wrote %s\n', opt.out);
file = opt.out;
end

% ========================================================================
function s = pm(m, sd)
if isnan(m)
    s = '--';
elseif isnan(sd) || sd == 0
    s = sprintf('%.2f', m);
else
    s = sprintf('%.2f +/- %.2f', m, sd);
end
end

function s = num2strOrDash(v)
if isempty(v) || isnan(v)
    s = '--';
else
    s = sprintf('%.2f', v);
end
end
