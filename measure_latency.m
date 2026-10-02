function measure_latency(varargin)
%MEASURE_LATENCY  Isolated per-scenario latency, with the per-stage split.
%
%   Latency measured during a matrix run is contaminated: this machine has
%   7.7 GB of RAM and under 1 GB free while the matrix runs, so a batch p95
%   includes time the process spent contending for memory rather than
%   planning.  This script measures one scenario at a time with nothing else
%   running, which is the figure worth quoting, and records where the time
%   actually goes stage by stage.
%
%   Writes results/latency_isolated.csv  [p50 p95 max pctUnder200] per scenario
%          results/latency_stages.csv    per-stage p50/p95 per scenario
%
%   Both are read by MAKE_RESULTS_MD.  Nothing here is typed into a document.
%
%   See also RUN_EXPERIMENTS, MAKE_RESULTS_MD.

p = inputParser;
p.addParameter('scenarios', {'village', 'intersection', 'highway', 'market', 'cattle'});
p.addParameter('config', 'PROPOSED');
p.addParameter('seed', 1);
p.addParameter('density', 1.0);
p.addParameter('repeats', 3);
p.addParameter('resultsDir', adRoot('results'));
p.parse(varargin{:});
opt = p.Results;

sc = cellstr(opt.scenarios);
names = SimEngine.stageNames();
nRep = max(1, round(opt.repeats));

iso = nan(numel(sc), 4);
p95Lo = nan(numel(sc), 1);
p95Hi = nan(numel(sc), 1);
stg = nan(numel(sc), 2 * numel(names) + 1);

% REPEATS, and why they are not optional. The simulation is deterministic: the
% same scenario, config and seed drive the vehicle along exactly the same path
% every time. Only the wall-clock timing varies. Measured across two passes,
% highway's p95 came out 178 ms and then 217 ms, and intersection 313 ms then
% 401 ms - so a single pass straddles the 200 ms threshold and a
% "meets target" verdict taken from one run is a coin toss dressed up as a
% measurement. The median across repeats is reported, with the full range, so
% the reader can see how much of the verdict is noise.
fprintf('\n  Isolated latency, %s seed %d, %d repeat(s) per scenario.\n', ...
    opt.config, opt.seed, nRep);
fprintf('  The run is deterministic; only the timing varies.\n\n');

for k = 1:numel(sc)
    fprintf('  %-13s ', sc{k});
    cfg = configPreset(opt.config, 'scenario', sc{k}, 'seed', opt.seed);

    rep = nan(nRep, 4);
    repStg = nan(nRep, 2 * numel(names) + 1);
    reason = '';
    t0 = tic;
    for rr = 1:nRep
        s = buildScenario(sc{k}, opt.seed, opt.density, cfg);
        r = SimEngine(s, cfg).run();
        m = r.metrics;
        reason = r.outcome.reason;

        rep(rr, :) = [m.latencyP50, m.latencyP95, m.latencyMax, ...
                      100 * m.latencyUnder200];
        sb = m.stageMs;
        for i = 1:numel(names)
            repStg(rr, 2*i - 1) = sb.([names{i} '_p50']);
            repStg(rr, 2*i)     = sb.([names{i} '_p95']);
        end
        repStg(rr, end) = sb.unaccounted_p50;
        fprintf('.');
    end

    iso(k, :) = median(rep, 1, 'omitnan');
    p95Lo(k) = min(rep(:, 2));
    p95Hi(k) = max(rep(:, 2));
    stg(k, :) = median(repStg, 1, 'omitnan');

    fprintf(' p50 %5.0f  p95 %5.0f ms [%.0f-%.0f]   (%s, %.0f s wall)\n', ...
        iso(k,1), iso(k,2), p95Lo(k), p95Hi(k), reason, toc(t0));
end

if ~isfolder(opt.resultsDir)
    mkdir(opt.resultsDir);
end

% Written twice, deliberately. The bare matrix is kept for backward
% compatibility with anything already reading it positionally; the NAMED table
% is what MAKE_RESULTS_MD reads, because matching two files by row order is a
% silent mislabelling waiting to happen - change the scenario order in one place
% and every latency figure in RESULTS.md attaches to the wrong scenario.
writematrix(iso, fullfile(opt.resultsDir, 'latency_isolated.csv'));

Tiso = array2table(iso, 'VariableNames', ...
    {'p50_ms', 'p95_ms', 'max_ms', 'pct_under_200'});
Tiso = addvars(Tiso, string(sc(:)), 'Before', 1, 'NewVariableNames', 'scenario');
Tiso = addvars(Tiso, p95Lo, p95Hi, repmat(nRep, numel(sc), 1), ...
    string(opt.config) + strings(numel(sc), 1), ...
    repmat(opt.seed, numel(sc), 1), ...
    'NewVariableNames', {'p95_lo', 'p95_hi', 'repeats', 'config', 'seed'});
writetable(Tiso, fullfile(opt.resultsDir, 'latency_isolated_named.csv'));

% The stage table carries its scenario names, because a bare matrix whose row
% order has to be remembered is a matrix that will eventually be read wrong.
hdr = strings(1, 2 * numel(names) + 1);
for i = 1:numel(names)
    hdr(2*i - 1) = names{i} + "_p50";
    hdr(2*i)     = names{i} + "_p95";
end
hdr(end) = "unaccounted_p50";
Tst = array2table(stg, 'VariableNames', hdr);
Tst = addvars(Tst, string(sc(:)), 'Before', 1, 'NewVariableNames', 'scenario');
writetable(Tst, fullfile(opt.resultsDir, 'latency_stages.csv'));

fprintf('\n  Per-stage p50 (ms), isolated:\n\n');
fprintf('  %-13s', 'scenario');
fprintf('%8s', names{:});
fprintf('%8s%8s\n', 'unacc', 'TOTAL');
for k = 1:numel(sc)
    fprintf('  %-13s', sc{k});
    fprintf('%8.1f', stg(k, 1:2:end-1));
    fprintf('%8.1f%8.1f\n', stg(k, end), sum(stg(k, 1:2:end-1)));
end

fprintf('\n  wrote %s\n', fullfile(opt.resultsDir, 'latency_isolated.csv'));
fprintf('  wrote %s\n', fullfile(opt.resultsDir, 'latency_stages.csv'));
end
