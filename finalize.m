function finalize(varargin)
%FINALIZE  Produce every submission artefact from the measured results.
%
%   FINALIZE() assumes run_experiments has already written results/*.csv and
%   then generates, in order:
%
%     1. isolated latency   results/latency_isolated.csv, latency_stages.csv
%     2. figures            results/figures/*.png
%     3. RESULTS.md         generated from summary.csv, never typed
%     4. demo replay logs   full-detail logs for the UI and the video
%     5. demo video         results/video/*.mp4
%     6. UI screenshots     results/figures/app_*.png
%
%   Latency is re-measured FIRST and in isolation, because the figures and
%   RESULTS.md both read the CSV it writes. Measuring it during the matrix would
%   charge the planner for time the process spent contending for memory: this
%   machine has 7.7 GB and under 1 GB free while a matrix runs.
%
%   Each step is independent: one failing does not stop the rest, and each
%   reports what it did or why it could not. Nothing here invents a result -
%   every artefact is derived from a run that happened.
%
%   FINALIZE('skipVideo', true) leaves out the slowest step.
%   FINALIZE('skipLatency', true) keeps the existing latency CSVs.
%
%   See also RUN_EXPERIMENTS, MEASURE_LATENCY, MAKE_FIGURES, MAKE_RESULTS_MD.

p = inputParser;
p.addParameter('skipVideo', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('skipLatency', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('demoScenarios', {'village', 'cattle'});
p.addParameter('demoSeed', 1);
p.parse(varargin{:});
opt = p.Results;

line = repmat('=', 1, 74);
fprintf('\n%s\n  AdaptaDrive - building submission artefacts\n%s\n', line, line);

ok = struct();

% ---------------------------------------------------------------- latency
if logical(opt.skipLatency)
    fprintf('\n  [skip] isolated latency (skipLatency requested)\n');
    ok.latency = true;
else
    ok.latency = step('isolated latency', @() measure_latency());
end

% ---------------------------------------------------------------- figures
ok.figures = step('figures', @() make_figures());

% ---------------------------------------------------------------- results
ok.results = step('RESULTS.md', @() make_results_md());

% ---------------------------------------------------------------- logs
ok.logs = step('demo replay logs', @() makeDemoLogs(opt));

% ---------------------------------------------------------------- video
if logical(opt.skipVideo)
    fprintf('\n  [skip] demo video (skipVideo requested)\n');
    ok.video = true;
else
    ok.video = step('demo video', @() ...
        make_video(opt.demoScenarios{1}, 'PROPOSED', opt.demoSeed, ...
                   'compare', true, 'maxSeconds', 40));
end

% ---------------------------------------------------------------- UI shots
ok.ui = step('UI screenshots', @() captureUI());

% ---------------------------------------------------------------- summary
fprintf('\n%s\n  ARTEFACT SUMMARY\n%s\n', line, line);
f = fieldnames(ok);
for k = 1:numel(f)
    if ok.(f{k})
        fprintf('  [ ok ]  %s\n', f{k});
    else
        fprintf('  [FAIL]  %s\n', f{k});
    end
end
fprintf('%s\n\n', line);
end

% ========================================================================
function good = step(name, fn)
fprintf('\n  --- %s ---\n', name);
good = true;
try
    fn();
catch ME
    good = false;
    fprintf('  !! %s failed: %s\n', name, ME.message);
    if ~isempty(ME.stack)
        fprintf('     at %s line %d\n', ME.stack(1).name, ME.stack(1).line);
    end
end
end

% ========================================================================
function makeDemoLogs(opt)
%MAKEDEMOLOGS  Full-detail logs for replay and filming.
%
%   The experiment matrix writes 'light' logs, which store the risk field and
%   candidate fan only every fifth cycle to keep a batch inside memory. The
%   demo wants every frame, so these are regenerated at full detail.
for k = 1:numel(opt.demoScenarios)
    sc = opt.demoScenarios{k};
    for cfgName = {'PROPOSED', 'BL1'}
        fprintf('    %s / %s / seed %d ... ', sc, cfgName{1}, opt.demoSeed);
        cfg = configPreset(cfgName{1}, 'scenario', sc, 'seed', opt.demoSeed);
        cfg.io.logDetail = 'full';
        s = buildScenario(sc, opt.demoSeed, 1.0, cfg);
        res = SimEngine(s, cfg).run();
        f = fullfile(adRoot('logs'), sprintf('%s_%s_seed%d.mat', ...
            sc, cfgName{1}, opt.demoSeed));
        res.log.save(f);
        fprintf('%s (%.0f m in %.0f s)\n', res.outcome.reason, ...
            res.metrics.pathLength, res.metrics.completionTime);
    end
end
end

% ========================================================================
function captureUI()
%CAPTUREUI  Screenshots of each tab, for the report and the slides.
app = AdaptaDriveApp;
cleanup = onCleanup(@() delete(app));
drawnow; pause(1.5);

tabs = app.tabs.Children;
names = {'live', 'results', 'architecture', 'scenarios'};
for k = 1:min(numel(tabs), numel(names))
    app.tabs.SelectedTab = tabs(k);
    drawnow; pause(1.2);
    f = adRoot('results', 'figures', sprintf('app_%s.png', names{k}));
    exportapp(app.fig, f);
    fprintf('    %s\n', f);
end
end
