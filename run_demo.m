function result = run_demo(scenarioName, configName, seed, varargin)
%RUN_DEMO  Run one scenario / configuration / seed headless and report it.
%
%   RUN_DEMO('village', 'scripted', 1)
%   RUN_DEMO('village', 'scripted', 1, 'density', 2, 'png', true)
%
%   Options
%     'density'  (1.0)   agent-count multiplier
%     'png'      (true)  save a bird's-eye snapshot to results/figures
%     'saveLog'  (false) save the SimLog to logs/ for replay
%     'verbose'  (false) print progress during the run
%
%   Every number printed comes from MetricsLogger reading the log this run
%   produced.  Nothing is cached, assumed or carried over between runs.
%
%   See also BUILDSCENARIO, SIMENGINE, METRICSLOGGER, RUN_EXPERIMENTS.

p = inputParser;
p.addParameter('density', 1.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('png', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('saveLog', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('verbose', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opt = p.Results;

if nargin < 3 || isempty(seed), seed = 1; end

% --- configuration --------------------------------------------------------
cfg = configPreset(char(configName), ...
    'scenario', char(scenarioName), ...
    'seed', seed, ...
    'density', opt.density);

% A log that is going to be replayed or filmed keeps every frame's
% visualisation payload. The experiment matrix uses the strided 'light' mode
% instead, because the candidate fan alone is ~43 MB per run on a machine
% with under 1 GB free during a batch.
if logical(opt.saveLog)
    cfg.io.logDetail = 'full';
end

banner(cfg);

% --- build and run --------------------------------------------------------
tBuild = tic;
scenario = buildScenario(cfg.scenario, cfg.seed, cfg.density, cfg);
buildMs = toc(tBuild) * 1000;
fprintf('  scenario built in %.0f ms  (%d agents, %d potholes, road %.1f m)\n', ...
    buildMs, numel(scenario.agents), scenario.hazards.numPotholes(), scenario.roadLength);

engine = SimEngine(scenario, cfg);
tRun = tic;
result = engine.run('verbose', logical(opt.verbose));
runMs = toc(tRun) * 1000;

fprintf('  simulated %d steps (%.2f s of sim) in %.0f ms wall clock\n', ...
    result.log.n, result.log.meta.duration, runMs);

% --- report ---------------------------------------------------------------
MetricsLogger.print(result.metrics, scaffoldNote(cfg));

% --- artefacts ------------------------------------------------------------
if logical(opt.png)
    result.pngFile = saveSnapshot(scenario, cfg, result);
    fprintf('  bird''s-eye snapshot : %s\n', result.pngFile);

    riskFile = saveRiskSnapshot(scenario, cfg, result);
    if ~isempty(riskFile)
        result.riskPngFile = riskFile;
        fprintf('  risk-field snapshot : %s\n', riskFile);
    end
end

if logical(opt.saveLog)
    if ~isfolder(cfg.io.logsDir), mkdir(cfg.io.logsDir); end
    f = fullfile(cfg.io.logsDir, sprintf('%s_%s_seed%d.mat', ...
        cfg.scenario, cfg.name, cfg.seed));
    result.logFile = result.log.save(f);
    fprintf('  replay log          : %s\n', result.logFile);
end

fprintf('\n');
end

% ========================================================================
function banner(cfg)
line = repmat('=', 1, 72);
fprintf('\n%s\n', line);
fprintf('  AdaptaDrive  |  SIMULATION  |  team SECOND INNINGS  |  SIH 2026 PS 26037\n');
fprintf('%s\n', line);
fprintf('  scenario : %s\n', cfg.scenario);
fprintf('  config   : %s\n', cfg.name);
fprintf('  seed     : %d      density: %.2f\n', cfg.seed, cfg.density);
fprintf('  backends : sensors=%s  tracker=%s  planner=%s  chart=%s\n', ...
    cfg.env.backends.sensors, cfg.env.backends.tracker, ...
    cfg.env.backends.globalPlanner, cfg.env.backends.behaviorChart);
fprintf('%s\n', line);

if strcmp(cfg.name, 'scripted')
    fprintf('  !! SCAFFOLD CONFIGURATION "scripted" (milestone M1)\n');
    fprintf('  !! Fixed reference path; longitudinal gap keeping uses GROUND TRUTH.\n');
    fprintf('  !! No perception, no risk map, no planner, no behaviour FSM.\n');
    fprintf('  !! This is NOT a reported configuration and must never appear in\n');
    fprintf('  !! results tables. It exists to exercise the loop and the vehicle model.\n');
    fprintf('%s\n', line);
end
end

% ========================================================================
function note = scaffoldNote(cfg)
if strcmp(cfg.name, 'scripted')
    note = ['"scripted" is an M1 scaffold using ground truth - not a reported ' ...
            'configuration. Latency is deliberately not measured for it.'];
else
    note = '';
end
end

% ========================================================================
function file = saveRiskSnapshot(scenario, cfg, result)
%SAVERISKSNAPSHOT  Ego-centred view of the unified risk field (B1).
%
%   Rendered from the log's stored field, so the picture is exactly what the
%   planner would have been given at that instant.  Returns '' when the
%   configuration produced no risk field.

file = '';
log = result.log;
if log.nCycles == 0 || ~isfield(log.cycles, 'risk')
    return
end

% Pick the cycle carrying the most risk near the ego - that is where the
% field is doing something worth looking at, rather than an arbitrary frame.
best = 0; bestScore = -inf;
for c = 1:log.nCycles
    R = log.cycles(c).risk;
    if isempty(R)
        continue
    end
    score = sum(double(R(:))) / numel(R);
    if score > bestScore
        bestScore = score;
        best = c;
    end
end
if best == 0
    return
end

cyc = log.cycles(best);
k = max(min(cyc.step, log.n), 1);
st = log.egoStateAt(k);

t = theme();
fig = figure('Visible', 'off', 'Color', t.bg, 'Position', [100 100 1500 900]);
ax = axes(fig); %#ok<LAXES>

r = BEVRenderer(ax, scenario, cfg);
r.create();

frame = struct( ...
    'ego', st, ...
    'agents', log.agents{k}, ...
    'trail', log.ego(1:k, 1:2), ...
    'refPath', scenario.refPath, ...
    'risk', cyc.risk, ...
    'riskMeta', cyc.riskMeta, ...
    't', cyc.t);
frame.titleText = sprintf(['%s  -  unified risk field (B1)  |  %s, seed %d  |  ' ...
    't = %.2f s  |  SIMULATION'], scenario.title, cfg.name, cfg.seed, cyc.t);

r.update(frame);

% Frame the risk window itself, not the whole road.
span = cfg.risk.aheadM + cfg.risk.behindM;
cx = st.x + (span/2 - cfg.risk.behindM) * cos(st.yaw);
cy = st.y + (span/2 - cfg.risk.behindM) * sin(st.yaw);
r.focusOn(cx, cy, span * 1.1, span * 0.66);

cb = colorbar(ax);
cb.Color = t.textDim;
cb.Label.String = 'risk (0 = clear, 1 = maximum)';
cb.Label.Color = t.text;
cb.Label.FontName = t.font;

annotation(fig, 'textbox', [0.005 0.030 0.92 0.035], ...
    'String', sprintf(['Simulation  |  risk = static hazards + edge + ' ...
    'class-weighted predicted occupancy  |  %d tracks, %d wrong-way, %d merging'], ...
    numel(cyc.tracks), numel(cyc.wrongWayIds), numel(cyc.mergeIds)), ...
    'Color', t.textDim, 'FontName', t.font, 'FontSize', 8, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left');

annotation(fig, 'textbox', [0.005 0.002 0.92 0.035], ...
    'String', sprintf(['Overlay covers drivable ground only; off-road is risk 1 ' ...
    'by definition and is the dark surround.  MATLAB R%s, %s'], ...
    cfg.env.platform.matlabRelease, cfg.env.platform.cpu), ...
    'Color', t.textFaint, 'FontName', t.font, 'FontSize', 8, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left');

file = fullfile(cfg.io.figuresDir, sprintf('%s_%s_seed%d_risk.png', ...
    cfg.scenario, cfg.name, cfg.seed));
exportgraphics(fig, file, 'Resolution', 300, 'BackgroundColor', t.bg);
close(fig);
end

% ========================================================================
function file = saveSnapshot(scenario, cfg, result)
%SAVESNAPSHOT  Whole-route bird's-eye view of the finished run.
%
%   Rendered from the LOG, not from live simulation state, so the picture and
%   the metrics describe exactly the same run.

if ~isfolder(cfg.io.figuresDir)
    mkdir(cfg.io.figuresDir);
end

t = theme();
fig = figure('Visible', 'off', 'Color', t.bg, 'Position', [100 100 1600 760]);
ax = axes(fig); %#ok<LAXES>

r = BEVRenderer(ax, scenario, cfg);
r.create();

log = result.log;
kEnd = log.n;

frame = struct( ...
    'ego', log.egoStateAt(kEnd), ...
    'agents', log.agents{kEnd}, ...
    'trail', log.ego(1:kEnd, 1:2), ...
    'refPath', scenario.refPath, ...
    't', log.t(kEnd));

frame.titleText = sprintf('%s  -  %s, seed %d  |  outcome: %s at t = %.2f s  |  SIMULATION', ...
    scenario.title, cfg.name, cfg.seed, upper(result.outcome.reason), log.t(kEnd));

r.update(frame);
r.fitAll(8);

% Match the figure to the road's aspect ratio.  'axis equal' is mandatory for
% a bird's-eye view, so a fixed-size canvas on a 250 m x 40 m road would be
% mostly empty navy.
xl = xlim(ax);
yl = ylim(ax);
aspect = diff(yl) / diff(xl);
figW = 1600;
figH = min(max(round(figW * aspect) + 200, 420), 1000);
set(fig, 'Position', [100 100 figW figH]);

% Provenance strip: every saved figure names the machine that produced it.
annotation(fig, 'textbox', [0.005 0.005 0.99 0.045], ...
    'String', sprintf(['Simulation result  |  MATLAB R%s  |  %s  |  %s  |  ' ...
    'trail = driven path, dashed = reference path'], ...
    cfg.env.platform.matlabRelease, cfg.env.platform.cpu, ...
    string(datetime('now'), 'yyyy-MM-dd HH:mm')), ...
    'Color', t.textDim, 'FontName', t.font, 'FontSize', 8, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left');

file = fullfile(cfg.io.figuresDir, sprintf('%s_%s_seed%d_bev.png', ...
    cfg.scenario, cfg.name, cfg.seed));
exportgraphics(fig, file, 'Resolution', 300, 'BackgroundColor', t.bg);
close(fig);
end
