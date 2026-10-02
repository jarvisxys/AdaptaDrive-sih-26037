function file = make_video(scenarioName, configName, seed, varargin)
%MAKE_VIDEO  Render a saved run to MP4 for the demo.
%
%   MAKE_VIDEO('village', 'PROPOSED', 1)
%   MAKE_VIDEO('village', 'PROPOSED', 1, 'compare', true)
%
%   Renders from a SAVED LOG, offscreen, at 1920x1080 and 30 fps, with a
%   title strip and a live overlay of the behaviour state and metrics.  If no
%   log exists for the selection it runs the simulation once and uses that.
%
%   'compare' puts the same scenario and seed side by side with the BL1
%   lane-follow baseline - the moment the demo turns on.
%
%   The profile actually used is whatever check_env found (MPEG-4 where
%   available, Motion JPEG AVI otherwise), and the real extension is
%   reported, never assumed.
%
%   See also RUN_DEMO, MAKE_FIGURES, ADAPTADRIVEAPP.

p = inputParser;
p.addParameter('compare', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('fps', 30, @isscalar);
p.addParameter('outDir', adRoot('results', 'video'));
p.addParameter('maxSeconds', inf, @isscalar);
p.parse(varargin{:});
opt = p.Results;

if nargin < 3 || isempty(seed), seed = 1; end
if ~isfolder(opt.outDir), mkdir(opt.outDir); end

% --- get the run -------------------------------------------------------
[log, scenario, cfg] = loadOrRun(scenarioName, configName, seed);
cmp = [];
if logical(opt.compare)
    [logB, scenB, cfgB] = loadOrRun(scenarioName, 'BL1', seed);
    cmp = struct('log', logB, 'scenario', scenB, 'cfg', cfgB);
end

th = theme();

% --- writer ------------------------------------------------------------
profile = cfg.env.backends.videoProfile;
switch profile
    case 'MPEG-4', ext = '.mp4';
    otherwise,     ext = '.avi';
end
name = sprintf('%s_%s_seed%d%s', scenarioName, configName, seed, ...
    ternary(logical(opt.compare), '_compare', ''));
file = fullfile(opt.outDir, [name ext]);

vw = VideoWriter(file, profile);
vw.FrameRate = opt.fps;
open(vw);
closer = onCleanup(@() close(vw));

% --- figure ------------------------------------------------------------
fig = figure('Visible', 'off', 'Color', th.bg, ...
    'Position', [0 0 1920 1080], 'Renderer', 'opengl');

if isempty(cmp)
    axMain = axes(fig, 'Position', [0.03 0.10 0.72 0.80]);
    axCmp = [];
else
    axMain = axes(fig, 'Position', [0.03 0.10 0.45 0.80]);
    axCmp  = axes(fig, 'Position', [0.50 0.10 0.45 0.80]);
end

rMain = BEVRenderer(axMain, scenario, cfg);
styleAxes(axMain, th); rMain.create();
rCmp = [];
if ~isempty(cmp)
    rCmp = BEVRenderer(axCmp, cmp.scenario, cmp.cfg);
    styleAxes(axCmp, th); rCmp.create();
end

% --- title strip and overlay -------------------------------------------
titleTxt = annotation(fig, 'textbox', [0.02 0.945 0.96 0.045], ...
    'String', sprintf('AdaptaDrive  -  %s  |  %s, seed %d  |  SIMULATION', ...
    scenario.title, configName, seed), ...
    'Color', th.text, 'FontName', th.font, 'FontSize', 20, ...
    'FontWeight', 'bold', 'EdgeColor', 'none'); %#ok<NASGU>

if isempty(cmp)
    hud = annotation(fig, 'textbox', [0.77 0.12 0.21 0.76], ...
        'String', '', 'Color', th.text, 'FontName', th.fontMono, ...
        'FontSize', 13, 'EdgeColor', th.border, 'BackgroundColor', th.panel, ...
        'VerticalAlignment', 'top');
else
    hud = annotation(fig, 'textbox', [0.02 0.01 0.96 0.075], ...
        'String', '', 'Color', th.text, 'FontName', th.fontMono, ...
        'FontSize', 13, 'EdgeColor', 'none');
end

foot = annotation(fig, 'textbox', [0.02 0.005 0.96 0.03], ...
    'String', sprintf('MATLAB R%s  |  %s  |  simulation, not a road test', ...
    cfg.env.platform.matlabRelease, cfg.env.platform.cpu), ...
    'Color', th.textFaint, 'FontName', th.font, 'FontSize', 9, ...
    'EdgeColor', 'none'); %#ok<NASGU>

% --- frames ------------------------------------------------------------
stride = max(1, round(1 / (cfg.sim.dt * opt.fps)));
lastStep = log.n;
if isfinite(opt.maxSeconds)
    lastStep = min(lastStep, round(opt.maxSeconds / cfg.sim.dt));
end

fprintf('  rendering %d frames to %s ...\n', numel(1:stride:lastStep), file);

for k = 1:stride:lastStep
    st = log.egoStateAt(k);
    cyc = cycleAt(log, k);
    rc = riskCycleAt(log, k);

    f = struct('ego', st, 'agents', log.agents{k}, ...
        'trail', log.ego(max(1,k-300):k, 1:2), ...
        'refPath', scenario.refPath, 't', log.t(k));
    if ~isempty(rc)
        f.risk = rc.risk; f.riskMeta = rc.riskMeta;
    end
    f.titleText = '';
    rMain.update(f);
    rMain.focusOn(st.x, st.y, 95, 50);

    if ~isempty(rCmp)
        kb = min(k, cmp.log.n);
        stb = cmp.log.egoStateAt(kb);
        fb = struct('ego', stb, 'agents', cmp.log.agents{kb}, ...
            'trail', cmp.log.ego(max(1,kb-300):kb, 1:2), ...
            'refPath', cmp.scenario.refPath, 't', cmp.log.t(kb), ...
            'titleText', '');
        rCmp.update(fb);
        rCmp.focusOn(stb.x, stb.y, 95, 50);
    end

    hud.String = hudText(log, cyc, st, k, cmp, isempty(cmp));
    drawnow;
    writeVideo(vw, getframe(fig));
end

close(fig);
fprintf('  wrote %s\n', file);
end

% ========================================================================
function [log, scenario, cfg] = loadOrRun(scenarioName, configName, seed)
f = adRoot('logs', sprintf('%s_%s_seed%d.mat', scenarioName, configName, seed));
if isfile(f)
    S = load(f, 'log');
    log = S.log;
    cfg = log.meta.cfg;
    scenario = buildScenario(log.meta.scenario, log.meta.seed, log.meta.density, cfg);
else
    fprintf('  no saved log for %s/%s/seed %d - running it now\n', ...
        scenarioName, configName, seed);
    cfg = configPreset(configName, 'scenario', scenarioName, 'seed', seed);
    cfg.io.logDetail = 'full';
    scenario = buildScenario(scenarioName, seed, 1.0, cfg);
    res = SimEngine(scenario, cfg).run();
    log = res.log;
end
end

% ========================================================================
function s = hudText(log, cyc, st, k, cmp, vertical)
lines = {};
lines{end+1} = sprintf('t        %6.2f s', log.t(k));
lines{end+1} = sprintf('speed    %6.2f m/s', st.v);
if ~isempty(cyc)
    lines{end+1} = sprintf('state    %s', cyc.state);
    lines{end+1} = sprintf('min TTC  %6.2f s', cyc.minTTC);
    lines{end+1} = sprintf('clear    %6.2f m', cyc.minClear);
    lines{end+1} = sprintf('latency  %6.0f ms', cyc.latencyMs);
    if ~isempty(cyc.reason)
        lines{end+1} = '';
        lines{end+1} = cyc.reason;
    end
end
if ~isempty(cmp)
    kb = min(k, cmp.log.n);
    lines{end+1} = sprintf('   |   BL1 speed %.2f m/s', cmp.log.ego(kb, 4));
end
if vertical
    s = strjoin(lines, newline);
else
    s = strjoin(lines, '    ');
end
end

function c = cycleAt(log, k)
c = [];
if log.nCycles == 0, return, end
idx = find([log.cycles.step] <= k, 1, 'last');
if ~isempty(idx), c = log.cycles(idx); end
end

function c = riskCycleAt(log, k)
c = [];
if log.nCycles == 0, return, end
cand = find([log.cycles.step] <= k);
for i = numel(cand):-1:1
    if ~isempty(log.cycles(cand(i)).risk)
        c = log.cycles(cand(i)); return
    end
end
end

function styleAxes(ax, th)
ax.Color = th.offroad;
ax.XColor = th.textDim; ax.YColor = th.textDim;
ax.GridColor = th.grid; ax.FontName = th.font; ax.FontSize = 10;
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
