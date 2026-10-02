function files = make_figures(varargin)
%MAKE_FIGURES  Turn results/*.csv into presentation figures.
%
%   MAKE_FIGURES()                 reads results/runs.csv and summary.csv
%   MAKE_FIGURES('outDir', path)   writes the PNGs somewhere else
%
%   Every figure is drawn from the CSV that run_experiments wrote.  If there
%   is no CSV, this stops and says so rather than drawing anything: a figure
%   with no measurements behind it is worse than no figure.
%
%   Figures produced
%     completion.png   collision-free completion by scenario and config,
%                      with Wilson 95% intervals (N is small; the interval is
%                      the honest way to show that)
%     latency.png      cycle latency per scenario against the 200 ms target
%     safety.png       minimum clearance and minimum TTC distributions
%     smoothness.png   jerk and lateral acceleration by config
%     stages.png       cycle time split across the eight pipeline stages,
%                      which is the figure that showed tracking, not planning,
%                      was the latency bottleneck
%     ablations.png    PROPOSED minus each ablation, where ablations were run
%
%   See also RUN_EXPERIMENTS, MAKE_VIDEO.

p = inputParser;
p.addParameter('resultsDir', adRoot('results'));
p.addParameter('outDir', adRoot('results', 'figures'));
p.parse(varargin{:});
opt = p.Results;

runsFile = fullfile(opt.resultsDir, 'runs.csv');
if ~isfile(runsFile)
    error('make_figures:noResults', ...
        ['No results at %s.\n' ...
         'Run  run_experiments  first. Figures are only ever drawn from ' ...
         'measured runs.'], runsFile);
end

T = readtable(runsFile);
% readtable returns text columns as cellstr by default while run_experiments
% writes them from string arrays.  Normalising here means the indexing below
% is the same regardless of which way round it came back.
T = convertvars(T, vartype('cellstr'), 'string');
if ~isfolder(opt.outDir), mkdir(opt.outDir); end

th = theme();
files = {};

files{end+1} = figCompletion(T, th, opt.outDir);
files{end+1} = figLatency(T, th, opt.outDir);
files{end+1} = figSafety(T, th, opt.outDir);
files{end+1} = figSmoothness(T, th, opt.outDir);

s = figStages(th, opt.outDir, opt.resultsDir, T);
if ~isempty(s), files{end+1} = s; end

a = figAblations(T, th, opt.outDir);
if ~isempty(a), files{end+1} = a; end

fprintf('\n  %d figure(s) written to %s\n', numel(files), opt.outDir);
for k = 1:numel(files)
    fprintf('    %s\n', files{k});
end
fprintf('\n');
end

% ========================================================================
function f = figCompletion(T, th, outDir)
[fig, ax] = newFig(th, 1100, 620);
success = T.completed & ~T.collision;

sc = unique(T.scenario, 'stable');
cf = unique(T.config, 'stable');
M = nan(numel(sc), numel(cf));
LO = M; HI = M;
for i = 1:numel(sc)
    for j = 1:numel(cf)
        sel = T.scenario == sc(i) & T.config == cf(j);
        n = sum(sel);
        if n == 0, continue, end
        k = sum(success(sel));
        M(i,j) = 100 * k / n;
        [lo, hi] = wilson(k, n);
        LO(i,j) = 100*lo; HI(i,j) = 100*hi;
    end
end

b = bar(ax, M, 'grouped');
for j = 1:numel(b)
    b(j).FaceColor = configColour(cf(j), th);
    b(j).EdgeColor = 'none';
end

% Wilson intervals on each bar: N is small and a bare percentage would imply
% precision the sample cannot support.
hold(ax, 'on');
for j = 1:numel(b)
    x = b(j).XEndPoints;
    errorbar(ax, x, M(:,j), M(:,j)-LO(:,j), HI(:,j)-M(:,j), ...
        'LineStyle', 'none', 'Color', th.text, 'LineWidth', 1.0, 'CapSize', 5);
end

ax.XTick = 1:numel(sc); ax.XTickLabel = cellstr(sc);
ylabel(ax, 'collision-free completion (%)');
ylim(ax, [0 112]);
title(ax, 'Collision-free completion, with Wilson 95% intervals  (SIMULATION)', ...
    'Color', th.text);
legend(ax, cellstr(cf), 'TextColor', th.text, 'Color', th.panelAlt, 'EdgeColor', 'none', ...
    'Location', 'northoutside', 'Orientation', 'horizontal');
f = save(fig, outDir, 'completion.png', T, th);
end

% ========================================================================
function f = figLatency(T, th, outDir)
[fig, ax] = newFig(th, 1100, 560);
sc = unique(T.scenario, 'stable');
data = []; grp = [];
for i = 1:numel(sc)
    sel = T.scenario == sc(i) & T.config == "PROPOSED" & ~T.stallFlag;
    d = T.latencyP95(sel);
    d = d(~isnan(d));
    data = [data; d]; grp = [grp; repmat(i, numel(d), 1)]; %#ok<AGROW>
end
if isempty(data)
    text(ax, 0.5, 0.5, 'no latency data', 'Color', th.textDim, ...
        'HorizontalAlignment', 'center');
else
    boxchart(ax, grp, data, 'BoxFaceColor', th.accent, ...
        'MarkerColor', th.caution, 'BoxEdgeColor', th.text);
    hold(ax, 'on');
    yline(ax, 200, '--', '200 ms target', 'Color', th.danger, ...
        'LineWidth', 1.4, 'LabelHorizontalAlignment', 'left', ...
        'FontName', th.font);
end
ax.XTick = 1:numel(sc); ax.XTickLabel = cellstr(sc);
ylabel(ax, 'p95 cycle latency (ms)');
title(ax, sprintf('Planning-cycle latency, PROPOSED  |  %s, MATLAB interpreted', ...
    shortCpu(T)), 'Color', th.text);
f = save(fig, outDir, 'latency.png', T, th);
end

% ========================================================================
function f = figSafety(T, th, outDir)
fig = figure('Visible', 'off', 'Color', th.bg, 'Position', [80 80 1200 520]);
sc = unique(T.scenario, 'stable');

for panel = 1:2
    ax = subplot(1, 2, panel, 'Parent', fig);
    styleAxes(ax, th);
    if panel == 1
        col = 'minClearance'; lab = 'minimum clearance (m)'; ref = 0.5;
    else
        col = 'minTTC'; lab = 'minimum TTC (s)'; ref = 1.5;
    end
    data = []; grp = [];
    for i = 1:numel(sc)
        sel = T.scenario == sc(i) & T.config == "PROPOSED";
        d = T.(col)(sel); d = d(~isnan(d));
        data = [data; d]; grp = [grp; repmat(i, numel(d), 1)]; %#ok<AGROW>
    end
    if ~isempty(data)
        swarm(ax, grp, data, th);
        yline(ax, ref, '--', sprintf('%.1f', ref), 'Color', th.danger, ...
            'FontName', th.font);
    end
    ax.XTick = 1:numel(sc); ax.XTickLabel = cellstr(sc);
    ylabel(ax, lab);
    title(ax, lab, 'Color', th.text);
end
sgtitle(fig, 'Safety margins, PROPOSED, one point per seed  (SIMULATION)', ...
    'Color', th.text, 'FontName', th.font);
f = save(fig, outDir, 'safety.png', T, th);
end

% ========================================================================
function f = figSmoothness(T, th, outDir)
[fig, ax] = newFig(th, 1100, 560);
cf = unique(T.config, 'stable');
data = []; grp = [];
for j = 1:numel(cf)
    sel = T.config == cf(j);
    d = T.jerkRMS(sel); d = d(~isnan(d));
    data = [data; d]; grp = [grp; repmat(j, numel(d), 1)]; %#ok<AGROW>
end
if ~isempty(data)
    swarm(ax, grp, data, th);
end
ax.XTick = 1:numel(cf); ax.XTickLabel = cellstr(cf);
ylabel(ax, 'RMS longitudinal jerk (m/s^3)');
title(ax, 'Ride smoothness by configuration  (SIMULATION)', 'Color', th.text);
f = save(fig, outDir, 'smoothness.png', T, th);
end

% ========================================================================
function f = figStages(th, outDir, resultsDir, T)
%FIGSTAGES  Where each cycle's time actually goes, per stage.
%
%   This figure exists because the end-to-end latency figure was misleading in
%   a specific way: it showed which scenarios missed the 200 ms budget, and
%   every reader (including the author) assumed the planner was responsible.
%   The breakdown showed tracking at 49-65% of every cycle that missed, and the
%   local planner at 11-34 ms throughout.
f = '';
stgFile = fullfile(resultsDir, 'latency_stages.csv');
if ~isfile(stgFile)
    return      % not measured; draw nothing rather than invent it
end
St = readtable(stgFile);
St = convertvars(St, vartype('cellstr'), 'string');
names = SimEngine.stageNames();

M = zeros(height(St), numel(names));
for i = 1:numel(names)
    M(:, i) = St.([names{i} '_p50']);
end

[fig, ax] = newFig(th, 1150, 620);
b = bar(ax, M, 'stacked');

% Eight visually distinct colours, picked rather than taken from lines(), whose
% eighth entry repeats the first - that put 'sense' and 'dwa' in the same blue,
% at opposite ends of every bar.
cmap = [ ...
    0.20 0.55 0.90;    % sense    blue
    0.95 0.55 0.15;    % track    orange
    0.95 0.85 0.25;    % predict  yellow
    0.70 0.40 0.85;    % detect   purple
    0.25 0.75 0.35;    % risk     green
    0.35 0.85 0.85;    % fsm      cyan
    0.95 0.40 0.70;    % global   pink
    0.60 0.60 0.68];   % dwa      grey
for i = 1:numel(b)
    b(i).FaceColor = cmap(min(i, size(cmap,1)), :);
    b(i).EdgeColor = th.bg;
end
ax.XTick = 1:height(St);
ax.XTickLabel = cellstr(St.scenario);
ylabel(ax, 'median cycle time (ms)');

% Headroom above the budget line, or its label sits off the top of the axes and
% the reader cannot see which bars clear it.
ax.YLim = [0, max(220, 1.15 * max(sum(M, 2)))];

% The bar handles are passed explicitly AND AutoUpdate is off. Passing the
% handles alone is not enough: legend defaults to AutoUpdate 'on', so the yline
% drawn below is adopted afterwards and appears as a spurious "data1" entry.
legend(ax, b, names, 'TextColor', th.text, 'Color', th.bg, ...
    'EdgeColor', th.textDim, 'Location', 'northwest', 'AutoUpdate', 'off');
yline(ax, 200, '--', '200 ms budget', 'Color', th.danger, ...
    'LabelHorizontalAlignment', 'right', 'FontWeight', 'bold');
title(ax, ['Where the cycle time goes, by stage (PROPOSED, isolated)' ...
    '  (SIMULATION)'], 'Color', th.text);
f = save(fig, outDir, 'stages.png', T, th, sprintf( ...
    '%d isolated runs, PROPOSED, 1 seed', height(St)));
end

% ========================================================================
function f = figAblations(T, th, outDir)
f = '';
abl = {'-riskmap', '-classpriors', '-uncertainty', '-context', '-wrongside'};
present = string(abl(ismember(abl, cellstr(unique(T.config)))));
if isempty(present)
    return      % ablations were not run; draw nothing rather than invent it
end

[fig, ax] = newFig(th, 1100, 560);
success = T.completed & ~T.collision;
base = 100 * mean(success(T.config == "PROPOSED"));

delta = zeros(1, numel(present));
for k = 1:numel(present)
    delta(k) = 100 * mean(success(T.config == present(k))) - base;
end

b = bar(ax, delta);
b.FaceColor = 'flat';
for k = 1:numel(delta)
    if delta(k) < 0, b.CData(k,:) = th.danger; else, b.CData(k,:) = th.safe; end
end
ax.XTick = 1:numel(present); ax.XTickLabel = cellstr(present);
% Headroom, or a bar that happens to equal the data range is drawn flush with
% the axis edge and reads as clipped rather than as its actual value.
pad = max(5, 0.18 * max(abs(delta)));
ax.YLim = [min(min(delta) - pad, -pad), max(max(delta) + pad, pad)];
ylabel(ax, 'completion vs PROPOSED (percentage points)');
title(ax, sprintf('Ablation deltas (PROPOSED baseline = %.0f%%)  (SIMULATION)', base), ...
    'Color', th.text);
yline(ax, 0, '-', 'Color', th.textDim);
f = save(fig, outDir, 'ablations.png', T, th);
end

% ========================================================================
function [fig, ax] = newFig(th, w, h)
fig = figure('Visible', 'off', 'Color', th.bg, 'Position', [80 80 w h]);
ax = axes(fig); %#ok<LAXES>
styleAxes(ax, th);
end

function styleAxes(ax, th)
ax.Color = th.panel;
ax.XColor = th.textDim; ax.YColor = th.textDim;
ax.GridColor = th.grid; ax.GridAlpha = 0.25;
ax.FontName = th.font; ax.FontSize = 10;
grid(ax, 'on');
hold(ax, 'on');
end

function swarm(ax, grp, data, th)
jitter = 0.09 * randn(size(grp));
scatter(ax, grp + jitter, data, 26, 'filled', ...
    'MarkerFaceColor', th.safe, 'MarkerFaceAlpha', 0.75, ...
    'MarkerEdgeColor', 'none');
for i = unique(grp)'
    m = median(data(grp == i));
    plot(ax, i + [-0.25 0.25], [m m], '-', 'Color', th.accent, 'LineWidth', 2);
end
end

function f = save(fig, outDir, name, T, th, sampleNote)
% Provenance on every figure: the machine and the release it was measured on.
%
% SAMPLENOTE overrides the run count for a figure whose data did NOT come from
% the matrix. Stamping "90 runs, 2 seeds" on a figure measured from five
% isolated runs would be a false provenance line, which is worse than none.
if nargin < 6 || isempty(sampleNote)
    sampleNote = sprintf('%d runs, %d seeds', height(T), numel(unique(T.seed)));
end
annotation(fig, 'textbox', [0.005 0.002 0.99 0.035], ...
    'String', sprintf(['SIMULATION  |  %s  |  %s  |  MATLAB R%s  |  %s'], ...
    sampleNote, shortCpu(T), ...
    char(string(T.matlabRelease(1))), string(datetime('now'), 'yyyy-MM-dd HH:mm')), ...
    'Color', th.textFaint, 'FontName', th.font, 'FontSize', 7.5, ...
    'EdgeColor', 'none');
f = fullfile(outDir, name);
exportgraphics(fig, f, 'Resolution', 300, 'BackgroundColor', th.bg);
close(fig);
end

function s = shortCpu(T)
s = char(string(T.cpu(1)));
s = regexprep(s, '\s+', ' ');
end

function c = configColour(name, th)
switch name
    case 'PROPOSED', c = th.safe;
    case 'BL1',      c = th.classFill(1,:);
    case 'BL2',      c = th.classFill(3,:);
    case 'BL3',      c = th.classFill(6,:);
    otherwise,       c = th.textDim;
end
end

function [lo, hi] = wilson(k, n)
if n == 0, lo = NaN; hi = NaN; return, end
z = 1.959963984540054;
phat = k/n; den = 1 + z^2/n;
centre = (phat + z^2/(2*n))/den;
half = z*sqrt(phat*(1-phat)/n + z^2/(4*n^2))/den;
lo = max(centre-half, 0); hi = min(centre+half, 1);
end
