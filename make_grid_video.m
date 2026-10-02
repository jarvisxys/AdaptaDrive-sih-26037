function file = make_grid_video(varargin)
%MAKE_GRID_VIDEO  Four scenarios in one frame, on one clock, under one HUD.
%
%   MAKE_GRID_VIDEO()
%   MAKE_GRID_VIDEO('speed', 4, 'maxSeconds', 90)
%   MAKE_GRID_VIDEO('scenarios', {'village','market','highway','cattle'})
%
%   Renders a 2x2 grid of saved runs at 1920x1080. Every panel is driven from
%   the SAME simulation clock, so what the viewer sees in the four panels
%   happened at the same moment of simulated time in each run.
%
%   That is the whole point of this script, and it is why laying four separate
%   renders side by side in an editor is not equivalent. The four runs are
%   different lengths - on seed 1 the intersection finishes in 31.6 s while
%   the village takes 99.3 s - so four clips started together drift apart
%   immediately, and a panel that has quietly looped or ended reads as though
%   it were still driving. Here a run that ends holds its final frame and is
%   stamped with the outcome it actually reached.
%
%   Parameters
%     'scenarios'   4 scenario names   (default village, intersection,
%                                       highway, cattle - the four PROPOSED
%                                       completes on seed 1)
%     'configs'     one config name, or 4   (default PROPOSED)
%     'seed'        default 1
%     'speed'       simulated seconds per real second (default 4). The grid
%                   runs as long as its longest panel, so at speed 1 the
%                   default set is a 99 s clip; at 4 it is 25 s.
%     'maxSeconds'  stop after this much SIMULATED time (default inf)
%     'fps'         default 30
%     'follow'      true  = each panel's camera tracks its own ego (default)
%                   false = each panel shows its whole route, fixed
%
%   See also MAKE_VIDEO, ADLOADRUN, FINALIZE.

p = inputParser;
p.addParameter('scenarios', {'village', 'intersection', 'highway', 'cattle'});
p.addParameter('configs', 'PROPOSED');
p.addParameter('seed', 1, @isscalar);
p.addParameter('speed', 4, @(x) isscalar(x) && x > 0);
p.addParameter('maxSeconds', inf, @isscalar);
p.addParameter('fps', 30, @isscalar);
p.addParameter('follow', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('outDir', adRoot('results', 'video'));
p.parse(varargin{:});
opt = p.Results;

sc = cellstr(opt.scenarios);
if numel(sc) ~= 4
    error('make_grid_video:needFour', ...
        'This renders a 2x2 grid, so it needs exactly 4 scenarios (got %d).', ...
        numel(sc));
end

cfgNames = cellstr(opt.configs);
if isscalar(cfgNames)
    cfgNames = repmat(cfgNames, 1, 4);
elseif numel(cfgNames) ~= 4
    error('make_grid_video:configCount', ...
        'Give one config for all four panels, or exactly four.');
end

if ~isfolder(opt.outDir), mkdir(opt.outDir); end
th = theme();

% --- load the four runs -------------------------------------------------
fprintf('\n  Grid: %s\n', strjoin(sc, ', '));
P = struct('log', {}, 'scenario', {}, 'cfg', {}, 'dist', {}, ...
    'outcome', {}, 'tEnd', {});
for i = 1:4
    fprintf('  loading %-13s %-10s ... ', sc{i}, cfgNames{i});
    [lg, scn, cf] = adLoadRun(sc{i}, cfgNames{i}, opt.seed);

    % Distance travelled, precomputed once. Doing this per frame would make
    % the renderer quadratic in run length for a number that never changes.
    xy = lg.ego(1:lg.n, 1:2);
    step = [0; cumsum(hypot(diff(xy(:,1)), diff(xy(:,2))))];

    P(i).log = lg; P(i).scenario = scn; P(i).cfg = cf; %#ok<AGROW>
    P(i).dist = step;                                   %#ok<AGROW>
    P(i).outcome = outcomeOf(lg);                       %#ok<AGROW>
    P(i).tEnd = lg.t(lg.n);                             %#ok<AGROW>
    fprintf('%s, %.1f m in %.1f s\n', P(i).outcome, step(end), P(i).tEnd);
end

% --- master clock -------------------------------------------------------
% The grid runs until its longest panel ends. Panels that finish earlier are
% held on their last frame rather than dropped, so the viewer can see that
% they finished rather than wonder where they went.
tStop = min(max([P.tEnd]), opt.maxSeconds);
nFrames = max(1, floor(tStop * opt.fps / opt.speed) + 1);

% --- writer -------------------------------------------------------------
profile = P(1).cfg.env.backends.videoProfile;
switch profile
    case 'MPEG-4', ext = '.mp4';
    otherwise,     ext = '.avi';
end
file = fullfile(opt.outDir, sprintf('grid4_%s_seed%d%s', ...
    cfgNames{1}, opt.seed, ext));

vw = VideoWriter(file, profile);
vw.FrameRate = opt.fps;
open(vw);
closer = onCleanup(@() close(vw)); %#ok<NASGU>

% --- figure -------------------------------------------------------------
fig = figure('Visible', 'off', 'Color', th.bg, ...
    'Position', [0 0 1920 1080], 'Renderer', 'opengl');

% 2x2 panels. Normalised [left bottom width height], top row first so the
% panel order matches the order the caller listed the scenarios in.
panelPos = [0.022 0.520 0.470 0.408
            0.508 0.520 0.470 0.408
            0.022 0.100 0.470 0.408
            0.508 0.100 0.470 0.408];

R = cell(1, 4);
lblTitle = gobjects(1, 4);
lblStat  = gobjects(1, 4);
lblStamp = gobjects(1, 4);

for i = 1:4
    ax = axes(fig, 'Position', panelPos(i, :)); %#ok<LAXES>
    ax.Color = th.offroad;
    ax.XColor = th.textDim; ax.YColor = th.textDim;
    ax.GridColor = th.grid; ax.FontName = th.font; ax.FontSize = 8;
    R{i} = BEVRenderer(ax, P(i).scenario, P(i).cfg);
    R{i}.create();
    if ~logical(opt.follow)
        R{i}.fitAll(8);
    end

    % No axis furniture. In the app the coordinates matter; in a four-panel
    % video they are unreadable at this size, they cost roughly a fifth of the
    % usable height, and the bottom row's tick labels land on top of the clock.
    % The distance readout in the corner is the number a viewer can actually
    % use, so the ticks go and the panels grow into the space.
    ax.XTick = []; ax.YTick = [];
    xlabel(ax, ''); ylabel(ax, '');
    ax.XColor = th.border; ax.YColor = th.border;

    % Scenario name, top-left inside the panel.
    lblTitle(i) = annotation(fig, 'textbox', ...
        [panelPos(i,1)+0.006, panelPos(i,2)+panelPos(i,4)-0.040, 0.30, 0.034], ...
        'String', upper(P(i).scenario.title), ...
        'Color', th.text, 'FontName', th.font, 'FontSize', 15, ...
        'FontWeight', 'bold', 'EdgeColor', 'none', ...
        'BackgroundColor', th.panel, 'FaceAlpha', 0.72, ...
        'VerticalAlignment', 'middle', 'Margin', 4);

    % Live distance and speed, bottom-left inside the panel.
    lblStat(i) = annotation(fig, 'textbox', ...
        [panelPos(i,1)+0.006, panelPos(i,2)+0.006, 0.30, 0.030], ...
        'String', '', 'Color', th.textDim, 'FontName', th.fontMono, ...
        'FontSize', 12, 'EdgeColor', 'none', ...
        'BackgroundColor', th.panel, 'FaceAlpha', 0.72, ...
        'VerticalAlignment', 'middle', 'Margin', 4);

    % Outcome stamp, centred, hidden until that panel's run ends.
    lblStamp(i) = annotation(fig, 'textbox', ...
        [panelPos(i,1)+panelPos(i,3)/2-0.085, panelPos(i,2)+panelPos(i,4)/2-0.030, ...
         0.17, 0.060], ...
        'String', '', 'Color', th.text, 'FontName', th.font, ...
        'FontSize', 26, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'EdgeColor', 'none', 'Visible', 'off');
end

annotation(fig, 'textbox', [0.025 0.945 0.95 0.045], ...
    'String', sprintf(['AdaptaDrive  -  four scenarios, one clock  |  %s, ' ...
    'seed %d  |  SIMULATION'], cfgNames{1}, opt.seed), ...
    'Color', th.text, 'FontName', th.font, 'FontSize', 21, ...
    'FontWeight', 'bold', 'EdgeColor', 'none');

clockTxt = annotation(fig, 'textbox', [0.022 0.054 0.95 0.044], ...
    'String', '', 'Color', th.accent, 'FontName', th.fontMono, ...
    'FontSize', 17, 'FontWeight', 'bold', 'EdgeColor', 'none');

hud = annotation(fig, 'textbox', [0.022 0.026 0.95 0.032], ...
    'String', '', 'Color', th.textDim, 'FontName', th.fontMono, ...
    'FontSize', 12, 'EdgeColor', 'none');

annotation(fig, 'textbox', [0.022 0.002 0.95 0.024], ...
    'String', sprintf(['MATLAB R%s  |  %s  |  every panel advances on the ' ...
    'same simulated clock  |  simulation, not a road test'], ...
    P(1).cfg.env.platform.matlabRelease, P(1).cfg.env.platform.cpu), ...
    'Color', th.textFaint, 'FontName', th.font, 'FontSize', 9, ...
    'EdgeColor', 'none');

% --- frames -------------------------------------------------------------
fprintf('\n  rendering %d frames (%.1f s of video, %.0fx speed) to %s\n', ...
    nFrames, nFrames / opt.fps, opt.speed, file);

tick = max(1, round(nFrames / 20));

for f = 1:nFrames
    tNow = (f - 1) * opt.speed / opt.fps;      % the one shared clock
    hudParts = cell(1, 4);

    for i = 1:4
        lg = P(i).log;

        % Index by TIME, not by step count. The panels are only guaranteed to
        % share a clock if each one is asked where it was at tNow; indexing by
        % step assumes every run used the same timestep, which is true today
        % and is exactly the kind of assumption that breaks silently later.
        k = find(lg.t(1:lg.n) <= tNow, 1, 'last');
        if isempty(k), k = 1; end
        done = tNow >= P(i).tEnd;

        st = lg.egoStateAt(k);
        fr = struct('ego', st, 'agents', lg.agents{k}, ...
            'trail', lg.ego(max(1, k-300):k, 1:2), ...
            'refPath', P(i).scenario.refPath, 't', lg.t(k), 'titleText', '');
        rc = riskCycleAt(lg, k);
        if ~isempty(rc)
            fr.risk = rc.risk; fr.riskMeta = rc.riskMeta;
        end
        R{i}.update(fr);
        if logical(opt.follow)
            R{i}.focusOn(st.x, st.y, 110, 58);
        end

        lblStat(i).String = sprintf(' %6.1f m   %4.1f m/s ', ...
            P(i).dist(k), st.v);

        % The stamp is drawn once, on the frame the run ends, and then left
        % alone. Re-setting a visible annotation every frame makes MATLAB
        % re-lay-out the text box 400 times for a string that never changes.
        if done && strcmp(lblStamp(i).Visible, 'off')
            [stampTxt, stampCol] = stampFor(P(i).outcome, th);
            lblStamp(i).String = stampTxt;
            lblStamp(i).Color = stampCol;
            lblStamp(i).Visible = 'on';
        end

        hudParts{i} = sprintf('%-12s %7.1f m %s', ...
            lower(sc{i}), P(i).dist(k), ternary(done, upper(P(i).outcome), '...'));
    end

    clockTxt.String = sprintf('t = %6.2f s        %.0fx speed', tNow, opt.speed);
    hud.String = strjoin(hudParts, '    |    ');

    drawnow;
    writeVideo(vw, getframe(fig));

    if mod(f, tick) == 0
        fprintf('    %3.0f%%  t = %.1f s\n', 100 * f / nFrames, tNow);
    end
end

close(fig);
fprintf('\n  wrote %s\n', file);
fprintf('  %.1f s of video covering %.1f s of simulation\n\n', ...
    nFrames / opt.fps, tStop);
end

% ========================================================================
function s = outcomeOf(log)
%OUTCOMEOF  The reason the run ended, as a lower-case word.
%   Reported from the log's own metadata rather than re-derived, so the stamp
%   on the video can never disagree with the row in runs.csv.
s = 'unknown';
if ~isfield(log.meta, 'outcome') || isempty(log.meta.outcome)
    return
end
o = log.meta.outcome;
if isstruct(o) && isfield(o, 'reason')
    s = lower(char(string(o.reason)));
elseif ischar(o) || isstring(o)
    s = lower(char(string(o)));
end
end

% ========================================================================
function [txt, col] = stampFor(outcome, th)
switch outcome
    case 'goal'
        txt = 'GOAL';      col = th.safe;
    case 'collision'
        txt = 'COLLISION'; col = th.danger;
    case 'timeout'
        txt = 'TIMEOUT';   col = th.caution;
    case 'offroad'
        txt = 'OFF ROAD';  col = th.danger;
    otherwise
        txt = upper(outcome); col = th.textDim;
end
end

% ========================================================================
function c = riskCycleAt(log, k)
%RISKCYCLEAT  The most recent cycle at or before step k that stored a field.
%   In 'light' logging the risk field is kept every fifth cycle, so the frames
%   between reuse the last one rather than flickering the overlay off.
c = [];
if log.nCycles == 0, return, end
cand = find([log.cycles.step] <= k);
for i = numel(cand):-1:1
    if ~isempty(log.cycles(cand(i)).risk)
        c = log.cycles(cand(i));
        return
    end
end
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
