classdef AdaptaDriveApp < handle
    %ADAPTADRIVEAPP  Presentation UI for the AdaptaDrive prototype.
    %
    %   Built programmatically with uifigure/uigridlayout rather than as a
    %   .mlapp, so it is diffable, reviewable and editable as source.
    %
    %   TABS
    %     Live / Replay   bird's-eye view with the risk field, tracks,
    %                     predictions, planned paths and the behaviour state
    %     Results         reads results/*.csv; says so plainly when there are none
    %     Architecture    the pipeline, and which backend each block is using
    %     Scenarios       the five scenarios and what each one tests
    %
    %   HONESTY
    %   Every number shown here is read from a SimLog that a real run
    %   produced, or from a CSV that run_experiments wrote.  Nothing is
    %   hardcoded, and anything not yet computed is displayed as "not yet run"
    %   rather than as a plausible-looking value.
    %
    %   Usage:  app = AdaptaDriveApp;
    %
    %   See also RUN_DEMO, RUN_EXPERIMENTS, BEVRENDERER.

    properties (SetAccess = private)
        fig
        th
        tabs
        ui = struct()

        cfg
        scenario
        log
        renderer

        cmpScenario
        cmpLog
        cmpRenderer

        frameIdx = 1
        playing = false
        speedMult = 1
        playTimer
        compareOn = false
    end

    methods
        function obj = AdaptaDriveApp()
            obj.th = theme();
            obj.cfg = defaultConfig();
            obj.buildFigure();
            obj.buildLiveTab();
            obj.buildResultsTab();
            obj.buildArchitectureTab();
            obj.buildGalleryTab();

            % Open on a replayed log if one exists.  A live demo must never
            % depend on a simulation finishing on stage.
            obj.loadInitial();
        end

        function delete(obj)
            obj.stopTimer();
            if ~isempty(obj.fig) && isvalid(obj.fig)
                delete(obj.fig);
            end
        end
    end

    % ====================================================================
    methods (Access = private)

        function buildFigure(obj)
            t = obj.th;
            obj.fig = uifigure('Name', 'AdaptaDrive - lane-free risk-aware planning (SIMULATION)', ...
                'Color', t.bg, 'Position', [60 60 1680 940], ...
                'CloseRequestFcn', @(~,~) obj.onClose());

            outer = uigridlayout(obj.fig, [2 1]);
            outer.RowHeight = {54, '1x'};
            outer.BackgroundColor = t.bg;
            outer.Padding = [10 10 10 6];
            outer.RowSpacing = 6;

            obj.buildHeader(outer);

            obj.tabs = uitabgroup(outer);
            obj.tabs.Layout.Row = 2;
        end

        % ----------------------------------------------------------------
        function buildHeader(obj, parent)
            t = obj.th;
            hdr = uigridlayout(parent, [1 3]);
            hdr.Layout.Row = 1;
            hdr.ColumnWidth = {360, '1x', 420};
            hdr.BackgroundColor = t.panel;
            hdr.Padding = [14 4 14 4];

            lbl = uilabel(hdr, 'Text', 'AdaptaDrive');
            lbl.FontName = t.font; lbl.FontSize = 22; lbl.FontWeight = 'bold';
            lbl.FontColor = t.accent;

            sub = uilabel(hdr, 'Text', ...
                'Risk-aware lane-free planning for unstructured Indian roads');
            sub.FontName = t.font; sub.FontSize = 12; sub.FontColor = t.textDim;

            tag = uilabel(hdr, 'Text', ...
                sprintf('SIMULATION  |  team SECOND INNINGS  |  SIH 2026  PS 26037'));
            tag.FontName = t.font; tag.FontSize = 11; tag.FontColor = t.textDim;
            tag.HorizontalAlignment = 'right';
        end

        % ================================================================
        function buildLiveTab(obj)
            t = obj.th;
            tab = uitab(obj.tabs, 'Title', '  Live / Replay  ', 'BackgroundColor', t.bg);

            g = uigridlayout(tab, [3 2]);
            g.RowHeight = {46, '1x', 150};
            g.ColumnWidth = {'1x', 330};
            g.BackgroundColor = t.bg;
            g.Padding = [8 8 8 8];
            g.RowSpacing = 8; g.ColumnSpacing = 8;

            obj.buildControls(g);

            % --- bird's-eye view(s) ------------------------------------
            viewPanel = uipanel(g, 'BackgroundColor', t.panel, 'BorderType', 'none');
            viewPanel.Layout.Row = 2; viewPanel.Layout.Column = 1;
            vg = uigridlayout(viewPanel, [1 2]);
            vg.ColumnWidth = {'1x', '0x'};      % second view hidden until compare
            vg.BackgroundColor = t.panel;
            vg.Padding = [4 4 4 4]; vg.ColumnSpacing = 6;
            obj.ui.viewGrid = vg;

            obj.ui.ax = uiaxes(vg);
            obj.ui.axCmp = uiaxes(vg);
            obj.ui.axCmp.Visible = 'off';

            obj.buildSidePanel(g);
            obj.buildTimeline(g);
        end

        % ----------------------------------------------------------------
        function buildControls(obj, parent)
            t = obj.th;
            p = uipanel(parent, 'BackgroundColor', t.panel, 'BorderType', 'none');
            p.Layout.Row = 1; p.Layout.Column = [1 2];

            g = uigridlayout(p, [1 14]);
            g.ColumnWidth = {70, 130, 60, 120, 48, 70, 62, 90, 90, 80, 78, 64, 150, '1x'};
            g.BackgroundColor = t.panel;
            g.Padding = [8 5 8 5]; g.ColumnSpacing = 6;

            obj.mkLabel(g, 'Scenario');
            % 'unseen' is the B5 held-out variant. It is offered here so it can
            % be demonstrated live, but it is NOT in run_experiments' default
            % scenario list and never enters the headline completion rate.
            obj.ui.scenarioDD = uidropdown(g, ...
                'Items', {'village', 'intersection', 'highway', 'market', ...
                          'cattle', 'unseen'}, ...
                'Value', 'village', 'FontName', t.font, ...
                'BackgroundColor', t.panelAlt, 'FontColor', t.text);

            obj.mkLabel(g, 'Config');
            obj.ui.configDD = uidropdown(g, ...
                'Items', {'PROPOSED', 'BL1', 'BL2', 'BL3', 'STRESS-sensor', ...
                          '-riskmap', '-classpriors', '-uncertainty', ...
                          '-context', '-wrongside'}, ...
                'Value', 'PROPOSED', 'FontName', t.font, ...
                'BackgroundColor', t.panelAlt, 'FontColor', t.text);

            obj.mkLabel(g, 'Seed');
            obj.ui.seedSpin = uispinner(g, 'Limits', [1 999], 'Value', 1, ...
                'FontName', t.font, 'BackgroundColor', t.panelAlt, 'FontColor', t.text);

            obj.ui.runBtn = obj.mkButton(g, 'Run live', t.accent, ...
                @(~,~) obj.onRunLive());
            obj.ui.playBtn = obj.mkButton(g, 'Play', t.safe, ...
                @(~,~) obj.onPlayPause());
            obj.ui.restartBtn = obj.mkButton(g, 'Restart', t.panelAlt, ...
                @(~,~) obj.onRestart());

            obj.mkLabel(g, 'Speed');
            obj.ui.speedDD = uidropdown(g, 'Items', {'0.5x','1x','2x','4x'}, ...
                'Value', '1x', 'FontName', t.font, ...
                'BackgroundColor', t.panelAlt, 'FontColor', t.text, ...
                'ValueChangedFcn', @(src,~) obj.onSpeed(src.Value));

            obj.ui.cmpChk = uicheckbox(g, 'Text', 'Compare vs BL1', ...
                'FontName', t.font, 'FontColor', t.text, ...
                'ValueChangedFcn', @(src,~) obj.onCompare(src.Value));

            obj.ui.status = uilabel(g, 'Text', 'starting...', ...
                'FontName', t.font, 'FontColor', t.textDim, 'FontSize', 11);
        end

        % ----------------------------------------------------------------
        function buildSidePanel(obj, parent)
            t = obj.th;
            p = uipanel(parent, 'BackgroundColor', t.panel, 'BorderType', 'none');
            p.Layout.Row = 2; p.Layout.Column = 2;

            g = uigridlayout(p, [4 1]);
            g.RowHeight = {28, 210, 26, '1x'};
            g.BackgroundColor = t.panel;
            g.Padding = [10 10 10 10]; g.RowSpacing = 6;

            h = uilabel(g, 'Text', 'BEHAVIOUR STATE');
            h.FontName = t.font; h.FontSize = 11; h.FontWeight = 'bold';
            h.FontColor = t.textDim;

            % --- FSM state pills, in the deck's order -------------------
            pills = uigridlayout(g, [numel(t.stateOrder) 1]);
            pills.RowHeight = repmat({26}, 1, numel(t.stateOrder));
            pills.BackgroundColor = t.panel;
            pills.Padding = [0 0 0 0]; pills.RowSpacing = 2;
            obj.ui.pills = gobjects(numel(t.stateOrder), 1);
            for k = 1:numel(t.stateOrder)
                obj.ui.pills(k) = uilabel(pills, 'Text', ['  ' t.stateOrder{k}], ...
                    'FontName', t.font, 'FontSize', 11, ...
                    'FontColor', t.textFaint, 'BackgroundColor', t.panelAlt);
            end

            obj.ui.reason = uilabel(g, 'Text', '', 'FontName', t.font, ...
                'FontSize', 10, 'FontColor', t.textDim, 'WordWrap', 'on');

            % --- live metric cards ---------------------------------------
            cards = uigridlayout(g, [7 2]);
            cards.RowHeight = repmat({30}, 1, 7);
            cards.ColumnWidth = {'1.1x', '1x'};
            cards.BackgroundColor = t.panel;
            cards.Padding = [0 0 0 0]; cards.RowSpacing = 3;

            names = {'time', 'speed', 'min TTC', 'clearance', ...
                     'cycle latency', 'jerk', 'e-brakes'};
            obj.ui.metric = struct();
            keys = {'time','speed','ttc','clear','lat','jerk','eb'};
            for k = 1:numel(names)
                l = uilabel(cards, 'Text', names{k}, 'FontName', t.font, ...
                    'FontSize', 11, 'FontColor', t.textDim);
                l.Layout.Row = k; l.Layout.Column = 1;
                v = uilabel(cards, 'Text', '-', 'FontName', t.font, ...
                    'FontSize', 13, 'FontWeight', 'bold', 'FontColor', t.text);
                v.Layout.Row = k; v.Layout.Column = 2;
                obj.ui.metric.(keys{k}) = v;
            end
        end

        % ----------------------------------------------------------------
        function buildTimeline(obj, parent)
            t = obj.th;
            p = uipanel(parent, 'BackgroundColor', t.panel, 'BorderType', 'none');
            p.Layout.Row = 3; p.Layout.Column = [1 2];

            g = uigridlayout(p, [2 2]);
            g.RowHeight = {34, '1x'};
            g.ColumnWidth = {'1.6x', '1x'};
            g.BackgroundColor = t.panel;
            g.Padding = [10 6 10 6]; g.RowSpacing = 4; g.ColumnSpacing = 10;

            obj.ui.scrub = uislider(g, 'Limits', [1 2], 'Value', 1, ...
                'MajorTicks', [], 'MinorTicks', [], ...
                'FontColor', t.textDim, ...
                'ValueChangingFcn', @(~,e) obj.onScrub(e.Value));
            obj.ui.scrub.Layout.Row = 1; obj.ui.scrub.Layout.Column = 1;

            obj.ui.ctxTable = uilabel(g, 'Text', '', 'FontName', t.fontMono, ...
                'FontSize', 10, 'FontColor', t.textDim, 'WordWrap', 'on');
            obj.ui.ctxTable.Layout.Row = 1; obj.ui.ctxTable.Layout.Column = 2;

            obj.ui.eventList = uitextarea(g, 'Editable', 'off', ...
                'FontName', t.fontMono, 'FontSize', 10, ...
                'BackgroundColor', t.panelAlt, 'FontColor', t.text);
            obj.ui.eventList.Layout.Row = 2; obj.ui.eventList.Layout.Column = [1 2];
        end

        % ================================================================
        function buildResultsTab(obj)
            t = obj.th;
            tab = uitab(obj.tabs, 'Title', '  Results  ', 'BackgroundColor', t.bg);
            g = uigridlayout(tab, [2 1]);
            g.RowHeight = {'1x', 34};
            g.BackgroundColor = t.bg; g.Padding = [10 10 10 10];

            obj.ui.resultsPanel = uipanel(g, 'BackgroundColor', t.panel, ...
                'BorderType', 'none');
            obj.ui.resultsFooter = uilabel(g, 'Text', '', 'FontName', t.font, ...
                'FontSize', 10, 'FontColor', t.textDim);

            obj.refreshResults();
        end

        % ----------------------------------------------------------------
        function refreshResults(obj)
            t = obj.th;
            delete(obj.ui.resultsPanel.Children);

            runsFile = adRoot('results', 'runs.csv');
            if ~isfile(runsFile)
                g = uigridlayout(obj.ui.resultsPanel, [1 1]);
                g.BackgroundColor = t.panel;
                l = uilabel(g, 'Text', ...
                    ['No experiment results yet.' newline newline ...
                     'Run   run_experiments   to generate results/runs.csv and summary.csv.' ...
                     newline 'This panel shows only measured results; it never displays placeholder numbers.'], ...
                    'FontName', obj.th.font, 'FontSize', 14, ...
                    'FontColor', t.textDim, 'HorizontalAlignment', 'center');
                obj.ui.resultsFooter.Text = 'No results file present.';
                return
            end

            T = readtable(runsFile);
            g = uigridlayout(obj.ui.resultsPanel, [2 2]);
            g.BackgroundColor = t.panel; g.Padding = [8 8 8 8];
            g.RowSpacing = 8; g.ColumnSpacing = 8;

            obj.plotCompletion(g, T);
            obj.plotLatency(g, T);
            obj.plotMetricBox(g, T, 'minClearance', 'minimum clearance (m)', 1);
            obj.plotMetricBox(g, T, 'minTTC', 'minimum TTC (s)', 2);

            nSeeds = numel(unique(T.seed));
            obj.ui.resultsFooter.Text = sprintf( ...
                ['Simulation results  |  %d runs, N = %d seeds  |  generated %s  |  %s, MATLAB R%s'], ...
                height(T), nSeeds, string(datetime('now'), 'yyyy-MM-dd HH:mm'), ...
                obj.cfg.env.platform.cpu, obj.cfg.env.platform.matlabRelease);
        end

        function plotCompletion(obj, g, T)
            ax = uiaxes(g); ax.Layout.Row = 1; ax.Layout.Column = 1;
            obj.styleAxes(ax, 'Collision-free completion by scenario');
            [gr, sc, cf] = findgroups(T.scenario, T.config);
            rate = splitapply(@(x) 100*mean(x), T.completed & ~T.collision, gr);
            scU = unique(sc, 'stable'); cfU = unique(cf, 'stable');
            M = nan(numel(scU), numel(cfU));
            for k = 1:numel(rate)
                M(strcmp(scU, sc(k)), strcmp(cfU, cf(k))) = rate(k);
            end
            b = bar(ax, M, 'grouped');
            for k = 1:numel(b), b(k).FaceColor = obj.configColour(cfU{k}); end
            ax.XTick = 1:numel(scU); ax.XTickLabel = scU;
            ylabel(ax, '%'); ylim(ax, [0 105]);
            legend(ax, cfU, 'TextColor', obj.th.textDim, 'Color', obj.th.panelAlt, ...
                'EdgeColor', 'none', 'Location', 'southoutside', 'Orientation', 'horizontal');
        end

        function plotLatency(obj, g, T)
            ax = uiaxes(g); ax.Layout.Row = 1; ax.Layout.Column = 2;
            obj.styleAxes(ax, 'Cycle latency (p95) vs 200 ms target');
            if ~ismember('latencyP95', T.Properties.VariableNames)
                return
            end
            sc = unique(T.scenario, 'stable');
            v = nan(1, numel(sc));
            for k = 1:numel(sc)
                sel = strcmp(T.scenario, sc{k}) & strcmp(T.config, 'PROPOSED');
                v(k) = mean(T.latencyP95(sel), 'omitnan');
            end
            b = bar(ax, v); b.FaceColor = obj.th.accent;
            yline(ax, 200, '--', '200 ms', 'Color', obj.th.danger, ...
                'LabelHorizontalAlignment', 'left');
            ax.XTick = 1:numel(sc); ax.XTickLabel = sc;
            ylabel(ax, 'ms');
        end

        function plotMetricBox(obj, g, T, col, label, slot)
            % Column passed in explicitly.  This used to keep its own
            % persistent counter, which survives between app instances and
            % would silently put both panels in the same cell the second time
            % the app was opened.
            ax = uiaxes(g); ax.Layout.Row = 2; ax.Layout.Column = slot;
            obj.styleAxes(ax, label);
            if ~ismember(col, T.Properties.VariableNames)
                return
            end
            sc = unique(T.scenario, 'stable');
            data = []; grp = [];
            for k = 1:numel(sc)
                sel = strcmp(T.scenario, sc{k}) & strcmp(T.config, 'PROPOSED');
                d = T.(col)(sel);
                data = [data; d]; grp = [grp; repmat(k, numel(d), 1)]; %#ok<AGROW>
            end
            if isempty(data), return, end
            scatter(ax, grp + 0.08*randn(size(grp)), data, 18, ...
                'MarkerFaceColor', obj.th.safe, 'MarkerEdgeColor', 'none', ...
                'MarkerFaceAlpha', 0.7);
            ax.XTick = 1:numel(sc); ax.XTickLabel = sc;
        end

        % ================================================================
        function buildArchitectureTab(obj)
            t = obj.th;
            tab = uitab(obj.tabs, 'Title', '  Architecture  ', 'BackgroundColor', t.bg);
            g = uigridlayout(tab, [1 1]);
            g.BackgroundColor = t.bg; g.Padding = [10 10 10 10];
            ax = uiaxes(g);
            obj.styleAxes(ax, '');
            axis(ax, 'off');
            hold(ax, 'on');

            e = obj.cfg.env;
            blocks = {
                'Sensors',      sprintf('camera + 2 radar\nbackend: %s', e.backends.sensors)
                'Tracking',     sprintf('track-level fusion\nbackend: %s', e.backends.tracker)
                'Prediction',   sprintf('multi-modal, class-conditioned\nB2 / B3')
                'Risk map',     sprintf('static + edge + dynamic + wrong-side\nB1 (A1, A2 fold in here)')
                'Detectors',    sprintf('wrong-way (A2)\ninformal merge (A3)')
                'Behaviour',    sprintf('7-state FSM + context params\nruntime: %s   chart: %s', ...
                                        e.backends.behaviorRuntime, e.backends.behaviorChart)
                'Global plan',  sprintf('%s\nfallback: %s', e.backends.globalPlanner, ...
                                        e.backends.globalPlannerFallback)
                'Local plan',   sprintf('dynamic window, 5 bounded costs\ncontinuous risk enters HERE')
                'Control',      sprintf('pure pursuit (%s) + jerk-limited PI', e.backends.purePursuit)
                'Vehicle',      sprintf('kinematic bicycle, RK4\nL=%.1f m, %.0f deg steer', ...
                                        obj.cfg.vehicle.wheelbase, rad2deg(obj.cfg.vehicle.maxSteer))
                };

            n = size(blocks, 1);
            w = 0.88 / n;
            for k = 1:n
                x = 0.06 + (k-1)*w;
                rectangle(ax, 'Position', [x 0.52 w*0.86 0.30], ...
                    'FaceColor', t.panelAlt, 'EdgeColor', t.accent, ...
                    'LineWidth', 1.2, 'Curvature', 0.12);
                text(ax, x + w*0.43, 0.75, blocks{k,1}, 'Color', t.text, ...
                    'FontName', t.font, 'FontSize', 11, 'FontWeight', 'bold', ...
                    'HorizontalAlignment', 'center');
                text(ax, x + w*0.43, 0.63, blocks{k,2}, 'Color', t.textDim, ...
                    'FontName', t.font, 'FontSize', 8, ...
                    'HorizontalAlignment', 'center');
                if k < n
                    annotationArrow(ax, x + w*0.86, 0.67, x + w, 0.67, t.accent);
                end
            end

            % Closing the loop, drawn as a loop.
            plot(ax, [0.94 0.94 0.06 0.06], [0.52 0.34 0.34 0.52], '-', ...
                'Color', t.textFaint, 'LineWidth', 1.2);
            text(ax, 0.5, 0.31, 'closed loop: vehicle state feeds back into sensing', ...
                'Color', t.textFaint, 'FontName', t.font, 'FontSize', 9, ...
                'HorizontalAlignment', 'center');

            text(ax, 0.06, 0.16, sprintf(['MATLAB R%s on %s  |  every block above is running the ' ...
                'backend named in it.'], e.platform.matlabRelease, e.platform.cpu), ...
                'Color', t.textDim, 'FontName', t.font, 'FontSize', 9);
            text(ax, 0.06, 0.09, ['Hybrid A* uses a BINARY validity check, so the continuous ' ...
                'risk field shapes the trajectory in the local planner, not the global search.'], ...
                'Color', t.caution, 'FontName', t.font, 'FontSize', 9);

            xlim(ax, [0 1]); ylim(ax, [0 1]);
        end

        % ================================================================
        function buildGalleryTab(obj)
            t = obj.th;
            tab = uitab(obj.tabs, 'Title', '  Scenarios  ', 'BackgroundColor', t.bg);
            g = uigridlayout(tab, [1 6]);
            g.BackgroundColor = t.bg; g.Padding = [10 10 10 10];
            g.ColumnSpacing = 8;

            % Six panels: the five tuned scenarios and the held-out variant.
            names = {'village', 'intersection', 'highway', 'market', ...
                     'cattle', 'unseen'};
            for k = 1:numel(names)
                p = uipanel(g, 'BackgroundColor', obj.th.panel, 'BorderType', 'none');
                pg = uigridlayout(p, [2 1]);
                pg.RowHeight = {'1x', 150};
                pg.BackgroundColor = obj.th.panel;
                pg.Padding = [6 6 6 6];

                ax = uiaxes(pg);
                obj.styleAxes(ax, '');
                try
                    sc = buildScenario(names{k}, 1, 1.0, obj.cfg);
                    r = BEVRenderer(ax, sc, obj.cfg);
                    r.create();
                    r.update(struct('ego', sc.ego, ...
                        'agents', arrayfun(@(a) a.truth(), sc.agents), ...
                        'refPath', sc.refPath, 'trail', [], ...
                        'titleText', sc.title));
                    % Zoom to the ego rather than fitting the whole route:
                    % 'axis equal' on a 250 m x 40 m road inside a narrow card
                    % squashes the scene into an unreadable strip.
                    r.focusOn(sc.ego.x + 30, sc.ego.y, 110, 150);
                    info = sprintf('%s\n\nCHALLENGE\n%s\n\nSUCCESS\n%s', ...
                        sc.description, sc.keyEvent, sc.successCriterion);
                catch ME
                    info = sprintf('%s\n\nnot available: %s', names{k}, ME.message);
                end

                uilabel(pg, 'Text', info, 'FontName', t.font, 'FontSize', 9, ...
                    'FontColor', t.textDim, 'WordWrap', 'on', ...
                    'VerticalAlignment', 'top');
            end
        end

        % ================================================================
        %  Playback
        % ================================================================

        function loadInitial(obj)
            f = obj.logPath('village', 'PROPOSED', 1);
            if isfile(f)
                obj.loadLog(f);
                obj.setStatus('replaying saved log - press Play');
            else
                obj.setStatus(['no saved log yet - press "Run live", ' ...
                    'or run  run_experiments  to generate replay logs']);
                obj.ui.scrub.Enable = 'off';
            end
        end

        function f = logPath(~, scenario, config, seed)
            f = adRoot('logs', sprintf('%s_%s_seed%d.mat', scenario, config, seed));
        end

        function loadLog(obj, file)
            S = load(file, 'log');
            obj.log = S.log;
            m = obj.log.meta;
            obj.cfg = m.cfg;
            obj.scenario = buildScenario(m.scenario, m.seed, m.density, obj.cfg);
            obj.renderer = BEVRenderer(obj.ui.ax, obj.scenario, obj.cfg);
            obj.styleAxes(obj.ui.ax, '');
            obj.renderer.create();

            obj.frameIdx = 1;
            obj.ui.scrub.Limits = [1 max(obj.log.n, 2)];
            obj.ui.scrub.Value = 1;
            obj.ui.scrub.Enable = 'on';
            obj.showEvents();
            obj.showContext();
            obj.renderFrame(1);
        end

        % ----------------------------------------------------------------
        function onRunLive(obj)
            obj.stopTimer();
            sc = obj.ui.scenarioDD.Value;
            cf = obj.ui.configDD.Value;
            sd = obj.ui.seedSpin.Value;
            obj.setStatus(sprintf('running %s / %s / seed %d ...', sc, cf, sd));
            drawnow;

            try
                cfgL = configPreset(cf, 'scenario', sc, 'seed', sd);
                scen = buildScenario(sc, sd, 1.0, cfgL);
                res = SimEngine(scen, cfgL).run();

                obj.cfg = cfgL;
                obj.scenario = scen;
                obj.log = res.log;
                obj.renderer = BEVRenderer(obj.ui.ax, scen, cfgL);
                obj.styleAxes(obj.ui.ax, '');
                obj.renderer.create();

                obj.frameIdx = 1;
                obj.ui.scrub.Limits = [1 max(obj.log.n, 2)];
                obj.ui.scrub.Value = 1;
                obj.ui.scrub.Enable = 'on';
                obj.showEvents();
                obj.showContext();
                obj.renderFrame(1);

                if obj.compareOn
                    obj.loadCompare();
                end
                obj.setStatus(sprintf('%s - %s in %.1f s', upper(res.outcome.reason), ...
                    sc, res.metrics.completionTime));
            catch ME
                obj.setStatus(sprintf('run failed: %s', ME.message));
            end
        end

        function onPlayPause(obj)
            if isempty(obj.log)
                obj.setStatus('nothing loaded - press "Run live"');
                return
            end
            obj.playing = ~obj.playing;
            if obj.playing
                obj.ui.playBtn.Text = 'Pause';
                obj.startTimer();
            else
                obj.ui.playBtn.Text = 'Play';
                obj.stopTimer();
            end
        end

        function onRestart(obj)
            obj.frameIdx = 1;
            if ~isempty(obj.log)
                obj.renderFrame(1);
            end
        end

        function onSpeed(obj, val)
            obj.speedMult = str2double(strrep(val, 'x', ''));
            if obj.playing
                obj.stopTimer(); obj.startTimer();
            end
        end

        function onScrub(obj, v)
            if isempty(obj.log), return, end
            obj.frameIdx = max(1, min(round(v), obj.log.n));
            obj.renderFrame(obj.frameIdx);
        end

        function onCompare(obj, val)
            obj.compareOn = logical(val);
            if obj.compareOn
                obj.ui.viewGrid.ColumnWidth = {'1x', '1x'};
                obj.ui.axCmp.Visible = 'on';
                obj.loadCompare();
            else
                obj.ui.viewGrid.ColumnWidth = {'1x', '0x'};
                obj.ui.axCmp.Visible = 'off';
                obj.cmpLog = [];
            end
        end

        function loadCompare(obj)
            if isempty(obj.log), return, end
            m = obj.log.meta;
            obj.setStatus('running baseline for comparison ...'); drawnow;
            try
                cfgB = configPreset('BL1', 'scenario', m.scenario, 'seed', m.seed);
                scB = buildScenario(m.scenario, m.seed, m.density, cfgB);
                resB = SimEngine(scB, cfgB).run();
                obj.cmpScenario = scB;
                obj.cmpLog = resB.log;
                obj.cmpRenderer = BEVRenderer(obj.ui.axCmp, scB, cfgB);
                obj.styleAxes(obj.ui.axCmp, '');
                obj.cmpRenderer.create();
                obj.renderFrame(obj.frameIdx);
                obj.setStatus(sprintf('compare: PROPOSED vs BL1 (BL1 outcome: %s)', ...
                    upper(resB.outcome.reason)));
            catch ME
                obj.setStatus(sprintf('compare failed: %s', ME.message));
                obj.cmpLog = [];
            end
        end

        function startTimer(obj)
            period = max(0.04, 0.05 / max(obj.speedMult, 0.1));
            obj.playTimer = timer('ExecutionMode', 'fixedSpacing', ...
                'Period', round(period, 3), 'TimerFcn', @(~,~) obj.tick());
            start(obj.playTimer);
        end

        function stopTimer(obj)
            if ~isempty(obj.playTimer) && isvalid(obj.playTimer)
                stop(obj.playTimer); delete(obj.playTimer);
            end
            obj.playTimer = [];
        end

        function tick(obj)
            if isempty(obj.log) || ~isvalid(obj.fig)
                obj.stopTimer(); return
            end
            step = max(1, round(obj.speedMult));
            obj.frameIdx = obj.frameIdx + step;
            if obj.frameIdx >= obj.log.n
                obj.frameIdx = obj.log.n;
                obj.playing = false;
                obj.ui.playBtn.Text = 'Play';
                obj.stopTimer();
            end
            obj.renderFrame(obj.frameIdx);
        end

        % ----------------------------------------------------------------
        function renderFrame(obj, k)
            if isempty(obj.log) || k < 1 || k > obj.log.n, return, end
            k = min(k, obj.log.n);
            st = obj.log.egoStateAt(k);

            cyc = obj.cycleAt(obj.log, k);
            frame = struct('ego', st, 'agents', obj.log.agents{k}, ...
                'trail', obj.log.ego(max(1,k-260):k, 1:2), ...
                'refPath', obj.scenario.refPath, 't', obj.log.t(k));

            % The risk field is stored on a stride in 'light' logs, so take the
            % most recent cycle that actually carries one.  State and metrics
            % still come from the exact cycle - only the picture lags, and
            % never by more than the stride.
            rc = obj.riskCycleAt(obj.log, k);
            if ~isempty(rc)
                frame.risk = rc.risk;
                frame.riskMeta = rc.riskMeta;
                frame.candidates = rc.candidates;   % stored on the same stride
            end
            if ~isempty(cyc)
                frame.globalPath = cyc.globalPath;
                frame.localTraj  = cyc.localTraj;
            end
            frame.titleText = sprintf('%s  -  %s, seed %d  |  t = %.2f s  |  SIMULATION', ...
                obj.scenario.title, obj.log.meta.config, obj.log.meta.seed, obj.log.t(k));

            obj.renderer.update(frame);
            obj.renderer.focusOn(st.x, st.y, 90, 46);

            if obj.compareOn && ~isempty(obj.cmpLog)
                kb = min(k, obj.cmpLog.n);
                stb = obj.cmpLog.egoStateAt(kb);
                fb = struct('ego', stb, 'agents', obj.cmpLog.agents{kb}, ...
                    'trail', obj.cmpLog.ego(max(1,kb-260):kb, 1:2), ...
                    'refPath', obj.cmpScenario.refPath, 't', obj.cmpLog.t(kb));
                fb.titleText = sprintf('BL1 lane-follow  |  t = %.2f s', obj.cmpLog.t(kb));
                obj.cmpRenderer.update(fb);
                obj.cmpRenderer.focusOn(stb.x, stb.y, 90, 46);
            end

            obj.updateSidePanel(k, cyc, st);
            obj.ui.scrub.Value = k;
            drawnow limitrate;
        end

        % ----------------------------------------------------------------
        function updateSidePanel(obj, k, cyc, st)
            t = obj.th;
            stateName = '';
            if ~isempty(cyc) && ~isempty(cyc.state)
                stateName = cyc.state;
            end
            for i = 1:numel(t.stateOrder)
                if strcmp(t.stateOrder{i}, stateName)
                    obj.ui.pills(i).BackgroundColor = t.stateColor(i, :);
                    obj.ui.pills(i).FontColor = [1 1 1];
                    obj.ui.pills(i).FontWeight = 'bold';
                else
                    obj.ui.pills(i).BackgroundColor = t.panelAlt;
                    obj.ui.pills(i).FontColor = t.textFaint;
                    obj.ui.pills(i).FontWeight = 'normal';
                end
            end

            if ~isempty(cyc) && ~isempty(cyc.reason)
                obj.ui.reason.Text = cyc.reason;
            end

            obj.ui.metric.time.Text  = sprintf('%.2f s', obj.log.t(k));
            obj.ui.metric.speed.Text = sprintf('%.2f m/s', st.v);

            if ~isempty(cyc)
                obj.setMetric('ttc', cyc.minTTC, 's', 2, [1.5 4.0]);
                obj.setMetric('clear', cyc.minClear, 'm', 2, [0.5 1.5]);
                obj.setMetric('lat', cyc.latencyMs, 'ms', 0, [-inf 200], true);
            end

            a = obj.log.ego(1:k, 5);
            if numel(a) > 1
                obj.ui.metric.jerk.Text = sprintf('%.1f m/s3', ...
                    max(abs(diff(a)))/obj.cfg.sim.dt);
            end
            obj.ui.metric.eb.Text = sprintf('%d', obj.countEvents(k, 'EMERGENCY_BRAKE'));
        end

        function setMetric(obj, key, val, unit, dec, good, invert)
            if nargin < 7, invert = false; end
            t = obj.th;
            if isempty(val) || isnan(val)
                obj.ui.metric.(key).Text = '-';
                obj.ui.metric.(key).FontColor = t.textDim;
                return
            end
            obj.ui.metric.(key).Text = sprintf(['%.' num2str(dec) 'f %s'], val, unit);
            if invert
                c = t.safe; if val > good(2), c = t.danger; end
            else
                if val < good(1), c = t.danger;
                elseif val < good(2), c = t.caution;
                else, c = t.safe;
                end
            end
            obj.ui.metric.(key).FontColor = c;
        end

        function n = countEvents(obj, k, type)
            n = 0;
            if isempty(obj.log.eventLog), return, end
            tk = obj.log.t(k);
            n = sum(strcmp({obj.log.eventLog.type}, type) & ...
                    [obj.log.eventLog.t] <= tk);
        end

        function c = cycleAt(~, log, k)
            c = [];
            if log.nCycles == 0, return, end
            steps = [log.cycles.step];
            idx = find(steps <= k, 1, 'last');
            if ~isempty(idx), c = log.cycles(idx); end
        end

        function c = riskCycleAt(~, log, k)
            %RISKCYCLEAT  Most recent cycle that carries a stored risk field.
            c = [];
            if log.nCycles == 0, return, end
            steps = [log.cycles.step];
            cand = find(steps <= k);
            for i = numel(cand):-1:1
                if ~isempty(log.cycles(cand(i)).risk)
                    c = log.cycles(cand(i));
                    return
                end
            end
        end

        function showEvents(obj)
            if isempty(obj.log) || isempty(obj.log.eventLog)
                obj.ui.eventList.Value = {'(no events)'};
                return
            end
            E = obj.log.eventLog;
            lines = cell(numel(E), 1);
            for k = 1:numel(E)
                lines{k} = sprintf('%7.2f s  %-16s %s', E(k).t, E(k).type, E(k).text);
            end
            obj.ui.eventList.Value = lines;
        end

        function showContext(obj)
            c = obj.log.meta.contextParams;
            obj.ui.ctxTable.Text = sprintf( ...
                ['ACTIVE CONTEXT (B4): %s\n' ...
                 'speedCap %.1f m/s   gap %.1f m   lateral %.2f m\n' ...
                 'ttcSlow %.1f s   ttcEmerg %.1f s   replan %.2f s'], ...
                c.name, c.speedCap, c.followingGap, c.lateralClearance, ...
                c.ttcSlow, c.ttcEmergency, c.replanPeriod);
        end

        % ================================================================
        function styleAxes(obj, ax, ttl)
            t = obj.th;
            ax.Color = t.offroad;
            ax.XColor = t.textDim; ax.YColor = t.textDim;
            ax.GridColor = t.grid;
            ax.FontName = t.font; ax.FontSize = 9;
            if ~isempty(ttl)
                title(ax, ttl, 'Color', t.text, 'FontName', t.font, 'FontSize', 12);
            end
        end

        function l = mkLabel(obj, parent, txt)
            l = uilabel(parent, 'Text', txt, 'FontName', obj.th.font, ...
                'FontSize', 11, 'FontColor', obj.th.textDim, ...
                'HorizontalAlignment', 'right');
        end

        function b = mkButton(obj, parent, txt, colour, cb)
            b = uibutton(parent, 'Text', txt, 'FontName', obj.th.font, ...
                'FontSize', 11, 'FontWeight', 'bold', ...
                'BackgroundColor', colour, 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', cb);
        end

        function c = configColour(obj, name)
            switch name
                case 'PROPOSED', c = obj.th.safe;
                case 'BL1',      c = obj.th.classFill(1,:);
                case 'BL2',      c = obj.th.classFill(3,:);
                case 'BL3',      c = obj.th.classFill(6,:);
                otherwise,       c = obj.th.textDim;
            end
        end

        function setStatus(obj, msg)
            obj.ui.status.Text = msg;
        end

        function onClose(obj)
            obj.stopTimer();
            delete(obj.fig);
        end
    end
end

% ========================================================================
function annotationArrow(ax, x1, y1, x2, y2, colour)
plot(ax, [x1 x2], [y1 y2], '-', 'Color', colour, 'LineWidth', 1.1);
plot(ax, x2, y2, '>', 'Color', colour, 'MarkerFaceColor', colour, 'MarkerSize', 4);
end
