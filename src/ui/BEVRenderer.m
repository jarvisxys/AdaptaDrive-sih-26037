classdef BEVRenderer < handle
    %BEVRENDERER  Bird's-eye view of a scenario, built once and updated.
    %
    %   Graphics objects are created ONCE in create() and only their data is
    %   touched in update().  Recreating patches every frame is what makes a
    %   MATLAB animation crawl, and the UI has a 15 fps target to hit.
    %
    %   The renderer draws what the simulation actually produced.  Layers that
    %   a milestone has not built yet (risk field, predictions, candidate fan)
    %   have their handles created empty and stay HIDDEN until real data
    %   arrives - they are never filled with illustrative values.
    %
    %   See also THEME, ADCLASSCOLOR, SIMLOG.

    properties (SetAccess = private)
        ax
        scenario
        cfg
        th              % theme
        h               % graphics handle struct
        maxAgents
        created = false
    end

    methods
        function obj = BEVRenderer(ax, scenario, cfg)
            obj.ax = ax;
            obj.scenario = scenario;
            obj.cfg = cfg;
            obj.th = theme();
            obj.maxAgents = max(numel(scenario.agents) * 2 + 8, 24);
        end

        % ----------------------------------------------------------------
        function create(obj)
            %CREATE  Static artwork plus empty handles for the dynamic layers.
            ax = obj.ax; %#ok<*PROPLC>
            t = obj.th;

            cla(ax);
            hold(ax, 'on');
            set(ax, 'Color', t.offroad, 'XColor', t.textDim, 'YColor', t.textDim, ...
                'GridColor', t.grid, 'GridAlpha', 0.25, 'Box', 'on', ...
                'FontName', t.font, 'FontSize', t.fsSmall);
            axis(ax, 'equal');
            grid(ax, 'on');

            % --- drivable surface ------------------------------------
            for k = 1:numel(obj.scenario.road.segments)
                P = obj.scenario.road.surfacePolygon(k);
                patch(ax, 'XData', P(:,1), 'YData', P(:,2), ...
                    'FaceColor', t.drivable, 'EdgeColor', t.roadEdge, ...
                    'LineWidth', 1.2, 'FaceAlpha', 1);
            end

            % --- risk field -------------------------------------------
            % A SURFACE, not an image: the risk grid is ego-ALIGNED, so it
            % rotates with the vehicle, and image objects cannot be rotated.
            % Created empty and hidden; it only becomes visible once a real
            % field arrives.
            obj.h.risk = surface(ax, 'XData', [], 'YData', [], 'ZData', [], ...
                'CData', [], 'AlphaData', [], ...
                'FaceColor', 'flat', 'FaceAlpha', 'flat', ...
                'AlphaDataMapping', 'none', 'EdgeColor', 'none', ...
                'Visible', 'off');
            colormap(ax, t.riskColormap);
            clim(ax, [0 1]);

            % --- hazards ----------------------------------------------
            hz = obj.scenario.hazards;
            for k = 1:hz.numPotholes()
                p = hz.potholes(k);
                [bx, by] = ellipsePts(p, 40);
                patch(ax, 'XData', bx, 'YData', by, ...
                    'FaceColor', t.pothole, 'FaceAlpha', 0.55, ...
                    'EdgeColor', t.pothole, 'LineWidth', 1.0);
            end
            for k = 1:numel(hz.edgeBreaks)
                poly = hz.edgeBreaks{k};
                patch(ax, 'XData', poly(:,1), 'YData', poly(:,2), ...
                    'FaceColor', t.offroad, 'FaceAlpha', 0.9, ...
                    'EdgeColor', t.caution, 'LineStyle', ':', 'LineWidth', 1.0);
            end
            for k = 1:numel(hz.statics)
                st = hz.statics(k);
                C = obbCorners(makeOBB(st.x, st.y, st.yaw, st.L, st.W));
                patch(ax, 'XData', C(:,1), 'YData', C(:,2), ...
                    'FaceColor', t.stall, 'EdgeColor', t.textDim, 'LineWidth', 1.0);
            end

            % --- paths -------------------------------------------------
            obj.h.refPath = plot(ax, NaN, NaN, '--', 'Color', [t.textFaint 0.7], ...
                'LineWidth', 1.0);
            obj.h.globalPath = plot(ax, NaN, NaN, '-', 'Color', t.globalPath, ...
                'LineWidth', 2.4, 'Visible', 'off');
            obj.h.candidates = plot(ax, NaN, NaN, '-', 'Color', t.candidate, ...
                'LineWidth', 0.5, 'Visible', 'off');
            obj.h.localTraj = plot(ax, NaN, NaN, '-', 'Color', t.localTraj, ...
                'LineWidth', 2.6, 'Visible', 'off');
            obj.h.trail = plot(ax, NaN, NaN, '-', 'Color', [t.accent 0.55], ...
                'LineWidth', 1.6);

            % --- goal marker -------------------------------------------
            g = obj.scenario.ego.goal;
            plot(ax, g(1), g(2), 'p', 'MarkerSize', 16, ...
                'MarkerFaceColor', t.safe, 'MarkerEdgeColor', t.text, 'LineWidth', 1.0);

            % --- agents (preallocated) ---------------------------------
            obj.h.agent = gobjects(obj.maxAgents, 1);
            obj.h.agentLabel = gobjects(obj.maxAgents, 1);
            for k = 1:obj.maxAgents
                obj.h.agent(k) = patch(ax, 'XData', NaN, 'YData', NaN, ...
                    'FaceColor', t.textDim, 'EdgeColor', t.classEdge(1,:), ...
                    'LineWidth', 1.0, 'Visible', 'off');
                obj.h.agentLabel(k) = text(ax, NaN, NaN, '', ...
                    'Color', t.text, 'FontName', t.font, 'FontSize', t.fsSmall - 1, ...
                    'HorizontalAlignment', 'center', 'Visible', 'off');
            end

            % --- ego ---------------------------------------------------
            obj.h.ego = patch(ax, 'XData', NaN, 'YData', NaN, ...
                'FaceColor', t.ego, 'EdgeColor', t.text, 'LineWidth', 1.4);
            obj.h.egoHeading = plot(ax, NaN, NaN, '-', 'Color', t.text, 'LineWidth', 1.6);

            % --- HUD ---------------------------------------------------
            obj.h.title = title(ax, '', 'Color', t.text, 'FontName', t.font, ...
                'FontSize', t.fsHeading, 'FontWeight', 'bold');
            xlabel(ax, 'x (m)', 'Color', t.textDim, 'FontName', t.font);
            ylabel(ax, 'y (m)', 'Color', t.textDim, 'FontName', t.font);

            obj.created = true;
        end

        % ----------------------------------------------------------------
        function update(obj, frame)
            %UPDATE  Push one frame of data into the existing handles.
            %
            %   frame fields: ego (egoState), agents (agentTruth array),
            %                 trail (n x 2), refPath (n x 2), t, titleText

            if ~obj.created
                obj.create();
            end
            t = obj.th;

            % --- risk field ---------------------------------------------
            if isfield(frame, 'risk') && ~isempty(frame.risk)
                obj.updateRisk(frame.risk, frame.riskMeta);
            else
                set(obj.h.risk, 'Visible', 'off');
            end

            % --- paths and trail ---------------------------------------
            if isfield(frame, 'refPath') && ~isempty(frame.refPath)
                set(obj.h.refPath, 'XData', frame.refPath(:,1), 'YData', frame.refPath(:,2));
            end
            if isfield(frame, 'trail') && ~isempty(frame.trail)
                set(obj.h.trail, 'XData', frame.trail(:,1), 'YData', frame.trail(:,2));
            end

            % --- what the planner is thinking ---------------------------
            % The candidate fan, the corridor the global planner chose, and
            % the trajectory actually selected.  Each is hidden when the
            % configuration does not produce it - BL1 has no global path and
            % no fan, and showing an empty line would imply it did.
            obj.setLine(obj.h.globalPath, getField(frame, 'globalPath'));
            obj.setLine(obj.h.localTraj,  getField(frame, 'localTraj'));

            cands = getField(frame, 'candidates');
            if isempty(cands)
                set(obj.h.candidates, 'Visible', 'off');
            else
                % One polyline with NaN separators: 77 rollouts as 77 separate
                % line objects would cost more than the rest of the frame.
                X = []; Y = [];
                for c = 1:numel(cands)
                    p = cands{c};
                    if isempty(p), continue, end
                    X = [X; p(:,1); NaN]; %#ok<AGROW>
                    Y = [Y; p(:,2); NaN]; %#ok<AGROW>
                end
                set(obj.h.candidates, 'XData', X, 'YData', Y, 'Visible', 'on');
            end

            % --- agents -------------------------------------------------
            agents = frame.agents;
            nShown = 0;
            for k = 1:numel(agents)
                a = agents(k);
                if ~a.active
                    continue
                end
                nShown = nShown + 1;
                if nShown > obj.maxAgents
                    break
                end
                C = obbCorners(makeOBB(a.x, a.y, a.yaw, a.length, a.width));
                [fill, edge] = adClassColor(a.classId);
                if a.isWrongWay
                    edge = t.danger;
                    lw = 2.4;
                else
                    lw = 1.0;
                end
                set(obj.h.agent(nShown), 'XData', C(:,1), 'YData', C(:,2), ...
                    'FaceColor', fill, 'EdgeColor', edge, 'LineWidth', lw, ...
                    'Visible', 'on');

                pr = classPriors(a.classId);
                set(obj.h.agentLabel(nShown), 'Position', [a.x, a.y + a.width/2 + 0.9, 0], ...
                    'String', sprintf('%s%d', pr.shortName, a.id), 'Visible', 'on');
            end
            for k = nShown+1:obj.maxAgents
                set(obj.h.agent(k), 'Visible', 'off');
                set(obj.h.agentLabel(k), 'Visible', 'off');
            end

            % --- ego ----------------------------------------------------
            e = frame.ego;
            off = obj.cfg.vehicle.length/2 - obj.cfg.vehicle.rearOverhang;
            box = makeOBB(e.x + off*cos(e.yaw), e.y + off*sin(e.yaw), e.yaw, ...
                obj.cfg.vehicle.length, obj.cfg.vehicle.width);
            C = obbCorners(box);
            set(obj.h.ego, 'XData', C(:,1), 'YData', C(:,2));
            hl = obj.cfg.vehicle.length * 0.75;
            set(obj.h.egoHeading, ...
                'XData', [e.x, e.x + hl*cos(e.yaw)], ...
                'YData', [e.y, e.y + hl*sin(e.yaw)]);

            % --- title --------------------------------------------------
            if isfield(frame, 'titleText')
                set(obj.h.title, 'String', frame.titleText);
            end
        end

        % ----------------------------------------------------------------
        function setLine(~, h, pts)
            %SETLINE  Draw a polyline, or hide the handle when there is none.
            if isempty(pts) || size(pts, 1) < 2
                set(h, 'Visible', 'off');
            else
                set(h, 'XData', pts(:,1), 'YData', pts(:,2), 'Visible', 'on');
            end
        end

        % ----------------------------------------------------------------
        function updateRisk(obj, R, meta)
            %UPDATERISK  Draw the ego-aligned risk field in world coordinates.
            %
            %   R may be the uint8 field stored in a replay log or a
            %   full-precision double from a live run; both are normalised
            %   back to [0,1] here.
            %
            %   Displayed at half resolution (180x60 instead of 360x120).
            %   That is a DISPLAY decision only - every metric and every
            %   planner query uses the full-resolution field.  A 43,200-vertex
            %   surface redrawn at 15 fps is the single most expensive thing
            %   in the view, and at 0.5 m the field is still smooth to the eye.

            if isa(R, 'uint8')
                R = double(R) / 255;
            end
            if isempty(R) || isempty(meta)
                set(obj.h.risk, 'Visible', 'off');
                return
            end

            step = 2;
            Rd = R(1:step:end, 1:step:end);
            [ny, nx] = size(Rd);

            % Cell centres in the ego frame, then rotated into the world.
            ex = linspace(meta.xe(1), meta.xe(2), nx);
            ey = linspace(meta.ye(1), meta.ye(2), ny);
            [EX, EY] = meshgrid(ex, ey);

            c = cos(meta.yaw);
            s = sin(meta.yaw);
            % originXY is the world position of ego-frame cell (1,1), so the
            % ego position is recovered by removing that cell's offset.
            ox = meta.originXY(1) - (c * meta.xe(1) - s * meta.ye(1));
            oy = meta.originXY(2) - (s * meta.xe(1) + c * meta.ye(1));

            WX = ox + c * EX - s * EY;
            WY = oy + s * EX + c * EY;

            t = obj.th;

            % Show the overlay only over DRIVABLE ground.  Everything off the
            % carriageway is risk 1 by definition, and at full alpha that slab
            % covers most of the frame and hides the graded on-road structure
            % the field exists to convey.  The off-road region is still
            % unmistakable - it is the dark surround outside the road patch -
            % and figure captions say so.  This is a display mask only: the
            % planner reads the unmasked field, where off-road is 1.
            alpha = min(Rd, 1) * t.riskAlphaMax;
            if ~isempty(obj.scenario) && isfield(obj.scenario, 'road') ...
                    && ~isempty(obj.scenario.road)
                onRoad = obj.scenario.road.isDrivable(WX, WY);
                alpha(~onRoad) = 0;
            end

            set(obj.h.risk, ...
                'XData', WX, 'YData', WY, 'ZData', zeros(ny, nx), ...
                'CData', Rd, 'AlphaData', alpha, ...
                'Visible', 'on');
        end

        % ----------------------------------------------------------------
        function focusOn(obj, cx, cy, spanX, spanY)
            %FOCUSON  Centre the view, used by the ego-following camera.
            if nargin < 5, spanY = spanX * 0.6; end
            xlim(obj.ax, [cx - spanX/2, cx + spanX/2]);
            ylim(obj.ax, [cy - spanY/2, cy + spanY/2]);
        end

        % ----------------------------------------------------------------
        function fitAll(obj, pad)
            %FITALL  Frame the whole road, for the static overview figure.
            if nargin < 2, pad = 6; end
            allXY = [];
            for k = 1:numel(obj.scenario.road.segments)
                allXY = [allXY; obj.scenario.road.surfacePolygon(k)]; %#ok<AGROW>
            end
            xlim(obj.ax, [min(allXY(:,1)) - pad, max(allXY(:,1)) + pad]);
            ylim(obj.ax, [min(allXY(:,2)) - pad, max(allXY(:,2)) + pad]);
        end
    end
end

% ========================================================================
function v = getField(s, f)
if isstruct(s) && isfield(s, f)
    v = s.(f);
else
    v = [];
end
end

% ========================================================================
function [bx, by] = ellipsePts(p, n)
th = linspace(0, 2*pi, n);
lx = p.a * cos(th);
ly = p.b * sin(th);
c = cos(p.yaw); s = sin(p.yaw);
bx = p.x + lx*c - ly*s;
by = p.y + lx*s + ly*c;
end
