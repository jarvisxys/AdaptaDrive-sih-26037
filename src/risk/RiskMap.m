classdef RiskMap < handle
    %RISKMAP  Unified dynamic risk field (B1), integrating A1, A2, B2 and B3.
    %
    %       R = clip( w_s*S + w_e*E
    %                 + sum_i w_c(class_i) * m_ww(i)
    %                   * sum_k sum_t gamma^t * p_k * N(x; mu_ik(t), Sigma_ik(t) + F_i)
    %                 , 0, 1 )
    %
    %     S     static hazards and solid obstacles (A1): potholes, stalls,
    %           parked vehicles.  1 inside, decaying outside so the planner
    %           leaves room instead of grazing every edge.
    %     E     edge risk, rising within 1 m of the drivable boundary and 1
    %           outside it.
    %     w_c   class weight from CLASSPRIORS (B3) - vulnerability, not mass.
    %     m_ww  2.0 for an agent flagged wrong-way (A2), else 1.0.
    %     p_k   mode probability from the Predictor (B2).
    %     F_i   the agent's own footprint, added to the covariance so a bus
    %           occupies more of the map than a pedestrian at equal certainty.
    %     gamma 0.9 per 0.1 s, so the near future dominates the far future.
    %
    %   THE POINT OF B1: one field, many causes.  A pothole, a wrong-way auto
    %   and an uncertain pedestrian all end up as the same quantity, so the
    %   planner does not need a separate rule per hazard type - it needs one
    %   cost.  The layers stay separately addressable for the UI toggles and
    %   the ablations, but the planner sees only R.
    %
    %   GAUSSIAN NORMALISATION - a deliberate departure from the literal
    %   formula.  N(x; mu, Sigma) as written is a probability DENSITY, with
    %   units of 1/m^2; a sharply known pedestrian would then peak in the
    %   hundreds while a diffuse one peaks near zero, and clipping to [0,1]
    %   would erase exactly the uncertainty B2 exists to represent.  We use the
    %   PEAK-NORMALISED kernel exp(-0.5 * d_M^2), which reads as "how much of
    %   this cell could this agent occupy at this time" and is naturally in
    %   [0,1].  Recorded in DEVIATIONS.md D22.
    %
    %   FRAME: the grid is ego-centred AND ego-aligned - 70 m ahead, 20 m
    %   behind, 15 m either side, at 0.25 m.  A world-axis-aligned window
    %   covering the same reach at any heading would need 720x720 cells
    %   instead of 360x120, twelve times the work at 10 Hz.  The planner runs
    %   in this frame and its path is transformed back to world.
    %
    %   See also PREDICTOR, CLASSPRIORS, HAZARDSET, ROADMODEL.

    properties (SetAccess = private)
        cfg
        road
        hazards
        res
        nx, ny              % columns (longitudinal), rows (lateral)
        xe, ye              % ego-frame cell-centre coordinate vectors
        XE, YE              % ego-frame meshgrid, precomputed once
        tSub                % subsampled prediction step indices
        wSub                % discount weight per subsampled step

        R                   % ny x nx risk field in [0,1]
        Rnoside             % same field WITHOUT the wrong-side layer
        layers              % struct: static, dynamic, edge, blocking
        originXY            % world position of cell (1,1)
        yaw                 % grid orientation (ego heading)
        lastEgo
    end

    methods
        function obj = RiskMap(cfg, road, hazards)
            obj.cfg = cfg;
            obj.road = road;
            obj.hazards = hazards;

            r = cfg.risk;
            obj.res = r.res;
            obj.nx = round((r.aheadM + r.behindM) / r.res);
            obj.ny = round((2 * r.lateralM) / r.res);

            obj.xe = (-r.behindM + (0.5:obj.nx) * r.res);
            obj.ye = (-r.lateralM + (0.5:obj.ny) * r.res);
            [obj.XE, obj.YE] = meshgrid(obj.xe, obj.ye);

            % Prediction-step subsampling.  Stamping all 30 steps of every
            % mode of every track costs more than the whole cycle budget; the
            % spatial kernels are wide and heavily overlapping, so every third
            % step with three times the weight is visually and numerically
            % close.  Documented, not silent.
            T = round(cfg.predict.horizon / cfg.predict.dt);
            stride = 3;
            obj.tSub = 1:stride:T;

            % Discount weights are NORMALISED to sum to 1.  The literal
            % formula sums gamma^t over 30 steps, which is 9.58 - so a single
            % agent contributes a peak of w_c * 9.58, between 4.8 and 9.6, and
            % every cell within a few metres of anything clips to exactly 1.
            % Measured before this change: peak dynamic risk 4.29 for a
            % pedestrian and 4.93 for a bus, both saturating to 1.
            %
            % That is not a cosmetic scaling issue.  A saturated field makes
            % B1, B2 and B3 invisible to the planner: class weight, mode
            % probability and covariance growth all vanish into the clip, and
            % the "unified risk map" degrades into the binary occupancy grid
            % it is supposed to improve on.
            %
            % Normalising preserves the intent of gamma - the near future
            % still dominates the far future, in the same proportions - while
            % bounding one agent's contribution by w_c * m_ww.  Overlapping
            % agents can still saturate, which is correct: two road users in
            % one place genuinely is maximum risk.  See DEVIATIONS D23.
            w = r.gamma .^ (obj.tSub - 1);
            obj.wSub = w / sum(w);

            obj.R = zeros(obj.ny, obj.nx);
            obj.layers = struct('static', zeros(obj.ny, obj.nx), ...
                                'dynamic', zeros(obj.ny, obj.nx), ...
                                'edge', zeros(obj.ny, obj.nx), ...
                                'blocking', zeros(obj.ny, obj.nx), ...
                                'wrongSide', zeros(obj.ny, obj.nx));
        end

        % ----------------------------------------------------------------
        function update(obj, egoState, preds, tracks, wrongWayIds)
            %UPDATE  Recompute the field for the current cycle.
            if nargin < 5, wrongWayIds = []; end

            obj.lastEgo = egoState;
            obj.yaw = egoState.yaw;

            c = cos(egoState.yaw);
            s = sin(egoState.yaw);
            WX = egoState.x + c * obj.XE - s * obj.YE;
            WY = egoState.y + s * obj.XE + c * obj.YE;
            obj.originXY = [WX(1, 1), WY(1, 1)];

            obj.layers.static    = obj.staticLayer(WX, WY);
            obj.layers.blocking  = obj.blockingLayer(WX, WY);
            obj.layers.edge      = obj.edgeLayer(WX, WY);
            obj.layers.wrongSide = obj.wrongSideLayer(WX, WY, egoState);
            obj.layers.dynamic = obj.dynamicLayer(egoState, preds, tracks, wrongWayIds);

            r = obj.cfg.risk;
            base = r.wStatic * obj.layers.static + ...
                   r.wEdge   * obj.layers.edge + ...
                   obj.layers.dynamic;

            obj.R = min(max(base + r.wWrongSide * obj.layers.wrongSide, 0), 1);

            % The same field with the wrong-side preference removed.  Needed
            % because "is the overtaking corridor free?" must not be answered
            % "no, because it is the oncoming half" - crossing to the oncoming
            % half is what an overtake IS.  Asking the question against R
            % meant the wrong-side weight (0.40) always exceeded the clear
            % threshold (0.35), so an overtake could never be authorised and
            % the ego queued behind a 0.9 m/s pushcart for the whole run.
            obj.Rnoside = min(max(base, 0), 1);

            if strcmp(obj.cfg.riskMode, 'binary')
                % B1 ablation: the same geometry, but every risk above the
                % threshold becomes indistinguishable from every other.  The
                % planner can then only avoid or not avoid; it cannot prefer
                % the cheaper of two occupied routes.
                obj.R = double(obj.R >= r.binaryThreshold);
            end
        end

        % ----------------------------------------------------------------
        function [wx, wy] = toWorld(obj, ex, ey)
            c = cos(obj.yaw); s = sin(obj.yaw);
            wx = obj.lastEgo.x + c * ex - s * ey;
            wy = obj.lastEgo.y + s * ex + c * ey;
        end

        function [ex, ey] = toEgo(obj, wx, wy)
            c = cos(obj.yaw); s = sin(obj.yaw);
            dx = wx - obj.lastEgo.x;
            dy = wy - obj.lastEgo.y;
            ex =  c * dx + s * dy;
            ey = -s * dx + c * dy;
        end

        % ----------------------------------------------------------------
        function v = sampleEgo(obj, ex, ey)
            %SAMPLEEGO  Risk at ego-frame points; 1 outside the map.
            %
            %   Outside the grid is treated as maximum risk, not zero: the
            %   planner must not discover free space by leaving the field of
            %   view.
            cols = round((ex - obj.xe(1)) / obj.res) + 1;
            rows = round((ey - obj.ye(1)) / obj.res) + 1;
            inside = cols >= 1 & cols <= obj.nx & rows >= 1 & rows <= obj.ny;

            v = ones(size(ex));
            idx = sub2ind([obj.ny, obj.nx], rows(inside), cols(inside));
            v(inside) = obj.R(idx);
        end

        function v = sampleWorld(obj, wx, wy)
            [ex, ey] = obj.toEgo(wx, wy);
            v = obj.sampleEgo(ex, ey);
        end

        % ----------------------------------------------------------------
        function v = sampleWorldNoSide(obj, wx, wy)
            %SAMPLEWORLDNOSIDE  Risk excluding the wrong-side preference.
            %   For questions about whether a space is physically free, as
            %   distinct from whether it is the side we would prefer to use.
            [ex, ey] = obj.toEgo(wx, wy);
            cols = round((ex - obj.xe(1)) / obj.res) + 1;
            rows = round((ey - obj.ye(1)) / obj.res) + 1;
            inside = cols >= 1 & cols <= obj.nx & rows >= 1 & rows <= obj.ny;

            v = ones(size(ex));
            idx = sub2ind([obj.ny, obj.nx], rows(inside), cols(inside));
            v(inside) = obj.Rnoside(idx);
        end

        % ----------------------------------------------------------------
        function m = asStruct(obj)
            %ASSTRUCT  The riskMap contract (Section 4).
            m = struct('R', obj.R, 'res', obj.res, 'originXY', obj.originXY, ...
                'yaw', obj.yaw, 'nx', obj.nx, 'ny', obj.ny, ...
                'xe', obj.xe, 'ye', obj.ye, 'layers', obj.layers);
        end
    end

    % ====================================================================
    methods (Access = private)

        function S = staticLayer(obj, WX, WY)
            %STATICLAYER  Potholes, stalls, parked vehicles, broken edges (A1).
            if isempty(obj.hazards)
                S = zeros(obj.ny, obj.nx);
                return
            end
            S = obj.hazards.severity(WX, WY, obj.cfg.risk.staticMarginM);
        end

        % ----------------------------------------------------------------
        function B = blockingLayer(obj, WX, WY)
            %BLOCKINGLAYER  Only the impassable static hazards.
            %
            %   Separate from the static layer because the global planner
            %   needs to know what it cannot drive through, while the risk
            %   field needs to know what it should prefer not to.  A pothole
            %   is the second and not the first (see HazardSet).
            if isempty(obj.hazards)
                B = zeros(obj.ny, obj.nx);
                return
            end
            B = obj.hazards.blockingSeverity(WX, WY, 0.5);
        end

        % ----------------------------------------------------------------
        function W = wrongSideLayer(obj, WX, WY, egoState)
            %WRONGSIDELAYER  Elevated risk on the oncoming half of the road.
            %
            %   Built from the SAME expected-direction field the wrong-way
            %   detector reads: a cell whose expected travel direction opposes
            %   the ego's heading is a cell where oncoming traffic belongs.
            %
            %   IS THIS A LANE GRAPH?  No, and the distinction matters for the
            %   problem statement.  There are no lane centrelines, no lane
            %   ids, no discrete lane-change decisions - only a continuous
            %   scalar field over free space, exactly like the other layers.
            %   The planner is never told "you are in lane 1"; it is told that
            %   some ground is riskier than other ground, and it is free to
            %   use that ground when the alternative is worse.
            %
            %   WHY IT IS NEEDED.  Without it the planner has no reason to
            %   prefer its own half of the road.  Measured: avoiding a
            %   crossing pedestrian, the ego swerved right into the oncoming
            %   half, stopped there, and was struck head-on by a car that was
            %   entirely in the right.  On an unmarked two-way road the
            %   oncoming half genuinely is more dangerous, and a "unified risk
            %   map" that omits that is missing a real hazard.
            %
            %   The weight is deliberately below the planner's hard threshold,
            %   so this discourages crossing without forbidding it - an
            %   overtake around a stopped pushcart must stay possible.
            W = zeros(obj.ny, obj.nx);
            if isempty(obj.road)
                return
            end

            [c, s, defined] = obj.road.expectedDirection(WX(:), WY(:));
            dotp = c * cos(egoState.yaw) + s * sin(egoState.yaw);

            % Smooth ramp: fully "wrong side" when the expected direction is
            % squarely opposed, nothing when it agrees.  A smooth transition
            % keeps the centreline from becoming a cost cliff the planner
            % would oscillate across.
            w = min(max(-dotp, 0), 1);
            w(~defined) = 0;            % junctions have no wrong side
            W = reshape(w, obj.ny, obj.nx);
        end

        % ----------------------------------------------------------------
        function E = edgeLayer(obj, WX, WY)
            %EDGELAYER  Rising within edgeMarginM of the boundary, 1 outside.
            if isempty(obj.road)
                E = zeros(obj.ny, obj.nx);
                return
            end
            d = obj.road.distanceToEdge(WX(:), WY(:));
            d = reshape(d, obj.ny, obj.nx);

            margin = obj.cfg.risk.edgeMarginM;
            E = zeros(obj.ny, obj.nx);
            E(d <= 0) = 1;                          % off the carriageway
            band = d > 0 & d < margin;
            E(band) = 1 - d(band) / margin;         % smooth rise toward the edge
        end

        % ----------------------------------------------------------------
        function D = dynamicLayer(obj, egoState, preds, tracks, wrongWayIds)
            %DYNAMICLAYER  Predicted agent occupancy, class- and mode-weighted.
            D = zeros(obj.ny, obj.nx);
            if isempty(preds)
                return
            end

            trackIds = [tracks.trackId];

            for i = 1:numel(preds)
                p = preds(i);

                prior = classPriors(p.classId);
                if ~obj.cfg.classPriors
                    % B3 ablation: the system is denied class knowledge
                    % entirely, so BOTH the weight and the footprint become
                    % generic.  Ablating the weight but keeping a
                    % class-specific footprint would leave a back channel and
                    % understate what class knowledge is worth.
                    T = classPriors();
                    prior.riskWeight = mean([T.riskWeight]);
                    prior.length = mean([T.length]);
                    prior.width  = mean([T.width]);
                end
                wc = prior.riskWeight;

                % A2: a wrong-way agent is elevated, not merely detected.
                mww = 1.0;
                if ~isempty(wrongWayIds) && ismember(p.trackId, wrongWayIds)
                    mww = obj.cfg.risk.wrongWayMultiplier;
                end

                % Footprint covariance: a bus occupies more of the map than a
                % pedestrian even when both are perfectly localised.
                if isempty(trackIds) || ~any(trackIds == p.trackId)
                    F = eye(2) * 0.25;
                else
                    F = diag([(prior.length / 2)^2, (prior.width / 2)^2]);
                end

                if ~obj.cfg.uncertainty
                    % B2 ablation: no covariance growth, a fixed 1 m buffer.
                    F = eye(2) * 1.0^2;
                end

                for k = 1:numel(p.modes)
                    mode = p.modes(k);
                    D = obj.stampMode(D, egoState, mode, wc * mww, F);
                end
            end
        end

        % ----------------------------------------------------------------
        function D = stampMode(obj, D, egoState, mode, weight, F)
            %STAMPMODE  Add one mode's discounted occupancy to the field.
            %
            %   Each prediction step is stamped only over a local window of
            %   +-3 sigma.  Evaluating every kernel over the whole 360x120
            %   grid would be about 40 times more arithmetic for a
            %   contribution that is numerically zero almost everywhere.

            c = cos(egoState.yaw);
            s = sin(egoState.yaw);

            for n = 1:numel(obj.tSub)
                ti = obj.tSub(n);
                if ti > size(mode.mu, 1)
                    break
                end
                w = weight * mode.prob * obj.wSub(n);
                if w < 1e-3
                    continue    % below the resolution of a [0,1] field
                end

                % Predicted mean into the ego frame.
                dx = mode.mu(ti, 1) - egoState.x;
                dy = mode.mu(ti, 2) - egoState.y;
                mx =  c * dx + s * dy;
                my = -s * dx + c * dy;

                % Covariance into the ego frame, plus the footprint.
                Rot = [c, s; -s, c];
                Sig = Rot * (mode.Sigma(:, :, ti) + F) * Rot.';
                Sig = 0.5 * (Sig + Sig.') + eye(2) * 1e-6;

                sx = sqrt(Sig(1, 1));
                sy = sqrt(Sig(2, 2));
                reach = 3;

                c1 = floor((mx - reach*sx - obj.xe(1)) / obj.res) + 1;
                c2 = ceil( (mx + reach*sx - obj.xe(1)) / obj.res) + 1;
                r1 = floor((my - reach*sy - obj.ye(1)) / obj.res) + 1;
                r2 = ceil( (my + reach*sy - obj.ye(1)) / obj.res) + 1;
                c1 = max(c1, 1); c2 = min(c2, obj.nx);
                r1 = max(r1, 1); r2 = min(r2, obj.ny);
                if c1 > c2 || r1 > r2
                    continue    % this mode is entirely outside the map
                end

                X = obj.XE(r1:r2, c1:c2) - mx;
                Y = obj.YE(r1:r2, c1:c2) - my;

                % Peak-normalised kernel: exp(-0.5 * Mahalanobis^2).
                detS = Sig(1,1)*Sig(2,2) - Sig(1,2)*Sig(2,1);
                if detS <= 0
                    continue
                end
                iA =  Sig(2,2) / detS;
                iB = -Sig(1,2) / detS;
                iD =  Sig(1,1) / detS;
                q = iA .* X.^2 + 2 * iB .* X .* Y + iD .* Y.^2;

                D(r1:r2, c1:c2) = D(r1:r2, c1:c2) + w * exp(-0.5 * q);
            end
        end
    end
end
