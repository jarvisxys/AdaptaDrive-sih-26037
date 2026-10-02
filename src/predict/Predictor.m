classdef Predictor < handle
    %PREDICTOR  Class-conditioned multi-modal motion prediction (B2 / B3).
    %
    %   For every track, produce one or more weighted future trajectories with
    %   growing uncertainty, over a 3 s horizon at 0.1 s steps.
    %
    %   PREDICTION CONTRACT (Section 4)
    %       pred(i).trackId
    %       pred(i).classId
    %       pred(i).modes(k).prob      mode probability, sums to 1 over k
    %       pred(i).modes(k).mu        T x 2 predicted positions
    %       pred(i).modes(k).Sigma     2 x 2 x T position covariance
    %       pred(i).modes(k).name      'cv' | 'cross' | 'enter' | 'stay'
    %
    %   WHY MULTI-MODAL (B2)
    %   A pedestrian at the edge of the road is not "going to be somewhere near
    %   where they are now, plus noise".  They are either continuing along the
    %   edge or stepping into the road, and those two futures are metres apart.
    %   A unimodal predictor averages them into a place the pedestrian will
    %   never be, and the planner then reserves space where nobody is going
    %   while leaving the crossing path clear.  Modes keep the two futures
    %   separate and let the risk map carry both.
    %
    %   WHY CLASS-CONDITIONED (B3)
    %   Covariance grows as Sigma(t) = P_track + Q_class * t, with Q_class
    %   rotated into each agent's heading frame so longitudinal and lateral
    %   uncertainty differ.  A bus may not be a metre sideways in a second; a
    %   two-wheeler may.  That asymmetry is the entire point - an isotropic
    %   blob would make the planner treat a bus like a two-wheeler.
    %
    %   ABLATIONS
    %     cfg.classPriors = false   one generic constant-velocity model for
    %                               every class, with a single averaged Q
    %     cfg.uncertainty = false   one deterministic trajectory per track and
    %                               a fixed 1 m buffer instead of covariance
    %
    %   The predictor reads TRACKS, never ground truth.  It may consult the
    %   RoadModel for drivable area and expected direction - that is the
    %   unstructured-road equivalent of knowing where the road is, and it is
    %   not a lane graph.
    %
    %   See also RISKMAP, CLASSPRIORS, TRACKERWRAPPER.

    properties (SetAccess = private)
        cfg
        road
        horizon
        dt
        T           % number of prediction steps
        gamma
    end

    methods
        function obj = Predictor(cfg, road)
            obj.cfg = cfg;
            obj.road = road;
            obj.horizon = cfg.predict.horizon;
            obj.dt = cfg.predict.dt;
            obj.T = round(obj.horizon / obj.dt);
            obj.gamma = cfg.risk.gamma;
        end

        % ----------------------------------------------------------------
        function preds = predict(obj, tracks)
            %PREDICT  One prediction entry per track.
            preds = Predictor.emptyPredictions();
            if isempty(tracks)
                return
            end

            out = cell(1, numel(tracks));
            for k = 1:numel(tracks)
                out{k} = obj.predictOne(tracks(k));
            end
            preds = [out{:}];
        end
    end

    % ====================================================================
    methods (Access = private)

        function p = predictOne(obj, tr)
            prior = obj.priorFor(tr.classId);

            p = struct('trackId', tr.trackId, 'classId', tr.classId, 'modes', []);

            if ~obj.cfg.uncertainty
                % B2 ablation: one deterministic trajectory, no covariance
                % growth.  The fixed buffer that replaces it is applied by the
                % consumer (RiskMap), so the difference is visible there
                % rather than hidden in a covariance that is secretly tiny.
                mu = obj.constantVelocity(tr, prior, 0);
                Sigma = repmat(eye(2) * 1e-6, 1, 1, obj.T);
                p.modes = obj.makeMode('cv', 1.0, mu, Sigma);
                return
            end

            model = prior.motionModel;
            if ~obj.cfg.classPriors
                model = 'cv';       % B3 ablation: one generic model
            end

            switch model
                case 'pedestrian'
                    p.modes = obj.pedestrianModes(tr, prior);
                case 'cow'
                    p.modes = obj.cowModes(tr, prior);
                case 'pushcart'
                    p.modes = obj.stationaryBiasedModes(tr, prior);
                otherwise
                    p.modes = obj.vehicleModes(tr, prior);
            end
        end

        % ----------------------------------------------------------------
        function modes = vehicleModes(obj, tr, prior)
            %VEHICLEMODES  Car, bus, auto, two-wheeler, unknown: single CV mode.
            mu = obj.constantVelocity(tr, prior, 0);
            Sigma = obj.growCovariance(tr, prior, obj.headingOf(tr));
            modes = obj.makeMode('cv', 1.0, mu, Sigma);
        end

        % ----------------------------------------------------------------
        function modes = pedestrianModes(obj, tr, prior)
            %PEDESTRIANMODES  Continue along the edge, or cross the road.
            %
            %   The crossing probability rises with two observable cues:
            %   whether the pedestrian is already moving toward the road, and
            %   how close to the carriageway they are.  Both come from tracks
            %   and the drivable-area field - no intent oracle.

            [inward, dOff, onRoad] = obj.roadGeometryAt(tr);

            speed = hypot(tr.vx, tr.vy);
            vLat = 0;
            if ~isempty(inward)
                vLat = tr.vx * inward(1) + tr.vy * inward(2);   % + = toward road
            end

            % Cue 1: motion toward the road, saturating at a brisk 0.8 m/s.
            fHeading = min(max(vLat, 0) / 0.8, 1);

            % Cue 2: proximity.  A pedestrian already in the road is committed;
            % one far from the edge is not an immediate crossing risk.
            if isempty(inward)
                fProx = 0.5;
            elseif onRoad
                fProx = 1.0;
            else
                fProx = max(0, 1 - max(abs(dOff) - 3.0, 0) / 4.0);
            end

            pCross = 0.15 + 0.75 * fHeading * fProx;
            pCross = min(max(pCross, 0.10), 0.90);

            % Mode 1: continue as observed.
            muA = obj.constantVelocity(tr, prior, 0);
            SigA = obj.growCovariance(tr, prior, obj.headingOf(tr));

            % Mode 2: cross toward the far side at a walking pace.
            crossSpeed = max(speed, 1.1);
            if isempty(inward)
                % No road reference: cross perpendicular to current motion.
                h = obj.headingOf(tr);
                inward = [-sin(h), cos(h)];
            end
            vCross = inward * crossSpeed;
            muB = obj.straightLine([tr.x tr.y], vCross);
            SigB = obj.growCovariance(tr, prior, atan2(vCross(2), vCross(1)));

            modes = [obj.makeMode('cv', 1 - pCross, muA, SigA), ...
                     obj.makeMode('cross', pCross, muB, SigB)];
        end

        % ----------------------------------------------------------------
        function modes = cowModes(obj, tr, prior)
            %COWMODES  Mostly stays put, occasionally walks into the road.
            %
            %   Large ISOTROPIC growth: unlike a vehicle, an animal's next
            %   metre carries no directional cue, so spreading uncertainty
            %   along a heading it happens to have would be false precision.

            [inward, ~, onRoad] = obj.roadGeometryAt(tr);

            pEnter = 0.15;
            if onRoad
                pEnter = 0.35;      % already committed to the carriageway
            end

            muA = obj.straightLine([tr.x tr.y], [tr.vx tr.vy] * 0.3);
            SigA = obj.growCovarianceIsotropic(tr, prior);

            if isempty(inward)
                h = obj.headingOf(tr);
                inward = [cos(h), sin(h)];
            end
            muB = obj.straightLine([tr.x tr.y], inward * 1.0);
            SigB = obj.growCovarianceIsotropic(tr, prior);

            modes = [obj.makeMode('stay', 1 - pEnter, muA, SigA), ...
                     obj.makeMode('enter', pEnter, muB, SigB)];
        end

        % ----------------------------------------------------------------
        function modes = stationaryBiasedModes(obj, tr, prior)
            %STATIONARYBIASEDMODES  Pushcart: slow, near-constant, small spread.
            mu = obj.constantVelocity(tr, prior, 0);
            Sigma = obj.growCovariance(tr, prior, obj.headingOf(tr));
            modes = obj.makeMode('cv', 1.0, mu, Sigma);
        end

        % ----------------------------------------------------------------
        function mu = constantVelocity(obj, tr, ~, accel)
            v = [tr.vx, tr.vy];
            if nargin >= 4 && accel ~= 0
                sp = hypot(v(1), v(2));
                if sp > 1e-6
                    v = v + accel * obj.dt * v / sp;
                end
            end
            mu = obj.straightLine([tr.x tr.y], v);
        end

        function mu = straightLine(obj, p0, vel)
            tvec = (1:obj.T)' * obj.dt;
            mu = [p0(1) + vel(1) * tvec, p0(2) + vel(2) * tvec];
        end

        % ----------------------------------------------------------------
        function Sigma = growCovariance(obj, tr, prior, heading)
            %GROWCOVARIANCE  Sigma(t) = P_track + R * diag(qLon,qLat)*t * R'
            %
            %   Rotated into the agent's heading frame, so the longitudinal
            %   and lateral growth rates from CLASSPRIORS actually mean
            %   longitudinal and lateral.
            P0 = obj.trackCovariance(tr);
            c = cos(heading); s = sin(heading);
            R = [c, -s; s, c];
            Q = R * diag([prior.qLon, prior.qLat]) * R.';

            Sigma = zeros(2, 2, obj.T);
            for k = 1:obj.T
                Sigma(:, :, k) = P0 + Q * (k * obj.dt);
            end
        end

        function Sigma = growCovarianceIsotropic(obj, tr, prior)
            P0 = obj.trackCovariance(tr);
            q = max(prior.qLon, prior.qLat);
            Sigma = zeros(2, 2, obj.T);
            for k = 1:obj.T
                Sigma(:, :, k) = P0 + eye(2) * q * (k * obj.dt);
            end
        end

        function P0 = trackCovariance(~, tr)
            P0 = tr.P;
            if isempty(P0) || any(~isfinite(P0(:)))
                P0 = eye(2) * 0.5;
            end
            % Keep it symmetric positive definite: it is inverted downstream.
            P0 = 0.5 * (P0 + P0.');
            P0 = P0 + eye(2) * 1e-4;
        end

        % ----------------------------------------------------------------
        function h = headingOf(~, tr)
            if isfinite(tr.heading)
                h = tr.heading;
            elseif hypot(tr.vx, tr.vy) > 1e-6
                h = atan2(tr.vy, tr.vx);
            else
                h = 0;
            end
        end

        % ----------------------------------------------------------------
        function [inward, dOff, onRoad] = roadGeometryAt(obj, tr)
            %ROADGEOMETRYAT  Unit vector from the agent toward the road centre.
            %
            %   Taken as the direction to the nearest centreline point, which
            %   is exact on a curved road and needs no reasoning about which
            %   side of the tangent the agent is on.  Empty when there is no
            %   road reference or the agent is already on the centreline.
            inward = [];
            dOff = NaN;
            onRoad = false;
            if isempty(obj.road)
                return
            end

            info = obj.road.nearest(tr.x, tr.y);
            dOff = info.d;
            onRoad = obj.road.isDrivable(tr.x, tr.y);

            if info.seg < 1 || info.seg > numel(obj.road.segments)
                return
            end
            centre = obj.road.pointAt(info.seg, info.s, 0);
            vec = centre - [tr.x, tr.y];
            n = hypot(vec(1), vec(2));
            if n > 1e-6
                inward = vec / n;
            end
        end

        % ----------------------------------------------------------------
        function prior = priorFor(obj, classId)
            if obj.cfg.classPriors
                prior = classPriors(classId);
            else
                % B3 ablation: one averaged model applied to everything.
                T = classPriors();
                prior = classPriors(0);
                prior.qLon = mean([T.qLon]);
                prior.qLat = mean([T.qLat]);
                prior.motionModel = 'cv';
                prior.riskWeight = mean([T.riskWeight]);
            end
        end

        function m = makeMode(~, name, prob, mu, Sigma)
            % Heading of the predicted motion, per step, so consumers can
            % place the agent's BODY along its path rather than treating it as
            % a point or a disc.  Derived from the mean sequence; a mode that
            % barely moves keeps the first well-defined heading, and one that
            % never moves reports NaN so the consumer can fall back rather
            % than silently assume an orientation.
            d = diff(mu, 1, 1);
            h = [atan2(d(:,1)*0 + d(:,2), d(:,1)); NaN];
            h(end) = h(max(end-1, 1));
            moved = hypot([d(:,1); 0], [d(:,2); 0]) > 1e-3;
            if any(moved)
                first = find(moved, 1);
                h(~moved) = h(first);
            else
                h(:) = NaN;
            end
            m = struct('name', name, 'prob', prob, 'mu', mu, 'Sigma', Sigma, ...
                'heading', h);
        end
    end

    % ====================================================================
    methods (Static)
        function p = emptyPredictions()
            p = struct('trackId', {}, 'classId', {}, 'modes', {});
        end
    end
end
