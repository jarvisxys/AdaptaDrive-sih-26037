classdef PurePursuit < handle
    %PUREPURSUIT  Lateral controller with a speed-scheduled lookahead.
    %
    %   Ld = clamp(lookaheadBase + lookaheadGain * v, min, max)
    %   steer = atan2(2 * wheelbase * sin(alpha), Ld)
    %
    %   where alpha is the angle from the vehicle heading to the lookahead
    %   point, measured at the REAR AXLE (the reference point of the bicycle
    %   model).
    %
    %   TWO BACKENDS
    %     'native'   the law above, implemented here
    %     'toolbox'  controllerPurePursuit
    %
    %   The toolbox controller's SECOND OUTPUT IS CURVATURE (1/m), NOT the
    %   angular velocity its documentation-era signature [v, omega] suggests.
    %   Measured on R2026a: the value is invariant under DesiredLinearVelocity
    %   and equals 1/R on a circle of radius R (see DEVIATIONS.md D13).  So the
    %   conversion is
    %       steer = atan(kappa * wheelbase)
    %   with NO division by speed.  Dividing by v, as an omega reading would
    %   require, under-steers by a factor of v - at 6.9 m/s that walked the ego
    %   off a 6 m road in 30 m.
    %
    %   check_env selects the backend; testPurePursuit asserts the two agree
    %   on the same path, so the fallback is a real equivalent rather than an
    %   untested branch.
    %
    %   See also KINEMATICBICYCLE, CHECK_ENV.

    properties (SetAccess = private)
        wheelbase
        base, gain, ldMin, ldMax
        backend
        ppObj            % controllerPurePursuit instance, toolbox backend only
        lastLookahead    % for the UI overlay
        lastTargetXY
    end

    methods
        function obj = PurePursuit(cfg, backend)
            obj.wheelbase = cfg.vehicle.wheelbase;
            obj.base  = cfg.control.lookaheadBase;
            obj.gain  = cfg.control.lookaheadGain;
            obj.ldMin = cfg.control.lookaheadMin;
            obj.ldMax = cfg.control.lookaheadMax;

            if nargin < 2 || isempty(backend)
                backend = cfg.env.backends.purePursuit;
            end
            if strcmp(backend, 'toolbox') && exist('controllerPurePursuit', 'file') == 0
                backend = 'native';   % never claim a backend that is absent
            end
            obj.backend = backend;

            if strcmp(obj.backend, 'toolbox')
                obj.ppObj = controllerPurePursuit;
            end

            obj.lastLookahead = obj.ldMin;
            obj.lastTargetXY = [NaN NaN];
        end

        % ----------------------------------------------------------------
        function Ld = lookahead(obj, v)
            %LOOKAHEAD  Speed-scheduled lookahead distance.
            Ld = min(max(obj.base + obj.gain * v, obj.ldMin), obj.ldMax);
        end

        % ----------------------------------------------------------------
        function [steer, info] = step(obj, state, path)
            %STEP  Road-wheel angle to track PATH from the current STATE.
            %
            %   PATH is an n x 2 (or n x 3+) array of waypoints; extra columns
            %   are ignored so a local trajectory [x y yaw v] can be passed
            %   straight in.

            info = struct('Ld', NaN, 'target', [NaN NaN], 'alpha', NaN, ...
                'backend', obj.backend, 'valid', false);

            if isempty(path) || size(path, 1) < 2
                steer = 0;
                return
            end
            P = path(:, 1:2);

            Ld = obj.lookahead(state.v);
            obj.lastLookahead = Ld;
            info.Ld = Ld;

            switch obj.backend
                case 'toolbox'
                    obj.ppObj.Waypoints = P;
                    obj.ppObj.LookaheadDistance = Ld;
                    obj.ppObj.DesiredLinearVelocity = max(state.v, 0.5);
                    [~, kappa] = obj.ppObj([state.x; state.y; state.yaw]);
                    % kappa is CURVATURE in 1/m (verified, DEVIATIONS D13).
                    steer = atan(kappa * obj.wheelbase);
                    info.kappa = kappa;
                    info.valid = true;

                otherwise
                    [target, found] = obj.findTarget(state, P, Ld);
                    info.target = target;
                    obj.lastTargetXY = target;
                    if ~found
                        steer = 0;
                        return
                    end
                    dx = target(1) - state.x;
                    dy = target(2) - state.y;
                    alpha = wrapToPiLocal(atan2(dy, dx) - state.yaw);
                    info.alpha = alpha;
                    % Use the true distance to the chosen target rather than
                    % the nominal Ld: near the end of a path the target may be
                    % closer than Ld, and using Ld there under-steers.
                    dist = max(hypot(dx, dy), 1e-3);
                    steer = atan2(2 * obj.wheelbase * sin(alpha), dist);
                    info.valid = true;
            end

            steer = wrapToPiLocal(steer);
        end
    end

    methods (Access = private)
        function [target, found] = findTarget(obj, state, P, Ld)
            %FINDTARGET  First point on the path at distance Ld ahead.
            %
            %   Searches forward from the closest point so the controller
            %   cannot latch onto a part of the path it has already driven
            %   (which happens on a hairpin, where an earlier segment is
            %   geometrically nearer than the one being tracked).

            d = hypot(P(:,1) - state.x, P(:,2) - state.y);
            [~, iNear] = min(d);

            target = P(end, :);
            found = false;

            for k = iNear:size(P, 1) - 1
                a = P(k, :);
                b = P(k+1, :);
                [hit, pt] = circleSegmentForward(state, a, b, Ld);
                if hit
                    target = pt;
                    found = true;
                    return
                end
            end

            % No intersection: the path ends inside the lookahead circle.  Aim
            % at the final waypoint, which is the correct behaviour on the
            % last stretch to the goal.
            if hypot(P(end,1) - state.x, P(end,2) - state.y) > 1e-3
                found = true;
            end
        end
    end
end

% ========================================================================
function [hit, pt] = circleSegmentForward(state, a, b, Ld)
%CIRCLESEGMENTFORWARD  Intersection of segment a-b with the lookahead circle.
%   Returns the intersection with the larger parameter (further along the
%   path), and only when it lies ahead of the vehicle.

hit = false;
pt = [NaN NaN];

d = b - a;
f = a - [state.x, state.y];

A = dot(d, d);
if A < 1e-12
    return
end
B = 2 * dot(f, d);
C = dot(f, f) - Ld^2;

disc = B*B - 4*A*C;
if disc < 0
    return
end
disc = sqrt(disc);

for tCand = [(-B + disc) / (2*A), (-B - disc) / (2*A)]
    if tCand >= 0 && tCand <= 1
        cand = a + tCand * d;
        % Must be in front of the vehicle, not behind it.
        rel = [cand(1) - state.x, cand(2) - state.y];
        if rel(1) * cos(state.yaw) + rel(2) * sin(state.yaw) > 0
            pt = cand;
            hit = true;
            return
        end
    end
end
end

% ========================================================================
function a = wrapToPiLocal(a)
a = mod(a + pi, 2*pi) - pi;
end
