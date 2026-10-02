classdef KinematicBicycle < handle
    %KINEMATICBICYCLE  Ego vehicle model (Section 5.11).
    %
    %   State is referenced to the REAR AXLE, which is the convention the
    %   kinematic bicycle equations are derived in:
    %
    %       xdot   = v cos(yaw)
    %       ydot   = v sin(yaw)
    %       yawdot = v / wheelbase * tan(steer)
    %       vdot   = a
    %
    %   Integrated with RK4 at the simulation step.  Euler at 0.05 s visibly
    %   drifts the heading on a curve, which would show up as a fake
    %   path-tracking error and pollute the smoothness metrics.
    %
    %   Actuation limits are enforced HERE, not in the planner, so no planner
    %   or baseline can quietly command something the vehicle cannot do:
    %     steer     clamped to maxSteer, slewed at maxSteerRate
    %     accel     clamped to [-maxDecel, +maxAccel]
    %
    %   The footprint is a box of length x width centred ahead of the rear
    %   axle by (length/2 - rearOverhang), so collision checks use the body,
    %   not the axle point.
    %
    %   See also PUREPURSUIT, SPEEDCONTROLLER, SIMENGINE.

    properties (SetAccess = private)
        p            % vehicle parameters (from cfg.vehicle)
        x, y, yaw    % rear-axle pose (m, m, rad)
        v            % longitudinal speed (m/s)
        a            % last applied acceleration (m/s^2)
        steer        % current road-wheel angle (rad)
        t            % simulation time (s)
        allowReverse
    end

    properties (Dependent)
        centerOffset % rear axle to body centre (m)
    end

    methods
        function obj = KinematicBicycle(cfg, init)
            %KINEMATICBICYCLE  init: struct with x, y, yaw and optional v.
            obj.p = cfg.vehicle;
            obj.x = init.x;
            obj.y = init.y;
            obj.yaw = init.yaw;
            obj.v = getOr(init, 'v', 0);
            obj.a = 0;
            obj.steer = getOr(init, 'steer', 0);
            obj.t = getOr(init, 't', 0);
            obj.allowReverse = getOr(init, 'allowReverse', false);
        end

        function c = get.centerOffset(obj)
            c = obj.p.length / 2 - obj.p.rearOverhang;
        end

        % ----------------------------------------------------------------
        function step(obj, dt, accelCmd, steerCmd)
            %STEP  Advance the vehicle by dt under a commanded accel and steer.

            % --- actuator limits -----------------------------------------
            steerCmd = max(min(steerCmd, obj.p.maxSteer), -obj.p.maxSteer);
            maxDelta = obj.p.maxSteerRate * dt;
            obj.steer = obj.steer + max(min(steerCmd - obj.steer, maxDelta), -maxDelta);

            accelCmd = max(min(accelCmd, obj.p.maxAccel), -obj.p.maxDecel);
            obj.a = accelCmd;

            % --- RK4 on [x y yaw v] --------------------------------------
            z = [obj.x; obj.y; obj.yaw; obj.v];
            f = @(zz) obj.deriv(zz, accelCmd);

            k1 = f(z);
            k2 = f(z + 0.5 * dt * k1);
            k3 = f(z + 0.5 * dt * k2);
            k4 = f(z + dt * k3);
            z = z + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);

            obj.x = z(1);
            obj.y = z(2);
            obj.yaw = wrapToPiLocal(z(3));
            obj.v = z(4);

            % A braking command must not push the vehicle backwards through
            % zero within a step.  Reverse is only reachable when explicitly
            % enabled (the local planner never commands it).
            if ~obj.allowReverse && obj.v < 0
                obj.v = 0;
                obj.a = 0;
            end

            obj.t = obj.t + dt;
        end

        % ----------------------------------------------------------------
        function o = footprint(obj, state)
            %FOOTPRINT  Body box in world coordinates.
            if nargin < 2
                state = obj.state();
            end
            off = obj.p.length / 2 - obj.p.rearOverhang;
            cx = state.x + off * cos(state.yaw);
            cy = state.y + off * sin(state.yaw);
            o = makeOBB(cx, cy, state.yaw, obj.p.length, obj.p.width);
        end

        % ----------------------------------------------------------------
        function st = state(obj)
            %STATE  The egoState contract (Section 4).
            st = struct('x', obj.x, 'y', obj.y, 'yaw', obj.yaw, 'v', obj.v, ...
                'a', obj.a, 'steer', obj.steer, 't', obj.t);
        end

        % ----------------------------------------------------------------
        function vel = velocity(obj)
            vel = [obj.v * cos(obj.yaw), obj.v * sin(obj.yaw)];
        end

        % ----------------------------------------------------------------
        function r = minTurningRadius(obj)
            %MINTURNINGRADIUS  Feeds the global planner (Section 5.10).
            r = obj.p.wheelbase / tan(obj.p.maxSteer);
        end
    end

    methods (Access = private)
        function dz = deriv(obj, z, accel)
            yaw = z(3);
            v = z(4);
            dz = [ v * cos(yaw)
                   v * sin(yaw)
                   v / obj.p.wheelbase * tan(obj.steer)
                   accel ];
        end
    end
end

% ========================================================================
function a = wrapToPiLocal(a)
%WRAPTOPILOCAL  Wrap to (-pi, pi] without needing a toolbox.
a = mod(a + pi, 2*pi) - pi;
end

function v = getOr(s, f, dflt)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f))
    v = s.(f);
else
    v = dflt;
end
end
