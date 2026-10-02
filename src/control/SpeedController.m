classdef SpeedController < handle
    %SPEEDCONTROLLER  Jerk-limited PI longitudinal controller (Section 5.11).
    %
    %   Normal driving is jerk limited, because comfort is one of the reported
    %   metrics and an unlimited controller would produce step accelerations
    %   that flatter the smoothness numbers of whichever planner asks for them.
    %
    %   EMERGENCY_BRAKE deliberately bypasses the jerk limit and commands full
    %   deceleration.  That is the point of the state: the vehicle gives up
    %   comfort to stop.  The bypass is explicit and is reported in the metrics
    %   as an emergency-brake event rather than hidden inside the controller.
    %
    %   Integral wind-up is clamped, so a long period of being unable to reach
    %   the target (blocked behind a slow cart) does not produce a surge when
    %   the road clears.
    %
    %   See also KINEMATICBICYCLE, BEHAVIORFSM.

    properties (SetAccess = private)
        kp, ki
        maxAccel, comfortDecel, maxDecel, maxJerk
        integral
        lastAccel
    end

    methods
        function obj = SpeedController(cfg)
            obj.kp = cfg.control.speedKp;
            obj.ki = cfg.control.speedKi;
            obj.maxAccel     = cfg.vehicle.maxAccel;
            obj.comfortDecel = cfg.vehicle.comfortDecel;
            obj.maxDecel     = cfg.vehicle.maxDecel;
            obj.maxJerk      = cfg.vehicle.maxJerk;
            obj.integral  = 0;
            obj.lastAccel = 0;
        end

        % ----------------------------------------------------------------
        function a = step(obj, dt, v, vTarget, emergency)
            %STEP  Acceleration command for this cycle.
            if nargin < 5 || isempty(emergency)
                emergency = false;
            end

            if emergency
                % Full deceleration, no jerk limit, no PI state.
                a = -obj.maxDecel;
                obj.integral = 0;
                obj.lastAccel = a;
                return
            end

            err = vTarget - v;
            obj.integral = obj.integral + err * dt;

            % Anti-windup: the integral may never ask for more than the
            % actuator can deliver on its own.
            iLimit = obj.maxAccel / max(obj.ki, 1e-6);
            obj.integral = max(min(obj.integral, iLimit), -iLimit);

            a = obj.kp * err + obj.ki * obj.integral;

            % Comfort envelope, then jerk limit.
            a = max(min(a, obj.maxAccel), -obj.comfortDecel);
            dA = obj.maxJerk * dt;
            a = obj.lastAccel + max(min(a - obj.lastAccel, dA), -dA);

            obj.lastAccel = a;
        end

        % ----------------------------------------------------------------
        function reset(obj)
            obj.integral = 0;
            obj.lastAccel = 0;
        end
    end
end
