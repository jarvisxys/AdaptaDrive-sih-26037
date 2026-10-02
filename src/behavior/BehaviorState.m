classdef BehaviorState < uint8
    %BEHAVIORSTATE  The seven behaviour states, in the team deck's order.
    %
    %   Backed by uint8 so a state can be written into a log, a CSV row or a
    %   Stateflow chart's data without conversion, and compared cheaply inside
    %   the loop.
    %
    %   See also BEHAVIORFSM, BUILDSTATEFLOWCHART.

    enumeration
        CRUISE          (1)
        SLOW_DOWN       (2)
        YIELD           (3)
        OVERTAKE_MERGE  (4)
        REJOIN          (5)
        STOP            (6)
        EMERGENCY_BRAKE (7)
    end

    methods (Static)
        function names = allNames()
            names = {'CRUISE', 'SLOW_DOWN', 'YIELD', 'OVERTAKE_MERGE', ...
                     'REJOIN', 'STOP', 'EMERGENCY_BRAKE'};
        end

        function s = fromName(name)
            s = BehaviorState.(char(name));
        end
    end
end
