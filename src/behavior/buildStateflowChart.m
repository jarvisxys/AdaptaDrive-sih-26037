function out = buildStateflowChart(varargin)
%BUILDSTATEFLOWCHART  Generate models/AdaptaDrive_Behavior.slx programmatically.
%
%   OUT = BUILDSTATEFLOWCHART() builds a Stateflow chart holding the seven
%   behaviour states and the transitions between them, via the Stateflow API
%   (sfnew, Stateflow.State, Stateflow.Transition).
%
%   OUT fields
%       available  false when Stateflow or Simulink is missing
%       file       path to the .slx, '' when not built
%       states     the state names placed in the chart
%       reason     why it was not built, when it was not
%
%   WHAT THIS IS FOR, and what it is NOT
%   BehaviorFSM.m is the RUNTIME. This chart is a display and review artefact:
%   it makes the behaviour layer inspectable in the form reviewers expect, and
%   testStateflowParity checks it against the runtime on shared input traces.
%   No simulation result ever comes from this model. Keeping one authoritative
%   implementation is what stops the results and the diagram from drifting
%   apart - the usual failure being a slide that shows a chart nobody runs.
%
%   On a machine without Stateflow this returns available = false and the
%   parity test reports SKIPPED, never PASSED.
%
%   See also BEHAVIORFSM, BEHAVIORSTATE.

p = inputParser;
p.addParameter('file', adRoot('models', 'AdaptaDrive_Behavior.slx'));
p.addParameter('open', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opt = p.Results;

out = struct('available', false, 'file', '', 'states', {{}}, 'reason', '');

env = check_env('print', false);
if ~strcmp(env.backends.behaviorChart, 'stateflow')
    out.reason = 'Stateflow and/or Simulink unavailable on this machine';
    return
end

names = BehaviorState.allNames();
out.states = names;

d = fileparts(opt.file);
if ~isfolder(d), mkdir(d); end

modelName = 'AdaptaDrive_Behavior';

% Close and remove any previous copy so the build is reproducible.
try
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
catch
end
if isfile(opt.file)
    delete(opt.file);
end

try
    rt = sfroot;

    sfnew(modelName);
    ch = rt.find('-isa', 'Stateflow.Chart', '-and', 'Path', [modelName '/Chart']);
    if isempty(ch)
        charts = rt.find('-isa', 'Stateflow.Chart');
        ch = charts(end);
    end

    % --- data the guards read -----------------------------------------
    inputs = {'ttc', 'clearance', 'corridorRisk', 'egoSpeed', ...
              'mergeFlag', 'wrongWayNear', 'pathBlocked', ...
              'altCorridorClear', 'blockerSlow', 'rejoinDone'};
    for k = 1:numel(inputs)
        dIn = Stateflow.Data(ch);
        dIn.Name = inputs{k};
        dIn.Scope = 'Input';
    end
    dOut = Stateflow.Data(ch);
    dOut.Name = 'stateId';
    dOut.Scope = 'Output';

    % Context thresholds, so the chart shows that the guards are
    % parameterised by road context (B4) rather than hard-coded.
    for nm = {'ttcSlow', 'ttcEmergency'}
        dP = Stateflow.Data(ch);
        dP.Name = nm{1};
        dP.Scope = 'Input';
    end

    % --- states --------------------------------------------------------
    st = struct();
    cols = 3;
    for k = 1:numel(names)
        s = Stateflow.State(ch);
        s.Name = names{k};
        row = floor((k-1)/cols);
        col = mod(k-1, cols);
        s.Position = [60 + col*220, 60 + row*140, 170, 90];
        s.LabelString = sprintf('%s\nen: stateId = %d;', names{k}, k);
        st.(names{k}) = s;
    end

    % --- default transition into CRUISE --------------------------------
    t0 = Stateflow.Transition(ch);
    t0.Destination = st.CRUISE;
    t0.DestinationOClock = 0;
    t0.SourceEndPoint = st.CRUISE.Position(1:2) + [50 -45];
    t0.MidPoint = st.CRUISE.Position(1:2) + [50 -20];

    % --- transitions ----------------------------------------------------
    % Mirrors BehaviorFSM: the emergency override first, then the entry
    % ladder, then the recoveries.
    T = {
        'CRUISE',          'EMERGENCY_BRAKE', '[egoSpeed > 0.5 && (ttc < ttcEmergency || clearance < 0.5)]'
        'SLOW_DOWN',       'EMERGENCY_BRAKE', '[egoSpeed > 0.5 && (ttc < ttcEmergency || clearance < 0.5)]'
        'YIELD',           'EMERGENCY_BRAKE', '[egoSpeed > 0.5 && (ttc < ttcEmergency || clearance < 0.5)]'
        'STOP',            'EMERGENCY_BRAKE', '[egoSpeed > 0.5 && (ttc < ttcEmergency || clearance < 0.5)]'
        'OVERTAKE_MERGE',  'EMERGENCY_BRAKE', '[egoSpeed > 0.5 && (ttc < ttcEmergency || clearance < 0.5)]'
        'REJOIN',          'EMERGENCY_BRAKE', '[egoSpeed > 0.5 && (ttc < ttcEmergency || clearance < 0.5)]'
        'CRUISE',          'STOP',            '[pathBlocked]'
        'SLOW_DOWN',       'STOP',            '[pathBlocked]'
        'CRUISE',          'YIELD',           '[mergeFlag && ttc < 3]'
        'SLOW_DOWN',       'YIELD',           '[mergeFlag && ttc < 3]'
        'CRUISE',          'SLOW_DOWN',       '[ttc < ttcSlow || corridorRisk > 0.7 || wrongWayNear]'
        'SLOW_DOWN',       'OVERTAKE_MERGE',  '[blockerSlow && altCorridorClear]'
        'CRUISE',          'OVERTAKE_MERGE',  '[blockerSlow && altCorridorClear]'
        'OVERTAKE_MERGE',  'REJOIN',          '[rejoinDone]'
        'OVERTAKE_MERGE',  'SLOW_DOWN',       '[!altCorridorClear]'
        'REJOIN',          'CRUISE',          '[ttc > ttcSlow*1.25]'
        'YIELD',           'SLOW_DOWN',       '[!mergeFlag || ttc > 3*1.25]'
        'STOP',            'SLOW_DOWN',       '[!pathBlocked]'
        'EMERGENCY_BRAKE', 'SLOW_DOWN',       '[ttc > ttcEmergency*1.25 && clearance > 0.5*1.25 && !pathBlocked]'
        'EMERGENCY_BRAKE', 'STOP',            '[ttc > ttcEmergency*1.25 && clearance > 0.5*1.25 && pathBlocked]'
        'SLOW_DOWN',       'CRUISE',          '[ttc > ttcSlow*1.25 && corridorRisk < 0.35]'
        };

    for k = 1:size(T, 1)
        tr = Stateflow.Transition(ch);
        tr.Source = st.(T{k,1});
        tr.Destination = st.(T{k,2});
        tr.LabelString = T{k,3};
    end

    ch.Name = 'AdaptaDrive Behaviour (display artefact - BehaviorFSM.m is the runtime)';

    save_system(modelName, opt.file);
    if ~logical(opt.open)
        close_system(modelName, 0);
    end

    out.available = true;
    out.file = opt.file;
    fprintf('  Stateflow chart written to %s\n', opt.file);
    fprintf('  NOTE: display artefact only. BehaviorFSM.m is the runtime.\n');

catch ME
    out.reason = ME.message;
    fprintf('  Stateflow chart NOT built: %s\n', ME.message);
    try
        if bdIsLoaded(modelName), close_system(modelName, 0); end
    catch
    end
end
end
