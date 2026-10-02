classdef SimLog < handle
    %SIMLOG  Serialisable record of one run.
    %
    %   The log is the single source of truth for everything downstream:
    %   metrics, figures, the replay UI and the video all read it, and none of
    %   them re-simulates.  If a number appears anywhere in this project, it
    %   was computed from a log that a real run produced.
    %
    %   Storage is preallocated in chunks and trimmed on finalize, because
    %   growing a struct array one step at a time inside the loop would show
    %   up in the replanning-latency measurements.
    %
    %   See also SIMENGINE, METRICSLOGGER.

    properties (SetAccess = private)
        meta        % scenario, config, seed, environment, timestamps
        n = 0       % number of recorded steps
        t           % n x 1  simulation time
        ego         % n x 7  [x y yaw v a steer t]
        agents      % n x 1  cell, each a struct array of agentTruth
        cycles      % plan-cycle records (10 Hz), see appendCycle
        nCycles = 0
        eventLog    % struct array: t, type, text, data
                    % ("events" is a reserved classdef keyword - see DEVIATIONS D12)
        finalized = false
    end

    properties (Constant, Access = private)
        Chunk = 4096;
    end

    methods
        function obj = SimLog(meta)
            obj.meta = meta;
            obj.meta.createdAt = datetime('now');
            obj.t      = zeros(obj.Chunk, 1);
            obj.ego    = zeros(obj.Chunk, 7);
            obj.agents = cell(obj.Chunk, 1);
            obj.eventLog = struct('t', {}, 'type', {}, 'text', {}, 'data', {});
            obj.cycles = struct('t', {}, 'step', {}, 'latencyMs', {}, 'stage', {}, ...
                'state', {}, 'reason', {}, 'plannerUsed', {}, 'replanReason', {}, ...
                'tracks', {}, 'nDets', {}, 'risk', {}, 'riskMeta', {}, 'preds', {}, ...
                'wrongWayIds', {}, 'mergeIds', {}, 'mergeTTC', {}, ...
                'globalPath', {}, 'localTraj', {}, 'candidates', {}, ...
                'nRejected', {}, 'speedCap', {}, 'vTarget', {}, ...
                'corridorRisk', {}, 'minTTC', {}, 'minClear', {}, 'stageMs', {}, ...
                'antiStall', {});
        end

        % ----------------------------------------------------------------
        function append(obj, egoState, agentTruths)
            %APPEND  Record one simulation step.
            obj.n = obj.n + 1;
            if obj.n > numel(obj.t)
                obj.grow();
            end
            k = obj.n;
            obj.t(k) = egoState.t;
            obj.ego(k, :) = [egoState.x, egoState.y, egoState.yaw, ...
                             egoState.v, egoState.a, egoState.steer, egoState.t];
            obj.agents{k} = agentTruths;
        end

        % ----------------------------------------------------------------
        function appendCycle(obj, rec)
            %APPENDCYCLE  Record one 10 Hz planning cycle.
            %
            %   LatencyMs is wall-clock and is therefore the one field that is
            %   NOT reproducible on replay.  Determinism tests exclude it by
            %   name rather than by rounding.
            obj.nCycles = obj.nCycles + 1;
            f = {'t', 'step', 'latencyMs', 'stage', 'state', 'reason', ...
                 'plannerUsed', 'replanReason', 'tracks', 'nDets', ...
                 'risk', 'riskMeta', 'preds', 'wrongWayIds', 'mergeIds', 'mergeTTC', ...
                 'globalPath', 'localTraj', 'candidates', 'nRejected', 'speedCap', ...
                 'vTarget', 'corridorRisk', 'minTTC', 'minClear', 'stageMs', ...
                 'antiStall'};
            for i = 1:numel(f)
                if isfield(rec, f{i})
                    obj.cycles(obj.nCycles).(f{i}) = rec.(f{i});
                else
                    obj.cycles(obj.nCycles).(f{i}) = [];
                end
            end
        end

        % ----------------------------------------------------------------
        function addEvent(obj, t, type, text, data)
            %ADDEVENT  Append to the event log shown on the UI timeline.
            if nargin < 5, data = struct(); end
            k = numel(obj.eventLog) + 1;
            obj.eventLog(k).t = t;
            obj.eventLog(k).type = type;
            obj.eventLog(k).text = text;
            obj.eventLog(k).data = data;
        end

        % ----------------------------------------------------------------
        function finalize(obj, outcome)
            %FINALIZE  Trim the buffers and stamp the outcome.
            obj.t = obj.t(1:obj.n, :);
            obj.ego = obj.ego(1:obj.n, :);
            obj.agents = obj.agents(1:obj.n);
            obj.meta.outcome = outcome;
            obj.meta.duration = obj.t(max(obj.n, 1));
            obj.finalized = true;
        end

        % ----------------------------------------------------------------
        function st = egoStateAt(obj, k)
            %EGOSTATEAT  Rebuild the egoState contract for step k.
            r = obj.ego(k, :);
            st = struct('x', r(1), 'y', r(2), 'yaw', r(3), 'v', r(4), ...
                'a', r(5), 'steer', r(6), 't', r(7));
        end

        % ----------------------------------------------------------------
        function f = save(obj, file)
            %SAVE  Write the log to disk for replay and video rendering.
            d = fileparts(file);
            if ~isempty(d) && ~isfolder(d)
                mkdir(d);
            end
            log = obj; %#ok<NASGU>
            save(file, 'log', '-v7.3');
            f = file;
        end
    end

    methods (Access = private)
        function grow(obj)
            obj.t(end + obj.Chunk, 1) = 0;
            obj.ego(end + obj.Chunk, 7) = 0;
            obj.agents{end + obj.Chunk, 1} = [];
        end
    end
end
