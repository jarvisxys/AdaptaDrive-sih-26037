function cfg = configPreset(name, varargin)
%CONFIGPRESET  Build the configuration for a named baseline or ablation.
%
%   CFG = CONFIGPRESET('BL1', 'scenario', 'village', 'seed', 3)
%
%   Every configuration is the SAME pipeline with components switched off.
%   None of them is a separate simulator: they share the vehicle model, the
%   sensors, the tracker and the controllers, so a difference in the results
%   is a difference in the thing being ablated and not in the physics.
%
%   CONFIGURATIONS
%     PROPOSED   everything on
%     BL1        LaneFollow: reference centreline + pure pursuit, longitudinal
%                IDM on the nearest tracked obstacle in a narrow corridor.
%                No risk map, no prediction, no planner, no FSM.  It is given
%                the road centreline, which the proposed planner never sees -
%                that is deliberately generous to the baseline and is stated
%                in ARCHITECTURE.md.
%     BL2        ReactiveCV: local planner only, binary occupancy,
%                class-agnostic constant-velocity prediction, no FSM context.
%     BL3        StaticOnly: full stack but every agent treated as frozen at
%                its current pose - no motion prediction at all.
%
%   ABLATIONS OF PROPOSED
%     -riskmap      risk thresholded to binary
%     -classpriors  one generic model and one weight for every class
%     -uncertainty  single deterministic prediction, fixed buffer
%     -context      one parameter set for every road type
%     -wrongside    no wrong-side preference in the risk map
%
%   Stress conditions (B5), all identical to PROPOSED but for the degradation
%     STRESS-sensor 20% camera dropout and 0.8 m position sigma
%   Agent-density stress takes no preset: RUN_EXPERIMENTS('density', 2.0).
%
%   See also DEFAULTCONFIG, SIMENGINE, RUN_EXPERIMENTS.

cfg = defaultConfig(varargin{:});
cfg.name = char(name);

switch cfg.name
    case 'PROPOSED'
        % everything on

    case 'BL1'
        cfg.useRiskMap = false;
        cfg.useGlobalPlanner = false;
        cfg.useDWA = false;
        cfg.useFSM = false;
        cfg.context = false;

    case 'BL2'
        cfg.useGlobalPlanner = false;
        cfg.useFSM = false;
        cfg.riskMode = 'binary';
        cfg.classPriors = false;
        cfg.uncertainty = false;
        cfg.context = false;
        cfg.risk.wWrongSide = 0;

    case 'BL3'
        cfg.freezeAgents = true;

    case '-riskmap'
        cfg.riskMode = 'binary';

    case '-classpriors'
        cfg.classPriors = false;

    case '-uncertainty'
        cfg.uncertainty = false;

    case '-context'
        cfg.context = false;

    case '-wrongside'
        cfg.risk.wWrongSide = 0;

    % --- B5 stress conditions -------------------------------------------
    % Identical to PROPOSED except for the stated degradation, so a difference
    % in the results is attributable to that degradation and nothing else.
    % Density stress needs no preset: run_experiments takes 'density'.
    case 'STRESS-sensor'
        % A camera four times worse at detecting and nearly three times worse
        % at localising. The radars are untouched on purpose: if every sensor
        % degrades at once, a drop in performance says nothing about which
        % sensor the stack actually depends on.
        cfg.stress.name = 'sensor';
        cfg.stress.cameraDropout = 0.20;    % nominal 0.05
        cfg.stress.cameraPosSigma = 0.8;    % nominal 0.3 m

    case 'scripted'
        % M1 scaffold: ground-truth gap keeping, never reported.

    otherwise
        error('configPreset:unknown', ...
            ['Unknown configuration "%s". Known: PROPOSED, BL1, BL2, BL3, ' ...
             '-riskmap, -classpriors, -uncertainty, -context, -wrongside, ' ...
             'STRESS-sensor.'], cfg.name);
end
end
