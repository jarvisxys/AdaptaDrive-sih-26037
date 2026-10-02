function out = contextParams(context, cfg)
%CONTEXTPARAMS  Context-adaptive driving parameters (B4).
%
%   P = CONTEXTPARAMS(context)        parameters for one road context
%   P = CONTEXTPARAMS(context, cfg)   honours the cfg.context ablation switch
%   T = CONTEXTPARAMS()               the whole table (struct array)
%
%   Contexts: village | intersection | highway | market | rural
%
%   This IS requirement B4.  The same planner behaves differently on a 5 m
%   market lane and a 400 m highway merge because this table changes under it;
%   nothing else in the pipeline is swapped.  The UI shows the active row so
%   the audience can see which parameter set is driving.
%
%   Ablation: with cfg.context == false every context returns the 'fixed' row,
%   which is the village row applied everywhere.  That is the honest way to
%   ablate - a single set that is reasonable somewhere, not a crippled one.
%
%   Fields
%     speedCap          m/s   upper speed limit for this context
%     followingGap      m     desired longitudinal gap to a leading agent
%     lateralClearance  m     desired lateral margin to any agent or edge
%     ttcSlow           s     TTC below which the FSM enters SLOW_DOWN
%     ttcEmergency      s     TTC below which the FSM enters EMERGENCY_BRAKE
%     replanPeriod      s     periodic global-replanning interval
%     dwaWeights        struct cost weights for the local planner
%
%   WEIGHT BALANCE, and why progress outweighs risk
%   Every DWA cost term is bounded in [0,1] and the weights sum to 1, so a
%   weight reads directly as "what share of the decision does this get".  An
%   earlier balance gave risk the largest share (0.357 in village) against
%   goal + speed combined (0.32).  On a road where merely BEING on the road
%   carries risk of roughly 0.25 - edge proximity, the wrong-side field, other
%   traffic - that made standing still cheaper than driving, and the vehicle
%   stalled mid-route with a clear road ahead, 10 s TTC and 10 m of clearance.
%
%   Progress now takes the larger share.  Risk still shapes WHERE the vehicle
%   drives, which is its job; it no longer decides WHETHER it drives, which
%   was never its job - the hard rejects and the behaviour states exist for
%   that, and they are not weights.
%
%   Values are DESIGN PARAMETERS tuned only on seeds 101-110 (freeze rule).
%
%   See also CLASSPRIORS, DEFAULTCONFIG, BEHAVIORFSM.

T = buildTable();

if nargin == 0
    out = T;
    return
end

if nargin >= 2 && isstruct(cfg) && isfield(cfg, 'context') && ~cfg.context
    context = 'fixed';   % B4 ablation: one parameter set everywhere
end

context = char(context);
idx = find(strcmp({T.name}, context), 1);
if isempty(idx)
    error('contextParams:unknownContext', ...
        'Unknown road context "%s". Known: %s.', context, strjoin({T.name}, ', '));
end
out = T(idx);
end

% ------------------------------------------------------------------------
function T = buildTable()
persistent cached
if ~isempty(cached)
    T = cached;
    return
end

kmh = @(v) v / 3.6;

T = struct('name', {}, 'speedCap', {}, 'followingGap', {}, 'lateralClearance', {}, ...
    'ttcSlow', {}, 'ttcEmergency', {}, 'replanPeriod', {}, 'dwaWeights', {}, 'note', {});

% --- village: narrow, unmarked, mixed slow traffic, poor surface ----------
T(1).name = 'village';
T(1).speedCap         = kmh(25);
T(1).followingGap     = 8.0;
T(1).lateralClearance = 0.8;
T(1).ttcSlow          = 4.0;
T(1).ttcEmergency     = 1.5;
T(1).replanPeriod     = 0.5;
T(1).dwaWeights       = weights(2.0, 1.4, 1.0, 0.4, 1.2);
T(1).note = 'Unmarked 6 m road: moderate speed, generous lateral margin for edge hazards.';

% --- intersection: no signals, right-of-way must be inferred -------------
T(2).name = 'intersection';
T(2).speedCap         = kmh(20);
T(2).followingGap     = 6.0;
T(2).lateralClearance = 0.9;
T(2).ttcSlow          = 4.5;      % react earlier: conflicts arrive laterally
T(2).ttcEmergency     = 1.8;
T(2).replanPeriod     = 0.3;      % replan faster: the conflict set changes quickly
T(2).dwaWeights       = weights(1.8, 1.8, 1.2, 0.4, 1.0);
T(2).note = 'Unsignalised crossing: earliest TTC thresholds and fastest replanning.';

% --- highway: high speed, large speed differentials -----------------------
T(3).name = 'highway';
T(3).speedCap         = kmh(60);
T(3).followingGap     = 20.0;
T(3).lateralClearance = 1.0;
T(3).ttcSlow          = 5.0;      % highest absolute TTC: stopping distance dominates
T(3).ttcEmergency     = 1.5;
T(3).replanPeriod     = 0.5;
T(3).dwaWeights       = weights(2.0, 1.2, 1.0, 0.8, 1.6);  % speed and smoothness matter most
T(3).note = 'Slow-merge highway: long gaps, smooth high-speed trajectories.';

% --- market: crawling speed, permanent clutter ---------------------------
T(4).name = 'market';
T(4).speedCap         = kmh(10);
T(4).followingGap     = 3.0;
T(4).lateralClearance = 0.6;      % a 0.8 m margin would make a 5 m lane impassable
T(4).ttcSlow          = 3.5;
T(4).ttcEmergency     = 1.2;      % low speed makes a late brake survivable
T(4).replanPeriod     = 0.25;     % highest replanning load by design
T(4).dwaWeights       = weights(1.6, 2.0, 1.4, 0.3, 0.9);
T(4).note = 'Dense market: tightest clearances, slowest cap, heaviest risk weighting.';

% --- rural: open two-lane road, animal and oncoming hazards --------------
T(5).name = 'rural';
T(5).speedCap         = kmh(40);
T(5).followingGap     = 12.0;
T(5).lateralClearance = 0.9;
T(5).ttcSlow          = 4.5;
T(5).ttcEmergency     = 1.6;
T(5).replanPeriod     = 0.5;
T(5).dwaWeights       = weights(2.0, 1.4, 1.1, 0.6, 1.3);
T(5).note = 'Rural two-lane: moderate speed with headroom for an abrupt animal entry.';

% --- fixed: the B4 ablation row ------------------------------------------
T(6) = T(1);
T(6).name = 'fixed';
T(6).note = 'B4 ABLATION: the village parameter set applied to every context.';

cached = T;
end

% ------------------------------------------------------------------------
function w = weights(goal, risk, clear, smooth, speed)
%WEIGHTS  Local-planner cost weights (Section 5.10).
%
%   NORMALISED TO SUM TO 1.  Every DWA cost term is bounded in [0,1], so the
%   weights are a partition of a unit budget and each one reads directly as
%   "what fraction of the decision does this consideration get".  Arguments
%   are given in relative terms and normalised here, so the table stays
%   readable while the invariant is enforced in one place.
%
%   The invariant matters: four separate planner defects came from a term
%   escaping its expected scale, and unnormalised weights made them invisible
%   because no single number looked wrong on its own.
total = goal + risk + clear + smooth + speed;
if total <= 0
    error('contextParams:badWeights', 'Cost weights must be positive.');
end
w = struct('goal', goal/total, 'risk', risk/total, 'clear', clear/total, ...
           'smooth', smooth/total, 'speed', speed/total);

% Guard the invariant the planner relies on.
assert(abs(w.goal + w.risk + w.clear + w.smooth + w.speed - 1) < 1e-12, ...
    'contextParams:weightsNotNormalised', 'DWA weights must sum to 1.');
end
