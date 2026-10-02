function s = ScenarioUnseen(seed, density, cfg)
%SCENARIOUNSEEN  Market plus a wrong-way bus and a crossing cow (B5).
%
%   The held-out variant. Nothing was tuned on it: it is built from Scenario4
%   with two agents added, and it is the only scenario that combines a
%   large wrong-way vehicle with a crossing animal in a confined corridor.
%
%   Why these two additions specifically
%     wrong-way bus    The market already contains a wrong-way TWO-WHEELER,
%                      which the wrong-way detector (A2) sees easily and which
%                      the planner can pass. A bus is 10.5 m long and 2.6 m
%                      wide on a corridor of about 5.5 m, so it cannot be
%                      passed at all - the correct response is to stop and let
%                      it through, which is a different behaviour from the one
%                      the market otherwise exercises.
%     crossing cow     Class 7 carries the same risk weight as a pedestrian
%                      (1.00) but is 2.0 m long, does not yield, and in the
%                      cattle scenario is the object the ego brakes hardest
%                      for. Putting it in the market tests both class priors
%                      (B3) and the risk field (B1) in the one place where
%                      there is no room to go round.
%
%   This scenario is REPORTED SEPARATELY and is never pooled with the five.
%   Pooling a held-out case into an aggregate is how a held-out case stops
%   being held out.
%
%   See also SCENARIO4, CONFIGPRESET, RUN_EXPERIMENTS.

if nargin < 1 || isempty(seed),    seed = 1;      end
if nargin < 2 || isempty(density), density = 1.0; end
if nargin < 3 || isempty(cfg),     cfg = defaultConfig(); end

s = Scenario4(seed, density, cfg);

s.name  = 'unseen';
s.title = 'Held-out: Market + Bus + Cow';
s.description = ['The dense market, with a wrong-way bus filling the corridor ' ...
    'and a cow crossing it. Held out: nothing was tuned on this variant.'];
s.keyEvent = ['A 10.5 m bus comes the wrong way up a 5.5 m street while a cow ' ...
    'crosses it. Neither can be driven around.'];
s.successCriterion = ['Stop for both and traverse without collision. ' ...
    'Reported separately from the five tuned scenarios.'];

if density <= 0
    return      % the empty-road form stays empty, as elsewhere
end

% A separate substream, so adding these two agents cannot shift the random
% draws of the market agents Scenario4 already placed. Without this the
% "variant" would differ from the market in every agent's position too, and any
% difference in the result would be unattributable.
rs = RandStream('mrg32k3a', 'Seed', seed);
rs.Substream = 41;

if isempty(s.agents)
    nextId = 1;
else
    nextId = max([s.agents.id]) + 1;
end

goalS = s.road.segmentLength(1) - 8;
mk = @(spec) AgentModel(spec, s.road, seed);
extra = {};

% Wrong-way bus, entering from the far end and coming down the street. Placed
% beyond half way so the ego has committed to the corridor before meeting it.
extra{end+1} = mk(struct('id', nextId, 'classId', 2, 'seg', 1, ...
    's', goalS - 12 - rand(rs)*8, 'd', 1.0, 'dir', -1, ...
    'targetSpeed', 2.0 + rand(rs)*1.0));
nextId = nextId + 1;

% Cow crossing, in the middle third. A cow starts in 'graze' and enters the road
% on a TRIGGER - `mode` in the spec is ignored, because the mode is a property of
% the class. The trigger is the same braking-distance form Scenario5 uses, so the
% crossing stays avoidable whatever speed the ego happens to be doing, and a
% configuration cannot pass by crawling until the cow has finished.
cowSpec = struct('id', nextId, 'classId', 7, 'seg', 1, ...
    's', 55 + rand(rs)*25, 'd', 2.6, 'dir', 1, 'targetSpeed', 0);
cowSpec.trigger = struct('type', 'egoBrakingDistance', ...
    'factor', 1.3 + rand(rs)*0.5, 'decel', cfg.vehicle.comfortDecel, ...
    'minDistance', 10.0);
extra{end+1} = mk(cowSpec);

s.agents = [s.agents, extra{:}];
end
