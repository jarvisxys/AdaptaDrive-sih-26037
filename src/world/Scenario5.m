function s = Scenario5(seed, density, cfg)
%SCENARIO5  Sudden Cattle Crossing.
%
%   7 m two-lane rural road, 300 m.  1-3 cows grazing at the roadside and one
%   oncoming car.  Ego starts at 40 km/h.
%
%   THE KEY EVENT, and why its timing is computed rather than scripted:
%   a cow steps into the carriageway when the ego is between 1.2 and 2.0
%   BRAKING DISTANCES away, with the factor drawn per seed.  Braking distance
%   is evaluated from the ego's speed AT THAT MOMENT, so the event scales with
%   whatever the configuration under test is actually doing.
%
%   That construction matters for the benchmark's honesty:
%     - at 1.2x the event is avoidable, but only by reacting promptly; there
%       is no margin for a late detection or a slow replan.
%     - at 2.0x a competent system should handle it comfortably.
%     - a configuration that crawls cannot dodge the test by arriving after
%       the cow has finished crossing, and one that speeds cannot outrun it.
%   A fixed trigger time would have rewarded exactly those two behaviours
%   instead of measuring avoidance.
%
%   The cow is deliberately NOT reactive to the ego (AgentModel): it is the
%   hazard, and an animal that yields would turn an avoidance test into a test
%   of the animal's caution.
%
%   See also SCENARIO1, BUILDSCENARIO, AGENTMODEL.

if nargin < 2 || isempty(density), density = 1.0; end
if nargin < 3 || isempty(cfg),     cfg = defaultConfig(); end

rs = RandStream('mrg32k3a', 'Seed', seed);

s = struct();
s.name    = 'cattle';
s.title   = 'Sudden Cattle Crossing';
s.context = 'rural';
s.seed    = seed;
s.density = density;
s.description = ['7 m two-lane rural road, 300 m, open sightlines, cattle ' ...
    'grazing at the roadside and oncoming traffic.'];
s.keyEvent = ['A cow enters the carriageway when the ego is 1.2-2.0 braking ' ...
    'distances away (factor drawn per seed).'];
s.successCriterion = ['Stop or steer clear without collision, then resume and ' ...
    'reach the far end.  Emergency-brake count and minimum TTC are reported.'];

% --- road ---------------------------------------------------------------
% Gentle curvature: sightlines are long, so the difficulty is the abruptness
% of the event rather than an occlusion.
rs.Substream = 1;
roadSpec = struct( ...
    'centers', [0 0; 80 6; 160 -4; 240 4; 300 0], ...
    'width',   7.0, ...
    'name',    'rural_road', ...
    'twoWay',  true, ...
    'directionDefined', true, ...
    'edgeNoise', 0.25);

s.road = RoadModel(roadSpec, 'res', cfg.risk.res, 'rs', rs);
L = s.road.segmentLength(1);
s.roadLength = L;

% --- ego route ----------------------------------------------------------
egoOffset = 1.75;               % keep-left on a 7 m road
startS = 5.0;
goalS  = L - 8.0;

startXY = s.road.pointAt(1, startS, egoOffset);
[~, T0] = s.road.pointAt(1, startS, egoOffset);
goalXY  = s.road.pointAt(1, goalS, egoOffset);

s.ego = struct( ...
    'x', startXY(1), 'y', startXY(2), ...
    'yaw', atan2(T0(2), T0(1)), ...
    'v', 40 / 3.6, ...
    'seg', 1, 'startS', startS, 'goalS', goalS, ...
    'laneOffset', egoOffset, ...
    'goal', goalXY, 'goalTol', 4.0);

sRef = (startS:0.5:goalS)';
s.refPath = s.road.pointAt(1, sRef, egoOffset);

ctx = contextParams(s.context, cfg);
s.nominalTime = (goalS - startS) / max(ctx.speedCap, 0.5);

% --- hazards ------------------------------------------------------------
% A rural road, not a broken village track: a couple of surface defects, no
% encroaching structures.  The test here is the animal, not the surface.
rs.Substream = 2;
s.hazards = HazardSet();
for k = 1:2
    sp = 60 + rand(rs) * (goalS - 120);
    dp = -2.5 + rand(rs) * 5.0;
    xy = s.road.pointAt(1, sp, dp);
    [~, T] = s.road.pointAt(1, sp, dp);
    s.hazards.addPothole(xy(1), xy(2), 0.35 + rand(rs)*0.4, 0.3 + rand(rs)*0.3, ...
        atan2(T(2), T(1)), 0.08);
end

% --- agents -------------------------------------------------------------
rs.Substream = 3;
agents = {};
nextId = 1;

% density == 0 means NO TRAFFIC AT ALL.  Used by the empty-road tests, which
% need a genuinely empty road: deleting agents from the struct afterwards does
% not remove them from the drivingScenario container, so the sensors go on
% detecting them frozen at their spawn positions and the "empty" road is not
% empty.  The only way to have no agents is to never create them.
if density <= 0
    s.agents = AgentModel.empty(1, 0);
    s.triggerFactor = NaN;
    return
end

% Cows at the roadside.  One of them is the trigger; the others graze.
nCows = max(1, min(3, round((1 + randi(rs, [0 2])) * density)));
triggerIdx = 1;

for k = 1:nCows
    % The trigger cow sits far enough along that the ego is at full speed,
    % and on the left verge so it crosses INTO the ego's half of the road.
    if k == triggerIdx
        sp = 120 + rand(rs) * 40;
        dp = 3.2;                         % just off the left edge
    else
        sp = 70 + rand(rs) * (goalS - 140);
        dp = (2*randi(rs, [0 1]) - 1) * (3.0 + rand(rs) * 0.8);
    end

    spec = struct('id', nextId, 'classId', 7, 'seg', 1, ...
        's', sp, 'd', dp, 'dir', 1, 'targetSpeed', 0);

    if k == triggerIdx
        % Factor drawn per seed: the difficulty varies across the ten
        % reported seeds instead of being pinned to one convenient value.
        factor = 1.2 + rand(rs) * 0.8;      % [1.2, 2.0]
        spec.trigger = struct('type', 'egoBrakingDistance', ...
            'factor', factor, 'decel', cfg.vehicle.comfortDecel, ...
            'minDistance', 12.0);   % keeps the event avoidable at any speed
        s.triggerFactor = factor;
    end

    agents{end+1} = AgentModel(spec, s.road, seed); %#ok<AGROW>
    nextId = nextId + 1;
end

% Oncoming car: removes "just swerve into the other lane" as a free answer.
nOnc = max(1, round(1 * density));
for k = 1:nOnc
    spec = struct('id', nextId, 'classId', 1, 'seg', 1, ...
        's', goalS - 30 - (k-1) * 60, 'd', -1.75, 'dir', -1, ...
        'targetSpeed', 9.0 + rand(rs) * 2.5);
    agents{end+1} = AgentModel(spec, s.road, seed); %#ok<AGROW>
    nextId = nextId + 1;
end

s.agents = [agents{:}];
end
