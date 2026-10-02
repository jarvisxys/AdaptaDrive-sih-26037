function s = buildScenario(name, seed, density, cfg)
%BUILDSCENARIO  Construct one of the five scenarios by name.
%
%   S = BUILDSCENARIO(name, seed, density, cfg)
%
%   Names: village | intersection | highway | market | cattle | unseen
%
%   'unseen' is the B5 held-out variant (market + wrong-way bus + crossing cow).
%   It is not one of the five and is reported on its own.
%
%   Scenarios that are not built yet raise an error naming the milestone that
%   will build them.  They do NOT silently fall back to another scenario,
%   which would put a real number in the results table under the wrong label.
%
%   See also SCENARIO1, RUN_DEMO.

if nargin < 2 || isempty(seed),    seed = 1;      end
if nargin < 3 || isempty(density), density = 1.0; end
if nargin < 4 || isempty(cfg),     cfg = defaultConfig(); end

name = lower(char(name));

switch name
    case {'village', 'scenario1', '1'}
        s = Scenario1(seed, density, cfg);

    case {'intersection', 'scenario2', '2'}
        s = Scenario2(seed, density, cfg);

    case {'highway', 'scenario3', '3'}
        s = Scenario3(seed, density, cfg);

    case {'market', 'scenario4', '4'}
        s = Scenario4(seed, density, cfg);

    case {'cattle', 'scenario5', '5'}
        s = Scenario5(seed, density, cfg);

    % B5 held-out variant. Reported separately, never pooled with the five:
    % pooling a held-out case into an aggregate is how it stops being held out.
    case {'unseen', 'market-unseen'}
        s = ScenarioUnseen(seed, density, cfg);

    otherwise
        error('buildScenario:unknownScenario', ...
            ['Unknown scenario "%s". Known: village, intersection, highway, ' ...
             'market, cattle, unseen.'], name);
end

s.cfg = cfg;
s = attachDrivingScenario(s, cfg);
end

% ========================================================================
function s = attachDrivingScenario(s, cfg)
%ATTACHDRIVINGSCENARIO  Mirror the world into a drivingScenario container.
%
%   The container exists so the Automated Driving Toolbox sensor models can be
%   driven from it at M2 (targetPoses needs actors in a scenario).  It is a
%   MIRROR, not the source of truth: AgentModel owns the motion and writes
%   poses in, which is what lets agents be reactive and seed-randomised.
%
%   Unit boundary: drivingScenario Actor.Yaw is in DEGREES while the whole of
%   AdaptaDrive works in radians (verified on R2026a, see DEVIATIONS.md D10).
%   The conversion happens here and nowhere else.

s.scenarioObj = [];
s.egoActor = [];
s.agentActors = [];

if ~strcmp(cfg.env.backends.scenario, 'drivingScenario')
    return
end

sc = drivingScenario('SampleTime', cfg.sim.dt);

% Road geometry, for fidelity of the container and for any toolbox utility
% that expects a road.  Our RoadModel remains authoritative for drivability.
try
    centres = s.road.segments(1).centerline;
    step = max(1, round(2.0 / (centres(2,1) - centres(1,1) + eps)));
    step = max(1, min(step, size(centres, 1) - 1));
    pts = centres(1:step:end, :);
    road(sc, [pts, zeros(size(pts, 1), 1)], 6.0);
catch ME
    % A container road is cosmetic; losing it must not lose the run.  Say so
    % rather than pretending it worked.
    warning('AdaptaDrive:roadContainer', ...
        'drivingScenario road() failed (%s); container has no road geometry.', ME.message);
end

ego = vehicle(sc, 'ClassID', 1, ...
    'Length', cfg.vehicle.length, 'Width', cfg.vehicle.width, 'Height', 1.5, ...
    'Position', [s.ego.x, s.ego.y, 0], 'Yaw', rad2deg(s.ego.yaw));

% Container actors.  The ClassID handed to drivingScenario is a PROXY (see
% adtContainerClass): the toolbox only accepts its own four actor classes and
% two vehicle classes.  Real dimensions are always passed explicitly, so the
% proxy never changes what a sensor sees, and AdaptaDrive's own classId stays
% authoritative everywhere a decision is made.
acts = cell(1, numel(s.agents));
containerMap = zeros(numel(s.agents), 2);   % [ourClassId, containerClassId]
for k = 1:numel(s.agents)
    a = s.agents(k);
    m = adtContainerClass(a.classId);
    args = {'ClassID', m.classId, ...
        'Length', a.L, 'Width', a.W, 'Height', a.H, ...
        'Position', [a.x, a.y, 0], 'Yaw', rad2deg(a.yaw)};
    if strcmp(m.kind, 'vehicle')
        acts{k} = vehicle(sc, args{:});
    else
        acts{k} = actor(sc, args{:});
    end
    containerMap(k, :) = [a.classId, m.classId];
end
s.containerClassMap = containerMap;

s.scenarioObj = sc;
s.egoActor = ego;
if isempty(acts)
    s.agentActors = [];
    s.actorIdToAgentIdx = containers.Map('KeyType', 'double', 'ValueType', 'double');
else
    s.agentActors = [acts{:}];
    % Sensor detections identify their source by ActorID (ObjectAttributes
    % .TargetIndex).  Store the mapping explicitly rather than assuming
    % ActorID == index + 1, which would break the moment the ego stops being
    % actor 1 or an actor is added mid-scenario.
    ids = arrayfun(@(a) a.ActorID, s.agentActors);
    s.actorIdToAgentIdx = containers.Map(num2cell(double(ids)), num2cell(1:numel(ids)));
end
s.egoActorId = ego.ActorID;

% OriginOffset is the vector from an actor's GEOMETRIC CENTRE to its
% ROTATIONAL CENTRE, and drivingScenario interprets Position as the latter.
% For anything built with vehicle() that offset is non-zero (the origin sits
% at the rear axle), so writing a body-centre position straight into Position
% displaces the body by over a metre and the sensors then ray-trace a vehicle
% that is not where our world says it is.  Cache the offsets and apply them on
% every sync.  Measured on R2026a: Car L=4.2 -> [-1.1 0 0]; pedestrian and
% bicycle actors -> [0 0 0].  See DEVIATIONS D19.
prof = actorProfiles(sc);
profIds = [prof.ActorID];
s.actorOriginOffset = zeros(numel(s.agents), 1);
for k = 1:numel(s.agents)
    if isempty(s.agentActors) || k > numel(s.agentActors)
        continue
    end
    j = find(profIds == s.agentActors(k).ActorID, 1);
    if ~isempty(j) && isfield(prof, 'OriginOffset')
        s.actorOriginOffset(k) = prof(j).OriginOffset(1);
    end
end
jEgo = find(profIds == ego.ActorID, 1);
s.egoOriginOffset = 0;
if ~isempty(jEgo) && isfield(prof, 'OriginOffset')
    s.egoOriginOffset = prof(jEgo).OriginOffset(1);
end
end
