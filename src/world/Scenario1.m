function s = Scenario1(seed, density, cfg)
%SCENARIO1  Unmarked Village Road.
%
%   6 m wide, curved, 250 m, no lane markings, uneven edges, 3-5 potholes.
%   Agents at density 1: 2 pedestrians, 1 two-wheeler, 1 pushcart, 1 oncoming
%   car.  Ego starts at 25 km/h and must reach the far end.
%
%   Key event: a pedestrian steps off the edge and crosses in front of the
%   ego.  The crossing is triggered by EGO PROXIMITY rather than by a fixed
%   clock, so the event stays meaningful whatever speed the configuration
%   under test chooses to drive at - a slow baseline cannot dodge the test by
%   arriving after the pedestrian has finished crossing.
%
%   What makes it hard: no lane structure to follow, an oncoming vehicle
%   sharing 6 m of road, surface hazards that must be avoided without
%   treating them as walls, and edges that are not straight lines.
%
%   See also BUILDSCENARIO, ROADMODEL, AGENTMODEL.

if nargin < 2 || isempty(density), density = 1.0; end
if nargin < 3 || isempty(cfg),     cfg = defaultConfig(); end

rs = RandStream('mrg32k3a', 'Seed', seed);

s = struct();
s.name    = 'village';
s.title   = 'Unmarked Village Road';
s.context = 'village';
s.seed    = seed;
s.density = density;
s.description = ['6 m unmarked village road, 250 m, uneven edges and surface ' ...
    'hazards, shared with pedestrians, a pushcart and oncoming traffic.'];
s.keyEvent = 'Pedestrian crosses from the edge as the ego approaches.';
s.successCriterion = ['Reach the far end with no collision, no pothole ' ...
    'traversal, and at least 0.5 m clearance from hazards (A1).'];

% --- road ---------------------------------------------------------------
% A curved centreline: the planner must handle a road that bends, and the
% wrong-way detector needs an expected direction that is not constant.
rs.Substream = 1;
roadSpec = struct( ...
    'centers', [0 0; 60 12; 120 -6; 185 14; 250 4], ...
    'width',   6.0, ...
    'name',    'village_road', ...
    'twoWay',  true, ...
    'directionDefined', true, ...
    'edgeNoise', 0.35);          % uneven edges, seeded

s.road = RoadModel(roadSpec, 'res', cfg.risk.res, 'rs', rs);
L = s.road.segmentLength(1);
s.roadLength = L;

% --- ego route ----------------------------------------------------------
% Keep-left convention: the ego travels on the +d half of the road.
egoOffset = 1.5;
startS = 5.0;
goalS  = L - 8.0;

startXY = s.road.pointAt(1, startS, egoOffset);
[~, T0]  = s.road.pointAt(1, startS, egoOffset);
goalXY  = s.road.pointAt(1, goalS, egoOffset);

s.ego = struct( ...
    'x',   startXY(1), ...
    'y',   startXY(2), ...
    'yaw', atan2(T0(2), T0(1)), ...
    'v',   25 / 3.6, ...
    'seg', 1, ...
    'startS', startS, ...
    'goalS', goalS, ...
    'laneOffset', egoOffset, ...
    'goal', goalXY, ...
    'goalTol', 4.0);

% Reference centreline path at the keep-left offset.  Used by the BL1
% lane-follow baseline and by the M1 'scripted' scaffold.  The PROPOSED
% planner does NOT receive it.
sRef = (startS:0.5:goalS)';
s.refPath = s.road.pointAt(1, sRef, egoOffset);

ctx = contextParams(s.context, cfg);
s.nominalTime = (goalS - startS) / max(ctx.speedCap, 0.5);

% --- hazards ------------------------------------------------------------
rs.Substream = 2;
s.hazards = HazardSet();

nPot = 3 + randi(rs, [0 2]);          % 3 to 5 potholes
placedS = [];
for k = 1:nPot
    % Keep potholes clear of the ego's first 25 m so every seed starts the
    % same way, and spread them out so they are separate obstacles.
    for attempt = 1:50
        sp = 25 + rand(rs) * (goalS - 40);
        if isempty(placedS) || min(abs(placedS - sp)) > 12
            break
        end
    end
    placedS(end+1) = sp; %#ok<AGROW>
    dp = -2.2 + rand(rs) * 4.4;
    xy = s.road.pointAt(1, sp, dp);
    [~, T] = s.road.pointAt(1, sp, dp);
    a = 0.3 + rand(rs) * 0.45;        % semi-axes -> 0.6 to 1.5 m across
    b = 0.3 + rand(rs) * 0.35;
    s.hazards.addPothole(xy(1), xy(2), a, b, atan2(T(2), T(1)), 0.08 + 0.1*rand(rs));
end

% Broken carriageway at one edge: a patch that is not drivable at all.
sb = 0.45 * L;
edgePoly = buildEdgeBreak(s.road, sb, 8.0, 1.1);
s.hazards.addEdgeBreak(edgePoly);

% --- agents -------------------------------------------------------------
rs.Substream = 3;
agents = {};
nextId = 1;

% density == 0 means NO TRAFFIC AT ALL - see Scenario5 for why this has to be
% handled at creation time rather than by deleting agents afterwards.
if density <= 0
    s.agents = AgentModel.empty(1, 0);
    return
end

nPed  = max(1, round(2 * density));
nTw   = max(1, round(1 * density));
nCart = max(1, round(1 * density));
nOnc  = max(1, round(1 * density));

% Pedestrians: walking the edge, one of them will cross.
for k = 1:nPed
    side = 1 - 2 * mod(k, 2);                  % alternate sides
    sp = 55 + (k - 1) * 45 + rand(rs) * 20;
    sp = min(sp, goalS - 15);
    spec = struct('id', nextId, 'classId', 5, 'seg', 1, ...
        's', sp, 'd', side * 2.5, 'dir', side);
    if k == 1
        % The key event.  Fires when the ego is 22 m away, which at the
        % 25 km/h context cap is about 3 s of warning: enough to react, not
        % enough to ignore.
        spec.trigger = struct('type', 'egoDistance', 'at', 22.0);
    end
    agents{end+1} = AgentModel(spec, s.road, seed); %#ok<AGROW>
    nextId = nextId + 1;
end

% Two-wheeler ahead, same direction, filtering along the left.
for k = 1:nTw
    spec = struct('id', nextId, 'classId', 4, 'seg', 1, ...
        's', 40 + (k-1) * 30 + rand(rs) * 10, 'd', 0.8, 'dir', 1, ...
        'targetSpeed', 6.5 + rand(rs) * 2.0);
    agents{end+1} = AgentModel(spec, s.road, seed); %#ok<AGROW>
    nextId = nextId + 1;
end

% Pushcart occupying the left edge: slow, and effectively an obstacle.
for k = 1:nCart
    spec = struct('id', nextId, 'classId', 6, 'seg', 1, ...
        's', 90 + (k-1) * 35 + rand(rs) * 15, 'd', 2.1, 'dir', 1, ...
        'targetSpeed', 0.6 + rand(rs) * 0.5);
    agents{end+1} = AgentModel(spec, s.road, seed); %#ok<AGROW>
    nextId = nextId + 1;
end

% Oncoming car on the other half of the road.
for k = 1:nOnc
    spec = struct('id', nextId, 'classId', 1, 'seg', 1, ...
        's', goalS - 20 - (k-1) * 40, 'd', -1.5, 'dir', -1, ...
        'targetSpeed', 6.0 + rand(rs) * 2.5);
    agents{end+1} = AgentModel(spec, s.road, seed); %#ok<AGROW>
    nextId = nextId + 1;
end

s.agents = [agents{:}];

end

% ========================================================================
function poly = buildEdgeBreak(road, sCentre, lengthM, widthM)
%BUILDEDGEBREAK  Polygon of broken carriageway along the right edge.
sv = linspace(sCentre - lengthM/2, sCentre + lengthM/2, 12)';
outer = road.pointAt(1, sv, -3.0);
inner = road.pointAt(1, flipud(sv), -3.0 + widthM);
poly = [outer; inner];
end
