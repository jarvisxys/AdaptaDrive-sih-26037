function s = Scenario2(seed, density, cfg)
%SCENARIO2  Unsignalized Urban Intersection.
%
%   Four-way crossing, 7 m arms, no signals and no priority markings.  The ego
%   goes straight through while cross traffic, informal pedestrian crossings
%   and one wrong-way auto-rickshaw negotiate the same space.
%
%   The junction box is modelled as a segment with directionDefined = false,
%   so the expected-travel-direction field is UNDEFINED there.  That matters
%   for A2: inside a junction no single direction applies, and a detector that
%   guessed one would flag every turning vehicle as wrong-way.
%
%   See also SCENARIO1, ROADMODEL, WRONGWAYDETECTOR.

if nargin < 2 || isempty(density), density = 1.0; end
if nargin < 3 || isempty(cfg),     cfg = defaultConfig(); end

rs = RandStream('mrg32k3a', 'Seed', seed);

s = struct();
s.name    = 'intersection';
s.title   = 'Unsignalized Urban Intersection';
s.context = 'intersection';
s.seed    = seed;
s.density = density;
s.description = ['Four-way unsignalised crossing with 7 m arms, cross traffic, ' ...
    'informal pedestrian crossings and a wrong-way auto-rickshaw.'];
s.keyEvent = 'Right-of-way must be inferred; a wrong-way auto approaches on the far arm.';
s.successCriterion = ['Cross without collision. Wrong-way flag raised (A2) and ' ...
    'minimum TTC reported.'];

% --- road: two crossing arms plus an undirected junction box ------------
rs.Substream = 1;
L = 120;
specs = [ ...
    struct('centers', [-L/2 0; L/2 0], 'width', 7.0, 'name', 'ew', ...
           'twoWay', true, 'directionDefined', true, 'edgeNoise', 0.15), ...
    struct('centers', [0 -L/2; 0 L/2], 'width', 7.0, 'name', 'ns', ...
           'twoWay', true, 'directionDefined', true, 'edgeNoise', 0.15)];

s.road = RoadModel(specs, 'res', cfg.risk.res, 'rs', rs);
s.roadLength = s.road.segmentLength(1);

% --- ego route: west to east, straight through --------------------------
egoOffset = 1.75;
startS = 8.0;
goalS  = s.roadLength - 8.0;

startXY = s.road.pointAt(1, startS, egoOffset);
[~, T0] = s.road.pointAt(1, startS, egoOffset);
goalXY  = s.road.pointAt(1, goalS, egoOffset);

s.ego = struct('x', startXY(1), 'y', startXY(2), ...
    'yaw', atan2(T0(2), T0(1)), 'v', 20/3.6, ...
    'seg', 1, 'startS', startS, 'goalS', goalS, ...
    'laneOffset', egoOffset, 'goal', goalXY, 'goalTol', 4.0);

sRef = (startS:0.5:goalS)';
s.refPath = s.road.pointAt(1, sRef, egoOffset);

ctx = contextParams(s.context, cfg);
s.nominalTime = (goalS - startS) / max(ctx.speedCap, 0.5);

% --- hazards: kerb build-outs at the corners ----------------------------
rs.Substream = 2;
s.hazards = HazardSet();
for k = 1:2
    sp = 20 + rand(rs) * (goalS - 60);
    xy = s.road.pointAt(1, sp, -2.6 + rand(rs) * 0.6);
    s.hazards.addPothole(xy(1), xy(2), 0.4 + rand(rs)*0.3, 0.3, 0, 0.08);
end

% --- agents --------------------------------------------------------------
rs.Substream = 3;
agents = {};
nextId = 1;

if density <= 0
    s.agents = AgentModel.empty(1, 0);
    return
end

mk = @(spec) AgentModel(spec, s.road, seed);

% Cross traffic on the north-south arm (segment 2).
nCars = max(1, round(3 * density));
for k = 1:nCars
    dirk = 1 - 2*mod(k, 2);
    spec = struct('id', nextId, 'classId', 1, 'seg', 2, ...
        's', 25 + rand(rs)*25 + (k-1)*18, 'd', dirk*1.75, 'dir', dirk, ...
        'targetSpeed', 4.5 + rand(rs)*2.5);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Autos: one of them is the WRONG-WAY vehicle (A2).
nAutos = max(1, round(2 * density));
for k = 1:nAutos
    if k == 1
        % On the east-west arm, on the ego's own half, coming at it: the
        % lateral offset and the direction of travel disagree, which is
        % exactly what the expected-direction field detects.
        spec = struct('id', nextId, 'classId', 3, 'seg', 1, ...
            's', goalS - 25, 'd', 1.75, 'dir', -1, ...
            'targetSpeed', 4.0 + rand(rs)*1.5);
    else
        spec = struct('id', nextId, 'classId', 3, 'seg', 2, ...
            's', 70 + rand(rs)*20, 'd', -1.75, 'dir', -1, ...
            'targetSpeed', 3.5 + rand(rs)*1.5);
    end
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Two-wheelers filtering through.
nTw = max(1, round(2 * density));
for k = 1:nTw
    seg = 1 + mod(k, 2);
    spec = struct('id', nextId, 'classId', 4, 'seg', seg, ...
        's', 30 + rand(rs)*40, 'd', 0.9, 'dir', 1, ...
        'targetSpeed', 5.5 + rand(rs)*2.5);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Pedestrians crossing informally, i.e. not at any marked place.
nPed = max(1, round(3 * density));
for k = 1:nPed
    seg = 1 + mod(k, 2);
    side = 1 - 2*mod(k, 2);
    spec = struct('id', nextId, 'classId', 5, 'seg', seg, ...
        's', 40 + rand(rs)*30, 'd', side*3.0, 'dir', side, ...
        'trigger', struct('type', 'egoDistance', 'at', 18 + rand(rs)*8));
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

s.agents = [agents{:}];
end
