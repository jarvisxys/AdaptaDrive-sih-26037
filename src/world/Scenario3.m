function s = Scenario3(seed, density, cfg)
%SCENARIO3  Highway Slow-Merge.
%
%   400 m of dual carriageway with an on-ramp.  A truck holds 25 km/h in a
%   60 km/h stream and a tractor joins from the ramp without signalling.
%
%   This is the scenario requirement A3 is measured on: the TTC at the moment
%   YIELD is triggered, and the minimum clearance to the merging vehicle.
%   The speed differential is the point - a 60 km/h ego closing on a 25 km/h
%   truck covers the gap in seconds.
%
%   See also SCENARIO1, MERGEDETECTOR.

if nargin < 2 || isempty(density), density = 1.0; end
if nargin < 3 || isempty(cfg),     cfg = defaultConfig(); end

rs = RandStream('mrg32k3a', 'Seed', seed);

s = struct();
s.name    = 'highway';
s.title   = 'Highway Slow-Merge';
s.context = 'highway';
s.seed    = seed;
s.density = density;
s.description = ['400 m dual carriageway with an on-ramp; a slow truck in the ' ...
    'stream and a tractor merging without signalling.'];
s.keyEvent = 'A tractor joins from the ramp into the ego''s lane without signalling.';
s.successCriterion = ['Pass or yield without collision. TTC at the YIELD trigger ' ...
    'and minimum clearance to the merging vehicle are reported (A3).'];

% --- road: a gently curving carriageway plus a converging ramp ----------
rs.Substream = 1;
main = struct('centers', [0 0; 120 6; 260 2; 400 8], 'width', 7.5, ...
    'name', 'carriageway', 'twoWay', false, 'directionDefined', true, ...
    'edgeNoise', 0.12);

% The ramp converges on the main road and ends inside it.  RoadModel clips
% each ribbon at its own end caps, so the ramp does not punch a hole in the
% carriageway where it terminates.
ramp = struct('centers', [60 -22; 120 -12; 175 -3.6], 'width', 5.0, ...
    'name', 'onramp', 'twoWay', false, 'directionDefined', true, ...
    'edgeNoise', 0.10);

s.road = RoadModel([main, ramp], 'res', cfg.risk.res, 'rs', rs);
s.roadLength = s.road.segmentLength(1);

% --- ego route -----------------------------------------------------------
egoOffset = 1.9;
startS = 6.0;
goalS  = s.roadLength - 8.0;

startXY = s.road.pointAt(1, startS, egoOffset);
[~, T0] = s.road.pointAt(1, startS, egoOffset);
goalXY  = s.road.pointAt(1, goalS, egoOffset);

s.ego = struct('x', startXY(1), 'y', startXY(2), ...
    'yaw', atan2(T0(2), T0(1)), 'v', 60/3.6, ...
    'seg', 1, 'startS', startS, 'goalS', goalS, ...
    'laneOffset', egoOffset, 'goal', goalXY, 'goalTol', 5.0);

sRef = (startS:0.5:goalS)';
s.refPath = s.road.pointAt(1, sRef, egoOffset);

ctx = contextParams(s.context, cfg);
s.nominalTime = (goalS - startS) / max(ctx.speedCap, 0.5);

% --- hazards: a highway is smooth; debris on the shoulder ---------------
rs.Substream = 2;
s.hazards = HazardSet();
xy = s.road.pointAt(1, 0.55 * goalS, -3.2);
s.hazards.addPothole(xy(1), xy(2), 0.5, 0.35, 0, 0.06);

% --- agents --------------------------------------------------------------
rs.Substream = 3;
agents = {};
nextId = 1;

if density <= 0
    s.agents = AgentModel.empty(1, 0);
    return
end

mk = @(spec) AgentModel(spec, s.road, seed);

% The slow truck, modelled as the bus class: same size and predictability.
spec = struct('id', nextId, 'classId', 2, 'seg', 1, ...
    's', 90 + rand(rs)*30, 'd', 1.9, 'dir', 1, 'targetSpeed', 25/3.6);
agents{end+1} = mk(spec); nextId = nextId + 1;

% Faster cars in the stream.
nCars = max(1, round(3 * density));
for k = 1:nCars
    lane = 1.9 - 3.4 * mod(k, 2);          % inner or outer
    spec = struct('id', nextId, 'classId', 1, 'seg', 1, ...
        's', 40 + (k-1)*55 + rand(rs)*20, 'd', lane, 'dir', 1, ...
        'targetSpeed', 13 + rand(rs)*4);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% The tractor on the ramp: slow, and it merges without signalling.  Modelled
% as the auto class, which carries the informal-merge behaviour.
nMerge = max(1, round(1 * density));
for k = 1:nMerge
    spec = struct('id', nextId, 'classId', 3, 'seg', 2, ...
        's', 10 + (k-1)*20 + rand(rs)*10, 'd', 0, 'dir', 1, ...
        'targetSpeed', 7.0 + rand(rs)*2.0);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

s.agents = [agents{:}];
end
