function s = Scenario4(seed, density, cfg)
%SCENARIO4  Dense Market.
%
%   150 m of street narrowed to about 5 m of usable width by stalls, with
%   pedestrians, pushcarts, autos, two-wheelers, parked cars and one wrong-way
%   two-wheeler.  Ego crawls at 10 km/h.
%
%   This is the heaviest replanning load of the five: the market context uses
%   the shortest replanning period (0.25 s) and the tightest clearances,
%   because the clutter changes faster than anywhere else.  It is the
%   scenario the latency target is hardest to meet on, which is why the
%   profiling target is stated against this one.
%
%   See also SCENARIO1, CONTEXTPARAMS.

if nargin < 2 || isempty(density), density = 1.0; end
if nargin < 3 || isempty(cfg),     cfg = defaultConfig(); end

rs = RandStream('mrg32k3a', 'Seed', seed);

s = struct();
s.name    = 'market';
s.title   = 'Dense Market';
s.context = 'market';
s.seed    = seed;
s.density = density;
s.description = ['150 m market street narrowed to roughly 5 m by encroaching ' ...
    'stalls, crowded with pedestrians, carts, autos and parked vehicles.'];
s.keyEvent = 'Continuous clutter; a wrong-way two-wheeler filters through against the flow.';
s.successCriterion = ['Traverse without collision under the heaviest replanning ' ...
    'load of the five scenarios.'];

% --- road ----------------------------------------------------------------
rs.Substream = 1;
% 8 m of carriageway, narrowed by the stalls below to roughly 5.5 m of usable
% width.  The spec asks for "5 m effective width, stalls encroaching": the
% encroachment has to come from the stalls, so the underlying road is wider
% than the space the vehicle actually gets.
roadSpec = struct('centers', [0 0; 50 4; 100 -3; 150 2], 'width', 8.0, ...
    'name', 'market_street', 'twoWay', true, 'directionDefined', true, ...
    'edgeNoise', 0.30);

s.road = RoadModel(roadSpec, 'res', cfg.risk.res, 'rs', rs);
L = s.road.segmentLength(1);
s.roadLength = L;

% --- ego route -----------------------------------------------------------
% Keep-left inside the corridor the stalls leave: the ego body spans
% 1.3 +- 0.9 m, i.e. out to 2.2 m, clear of the stall inner edge at ~2.8 m.
egoOffset = 1.3;
startS = 5.0;
goalS  = L - 6.0;

startXY = s.road.pointAt(1, startS, egoOffset);
[~, T0] = s.road.pointAt(1, startS, egoOffset);
goalXY  = s.road.pointAt(1, goalS, egoOffset);

s.ego = struct('x', startXY(1), 'y', startXY(2), ...
    'yaw', atan2(T0(2), T0(1)), 'v', 10/3.6, ...
    'seg', 1, 'startS', startS, 'goalS', goalS, ...
    'laneOffset', egoOffset, 'goal', goalXY, 'goalTol', 3.5);

sRef = (startS:0.5:goalS)';
s.refPath = s.road.pointAt(1, sRef, egoOffset);

ctx = contextParams(s.context, cfg);
s.nominalTime = (goalS - startS) / max(ctx.speedCap, 0.5);

% --- hazards: the stalls that make it a market --------------------------
% Alternating encroachment narrows 7 m of carriageway to roughly 5 m of
% usable width without ever fully blocking it.
rs.Substream = 2;
s.hazards = HazardSet();

nStalls = max(2, round(8 * density));
for k = 1:nStalls
    side = 1 - 2*mod(k, 2);
    sp = 12 + (k-1) * (goalS - 20) / max(nStalls, 1) + rand(rs)*3;
    % Centred near the kerb so the stall's INNER edge lands about 2.8 m from
    % the centreline, leaving the ~5.5 m corridor the scenario is about.
    dp = side * (3.45 + rand(rs)*0.15);
    xy = s.road.pointAt(1, sp, dp);
    [~, T] = s.road.pointAt(1, sp, dp);
    s.hazards.addStatic(xy(1), xy(2), atan2(T(2), T(1)), ...
        2.0 + rand(rs)*1.0, 1.2 + rand(rs)*0.3, 'stall');
end

% A few potholes: a market street is not resurfaced often.
for k = 1:3
    sp = 20 + rand(rs) * (goalS - 40);
    dp = -1.8 + rand(rs)*3.6;
    xy = s.road.pointAt(1, sp, dp);
    s.hazards.addPothole(xy(1), xy(2), 0.35 + rand(rs)*0.3, 0.3, 0, 0.09);
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

% Parked cars: stationary obstacles that are still AGENTS, so perception has
% to detect and classify them rather than being handed them as map furniture.
nParked = max(1, round(2 * density));
for k = 1:nParked
    sp = 30 + (k-1)*55 + rand(rs)*10;
    spec = struct('id', nextId, 'classId', 1, 'seg', 1, ...
        's', sp, 'd', -2.2, 'dir', 1, 'targetSpeed', 0);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Pedestrians: the dominant class here, some of them crossing.
nPed = max(2, round(6 * density));
for k = 1:nPed
    side = 1 - 2*mod(k, 2);
    spec = struct('id', nextId, 'classId', 5, 'seg', 1, ...
        's', 15 + (k-1) * (goalS - 25) / max(nPed, 1) + rand(rs)*4, ...
        'd', side * (2.3 + rand(rs)*0.5), 'dir', side);
    % Every third pedestrian crosses, not every second.  On a 5.5 m corridor
    % three simultaneous crossings leave the ego nowhere to be: it stops
    % correctly for the first and is then walked into by the next. Most people
    % on a market street walk ALONG it; the crossings are the exception, and
    % modelling them as the majority made the scenario unpassable rather than
    % difficult.
    if mod(k, 3) == 1
        spec.trigger = struct('type', 'egoDistance', 'at', 10 + rand(rs)*5);
    end
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Pushcarts occupying the edge at walking pace.
nCart = max(1, round(2 * density));
for k = 1:nCart
    spec = struct('id', nextId, 'classId', 6, 'seg', 1, ...
        's', 40 + (k-1)*45 + rand(rs)*10, 'd', 1.9, 'dir', 1, ...
        'targetSpeed', 0.6 + rand(rs)*0.4);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Autos threading through.
nAuto = max(1, round(2 * density));
for k = 1:nAuto
    dirk = 1 - 2*mod(k, 2);
    spec = struct('id', nextId, 'classId', 3, 'seg', 1, ...
        's', 25 + (k-1)*50 + rand(rs)*12, 'd', dirk*1.4, 'dir', dirk, ...
        'targetSpeed', 2.5 + rand(rs)*1.5);
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

% Two-wheelers, one of them filtering the WRONG WAY against the flow (A2).
nTw = max(1, round(2 * density));
for k = 1:nTw
    if k == 1
        spec = struct('id', nextId, 'classId', 4, 'seg', 1, ...
            's', goalS - 20, 'd', 1.2, 'dir', -1, ...
            'targetSpeed', 3.5 + rand(rs)*1.5);
    else
        spec = struct('id', nextId, 'classId', 4, 'seg', 1, ...
            's', 35 + rand(rs)*40, 'd', 0.7, 'dir', 1, ...
            'targetSpeed', 3.0 + rand(rs)*1.5);
    end
    agents{end+1} = mk(spec); nextId = nextId + 1; %#ok<AGROW>
end

s.agents = [agents{:}];
end
