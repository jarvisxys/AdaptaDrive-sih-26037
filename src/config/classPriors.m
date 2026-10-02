function out = classPriors(classId)
%CLASSPRIORS  The single class table for AdaptaDrive (B2/B3).
%
%   T = CLASSPRIORS()          returns the full table as a struct array,
%                              ordered so that T(k).id == k for k = 1..7.
%   P = CLASSPRIORS(classId)   returns one entry.  classId 0 ("unknown",
%                              a track the camera never classified) returns
%                              the deliberately most conservative prototype.
%
%   Fixed class IDs (never renumber - they are written into every log):
%       1 car   2 bus   3 auto-rickshaw   4 two-wheeler
%       5 pedestrian    6 pushcart        7 cow
%
%   ------------------------------------------------------------------
%   riskWeight (w_c in the risk-map equation, Section 5.5)
%   ------------------------------------------------------------------
%   w_c encodes VULNERABILITY AND UNPREDICTABILITY, not mass.  It answers
%   "how much room should the planner give this agent's uncertainty?"  A bus
%   is heavy but travels predictably along a fixed line, so it needs less
%   margin than a pedestrian who may reverse direction in one step.
%
%     1.00 pedestrian  - can stop, turn or reverse within one timestep;
%                        no protection at all in a collision.
%     1.00 cow         - equally unprotected, and its motion carries no
%                        intent cue a planner can read; stationary for
%                        minutes, then a sudden metre of movement.
%     0.85 two-wheeler - high lateral freedom, filters through gaps, rider
%                        is unprotected; still broadly follows traffic flow.
%     0.75 pushcart    - human-drawn and unprotected, but slow and
%                        effectively constrained to the road edge.
%     0.65 auto        - three wheels, sudden stops and informal merges,
%                        occupant partially protected.
%     0.55 car         - predictable envelope, protected occupants.
%     0.50 bus         - the most predictable trajectory on the road and the
%                        least vulnerable; large, so its FOOTPRINT already
%                        dominates the risk integral without extra weight.
%     1.00 unknown     - never guess in our own favour: an unclassified
%                        track is treated as the most demanding class.
%
%   ------------------------------------------------------------------
%   Prediction priors (Section 5.6) - consumed by Predictor.m
%   ------------------------------------------------------------------
%   motionModel   which predictor branch runs
%   qLon, qLat    process-noise growth, m/s^2 units: the predicted
%                 covariance grows as Sigma(t) = P_track + Q_class * t, with
%                 Q_class = diag(qLon, qLat) rotated into the heading frame.
%                 Lateral growth is the discriminating term: a two-wheeler
%                 may be a metre sideways in a second, a bus may not.
%
%   These are DESIGN PRIORS, not measurements, and they are tuned only on
%   seeds 101-110 (freeze rule, see README).
%
%   See also CONTEXTPARAMS, DEFAULTCONFIG, RISKMAP, PREDICTOR.

T = buildTable();

if nargin == 0
    out = T;
    return
end

validateattributes(classId, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 7}, ...
    mfilename, 'classId');

if classId == 0
    out = unknownPrototype();
else
    out = T(classId);
end
end

% ------------------------------------------------------------------------
function T = buildTable()
persistent cached
if ~isempty(cached)
    T = cached;
    return
end

%        id name            short   L     W     H     w_c   vmin  vmax  aMax  model         qLon  qLat
raw = {
    1, 'car',          'car',  4.20, 1.80, 1.50, 0.55, 5.0, 16.0, 2.5, 'vehicle',    0.60, 0.25
    2, 'bus',          'bus',  10.5, 2.60, 3.20, 0.50, 4.0, 13.0, 1.2, 'vehicle',    0.50, 0.15
    3, 'auto',         'auto', 2.60, 1.40, 1.70, 0.65, 3.0, 11.0, 2.0, 'auto',       0.70, 0.55
    4, 'twowheeler',   '2w',   1.80, 0.70, 1.40, 0.85, 3.0, 15.0, 3.0, 'twowheeler', 0.80, 0.90
    5, 'pedestrian',   'ped',  0.60, 0.60, 1.70, 1.00, 0.8,  1.8, 1.0, 'pedestrian', 0.45, 0.45
    6, 'pushcart',     'cart', 2.00, 1.00, 1.20, 0.75, 0.5,  1.2, 0.5, 'pushcart',   0.20, 0.20
    7, 'cow',          'cow',  2.00, 0.70, 1.40, 1.00, 0.0,  1.5, 1.0, 'cow',        0.55, 0.55
    };

T = struct('id', {}, 'name', {}, 'shortName', {}, 'length', {}, 'width', {}, ...
    'height', {}, 'riskWeight', {}, 'speedRange', {}, 'accelMax', {}, ...
    'motionModel', {}, 'qLon', {}, 'qLat', {}, 'behavior', {});

for k = 1:size(raw, 1)
    T(k).id          = raw{k, 1};
    T(k).name        = raw{k, 2};
    T(k).shortName   = raw{k, 3};
    T(k).length      = raw{k, 4};
    T(k).width       = raw{k, 5};
    T(k).height      = raw{k, 6};
    T(k).riskWeight  = raw{k, 7};
    T(k).speedRange  = [raw{k, 8}, raw{k, 9}];
    T(k).accelMax    = raw{k, 10};
    T(k).motionModel = raw{k, 11};
    T(k).qLon        = raw{k, 12};
    T(k).qLat        = raw{k, 13};
    T(k).behavior    = behaviorParams(raw{k, 2});
end

cached = T;
end

% ------------------------------------------------------------------------
function b = behaviorParams(name)
%BEHAVIORPARAMS  Simulated-agent behaviour knobs (Section 5.1).
%
%   These drive AgentModel, i.e. the WORLD, not the ego.  They are what makes
%   the traffic behave like Indian traffic rather than like a lane-following
%   benchmark.

b = struct();
b.speedNoiseSigma  = 0.4;   % m/s, Ornstein-Uhlenbeck speed jitter
b.speedNoiseTau    = 2.0;   % s, correlation time of that jitter
b.lateralDriftAmp  = 0.0;   % m, amplitude of slow lateral wander
b.lateralDriftTau  = 6.0;   % s, period of that wander
b.reactsToObstacles = true; % brakes for what is directly ahead
b.followGap        = 6.0;   % m, desired gap before braking
b.crossProb        = 0.0;   % probability of initiating a road crossing
b.dwellProb        = 0.0;   % probability per second of stopping to dwell

switch name
    case 'car'
        b.lateralDriftAmp = 0.15;
        b.followGap = 8.0;
    case 'bus'
        b.lateralDriftAmp = 0.10;
        b.followGap = 12.0;
        b.speedNoiseSigma = 0.25;
    case 'auto'
        % Section 5.1: "autos do occasional lateral drift (0.3-0.8 m)".
        b.lateralDriftAmp = 0.55;    % mid-range; per-agent draw in AgentModel
        b.followGap = 5.0;
        b.speedNoiseSigma = 0.6;
    case 'twowheeler'
        % Highest lateral variance: filters between vehicles, cuts gaps.
        b.lateralDriftAmp = 0.9;
        b.lateralDriftTau = 3.5;
        b.followGap = 3.0;
        b.speedNoiseSigma = 0.7;
    case 'pedestrian'
        b.reactsToObstacles = false;  % pedestrians here do not defer to the ego
        b.speedNoiseSigma = 0.15;
        b.crossProb = 0.5;            % per-scenario override
    case 'pushcart'
        b.reactsToObstacles = false;
        b.speedNoiseSigma = 0.10;
        b.dwellProb = 0.05;           % long dwells at the edge
    case 'cow'
        b.reactsToObstacles = false;  % the cow is the hazard, it does not yield
        b.speedNoiseSigma = 0.05;
end
end

% ------------------------------------------------------------------------
function p = unknownPrototype()
%UNKNOWNPROTOTYPE  classId 0: a track the camera never classified.
%
%   Deliberately the most demanding entry in the table - largest footprint
%   among the vulnerable classes, highest risk weight, largest covariance
%   growth.  An unknown object must never be cheaper to plan around than a
%   known one, or the planner would be rewarded for classification failure.
T = buildTable();
p = T(1);
p.id          = 0;
p.name        = 'unknown';
p.shortName   = '?';
p.length      = 2.0;
p.width       = 1.0;
p.height      = 1.7;
p.riskWeight  = 1.00;
p.speedRange  = [0.0, 16.0];
p.accelMax    = 3.0;
p.motionModel = 'cv';
p.qLon        = max([T.qLon]);
p.qLat        = max([T.qLat]);
end
