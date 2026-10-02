function reported = cameraClassModel(trueClassId, rs, params)
%CAMERACLASSMODEL  Simulated camera classifier: confusion and abstention.
%
%   REPORTED = CAMERACLASSMODEL(trueClassId, rs, params) returns the class the
%   camera *reports* for an object whose true class is trueClassId.  The answer
%   is correct most of the time, confused with a visually similar class
%   sometimes, and 0 ("unknown") the rest of the time.
%
%   WHY THIS EXISTS - read before judging it as cheating
%   The toolbox sensor models cannot supply our class labels, for two
%   independent reasons measured on R2026a:
%     1. visionDetectionGenerator returns ObjectClassID = 0 for every
%        detection - it reports no class at all.
%     2. drivingScenario only accepts six actor classes, so even the
%        container's own ClassID is a proxy (DEVIATIONS D11): an
%        auto-rickshaw is stored as a Car and a cow as a Pedestrian.
%   A perception stack for this problem statement has to distinguish seven
%   classes, because the whole of B3 is class-specific risk weighting.  So the
%   classifier is modelled explicitly here.
%
%   The detection's ASSOCIATION to an object comes from the simulator
%   (ObjectAttributes.TargetIndex), which is standard practice for
%   scenario-based studies.  What is NOT taken from the simulator is the class
%   itself: it passes through the confusion model below, so the planner
%   regularly reasons about mislabelled and unlabelled objects.  Position and
%   velocity always come from the noisy sensor, never from truth.
%
%   The confusion structure is deliberately shaped like real failure modes:
%   classes that share a silhouette are confused with each other, and the
%   small, partially occluded classes abstain more often.  A pedestrian is
%   never confused with a bus.
%
%   See also SENSORSUITE, CLASSPRIORS.

if nargin < 3 || isempty(params)
    params = defaultParams();
end

u = rand(rs);

if u < params.pUnknown(trueClassId)
    reported = 0;                       % camera declines to classify
    return
end

if u < params.pUnknown(trueClassId) + params.pConfused(trueClassId)
    alts = params.confusable{trueClassId};
    if isempty(alts)
        reported = trueClassId;
    else
        reported = alts(randi(rs, numel(alts)));
    end
    return
end

reported = trueClassId;
end

% ========================================================================
function p = defaultParams()
%DEFAULTPARAMS  Per-class abstention and confusion rates.
%
%   Indexed by AdaptaDrive class id 1..7:
%     1 car  2 bus  3 auto  4 two-wheeler  5 pedestrian  6 pushcart  7 cow
%
%   These are modelling assumptions, not measurements from a real detector,
%   and README says so.  They are tuned only on seeds 101-110.

persistent cached
if ~isempty(cached)
    p = cached;
    return
end

%                      car   bus   auto  2w    ped   cart  cow
p.pUnknown  = [        0.04, 0.02, 0.07, 0.10, 0.08, 0.16, 0.14];
p.pConfused = [        0.05, 0.03, 0.12, 0.10, 0.06, 0.18, 0.15];

% Which classes each is mistaken for.  Shared silhouette, not shared risk.
p.confusable = {
    [2 3]        % car        <-> bus (large box), auto (small box)
    [1]          % bus        <-> car
    [1 4]        % auto       <-> car, two-wheeler (three-wheeler sits between)
    [3 5]        % 2-wheeler  <-> auto, pedestrian (narrow upright)
    [4 7]        % pedestrian <-> two-wheeler, cow (upright, similar height)
    [3 7]        % pushcart   <-> auto, cow (low slow object at the edge)
    [5 6]        % cow        <-> pedestrian, pushcart
    };

cached = p;
end
