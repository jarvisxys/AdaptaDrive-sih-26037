function gap = capsuleGap(p1, p2, q1, q2, rSum)
%CAPSULEGAP  Vectorised gap between two capsules, one pair per row.
%
%   GAP = CAPSULEGAP(P1, P2, Q1, Q2, RSUM) treats each row as a capsule pair:
%   the first is the segment P1->P2 swollen by a radius, the second Q1->Q2,
%   and RSUM is the sum of the two radii.  The result is the clearance between
%   their surfaces, negative when they overlap.
%
%   WHY CAPSULES RATHER THAN CIRCLES
%   A rectangle L x W is exactly contained in the capsule of radius W/2 around
%   its own centreline - the capsule only rounds the corners outward.  That
%   makes it a TIGHT conservative bound, where a single bounding circle is a
%   disastrously loose one: the bounding circle of a 4.5 x 1.8 m car has
%   radius 2.42 m, which asserts the car is 4.8 m wide.
%
%   That looseness is not academic.  Two vehicles passing on a 6 m two-way
%   road are about 3.0 m apart, while the bounding circles of an ego and an
%   oncoming car sum to 3.46 m - so every oncoming vehicle read as a collision,
%   every rollout was rejected, and the ego stopped dead and could not pass a
%   car that was comfortably in its own half of the road.  With capsules the
%   same pair reports the true body gap of about 1.2 m.
%
%   Capsules are also CHEAPER than the multi-circle chains that would be
%   needed for comparable tightness: one segment-to-segment distance per pair
%   instead of 15 circle-to-circle tests.
%
%   See also OBBDISTANCE, DWAPLANNER.

d = segSegDistanceVec(p1, p2, q1, q2);
gap = d - rSum;
end

% ========================================================================
function d = segSegDistanceVec(p1, p2, q1, q2)
%SEGSEGDISTANCEVEC  Distance between segment pairs, one pair per row.

% For two segments in the plane that do not cross, the minimum distance is
% attained at an endpoint of one of them, so four point-to-segment distances
% cover every non-crossing case.
d = min([pointSegVec(p1, q1, q2), ...
         pointSegVec(p2, q1, q2), ...
         pointSegVec(q1, p1, p2), ...
         pointSegVec(q2, p1, p2)], [], 2);

% Crossing segments are at distance 0, and the endpoint minimum above would
% report a positive value for them - an overlap reported as clearance is the
% one error direction a safety check must never make.
d(segmentsCrossVec(p1, p2, q1, q2)) = 0;
end

% ========================================================================
function d = pointSegVec(P, A, B)
%POINTSEGVEC  Distance from each point P to the corresponding segment A-B.
AB = B - A;
den = max(sum(AB .* AB, 2), eps);
t = min(max(sum((P - A) .* AB, 2) ./ den, 0), 1);
C = A + t .* AB;
d = hypot(P(:,1) - C(:,1), P(:,2) - C(:,2));
end

% ========================================================================
function tf = segmentsCrossVec(p1, p2, q1, q2)
%SEGMENTSCROSSVEC  Proper intersection test, one pair per row.
d1 = cross2(q2 - q1, p1 - q1);
d2 = cross2(q2 - q1, p2 - q1);
d3 = cross2(p2 - p1, q1 - p1);
d4 = cross2(p2 - p1, q2 - p1);

tf = ((d1 > 0 & d2 < 0) | (d1 < 0 & d2 > 0)) & ...
     ((d3 > 0 & d4 < 0) | (d3 < 0 & d4 > 0));
end

% ========================================================================
function c = cross2(u, v)
c = u(:,1) .* v(:,2) - u(:,2) .* v(:,1);
end
