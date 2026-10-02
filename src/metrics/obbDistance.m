function d = obbDistance(a, b)
%OBBDISTANCE  Minimum distance between two oriented bounding boxes.
%
%   D = OBBDISTANCE(A, B) returns 0 when the boxes overlap, otherwise the
%   shortest distance between their boundaries.  This is the quantity
%   reported as "minimum clearance", so it is the true box-to-box gap, not a
%   centre-to-centre distance minus a radius.
%
%   Method: for two disjoint convex polygons the minimum distance is attained
%   between a pair of edges, so the exact answer is the minimum over the 16
%   edge pairs of the segment-to-segment distance.
%
%   See also OBBOVERLAP, MAKEOBB.

if obbOverlap(a, b)
    d = 0;
    return
end

Ca = obbCorners(a);
Cb = obbCorners(b);

d = inf;
for i = 1:4
    p1 = Ca(i, :);
    p2 = Ca(mod(i, 4) + 1, :);
    for j = 1:4
        q1 = Cb(j, :);
        q2 = Cb(mod(j, 4) + 1, :);
        d = min(d, segSegDistance(p1, p2, q1, q2));
        if d == 0
            return
        end
    end
end
end

% ------------------------------------------------------------------------
function d = segSegDistance(p1, p2, q1, q2)
%SEGSEGDISTANCE  Distance between segments p1-p2 and q1-q2 in 2-D.
%
%   For two segments in the plane that do NOT cross, the minimum distance is
%   always attained at an endpoint of one of them.  So the exact answer is
%   the smallest of four point-to-segment distances - no parameter clamping
%   to get subtly wrong.  The crossing case is tested first and returns 0.

if segmentsIntersect(p1, p2, q1, q2)
    d = 0;
    return
end

d = min([pointSegDistance(p1, q1, q2)
         pointSegDistance(p2, q1, q2)
         pointSegDistance(q1, p1, p2)
         pointSegDistance(q2, p1, p2)]);
end

% ------------------------------------------------------------------------
function d = pointSegDistance(p, a, b)
%POINTSEGDISTANCE  Distance from point p to segment a-b.
ab = b - a;
den = dot(ab, ab);
if den < 1e-14
    d = hypot(p(1) - a(1), p(2) - a(2));   % degenerate segment
    return
end
t = min(max(dot(p - a, ab) / den, 0), 1);
closest = a + t * ab;
d = hypot(p(1) - closest(1), p(2) - closest(2));
end

% ------------------------------------------------------------------------
function tf = segmentsIntersect(p1, p2, q1, q2)
%SEGMENTSINTERSECT  Proper or touching intersection of two 2-D segments.
d1 = cross2(q2 - q1, p1 - q1);
d2 = cross2(q2 - q1, p2 - q1);
d3 = cross2(p2 - p1, q1 - p1);
d4 = cross2(p2 - p1, q2 - p1);

if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ...
   ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
    tf = true;
    return
end

% Collinear touching cases.
tf = (abs(d1) < 1e-12 && onSegment(q1, q2, p1)) || ...
     (abs(d2) < 1e-12 && onSegment(q1, q2, p2)) || ...
     (abs(d3) < 1e-12 && onSegment(p1, p2, q1)) || ...
     (abs(d4) < 1e-12 && onSegment(p1, p2, q2));
end

% ------------------------------------------------------------------------
function c = cross2(u, v)
c = u(1)*v(2) - u(2)*v(1);
end

% ------------------------------------------------------------------------
function tf = onSegment(a, b, p)
tf = min(a(1), b(1)) - 1e-12 <= p(1) && p(1) <= max(a(1), b(1)) + 1e-12 && ...
     min(a(2), b(2)) - 1e-12 <= p(2) && p(2) <= max(a(2), b(2)) + 1e-12;
end
