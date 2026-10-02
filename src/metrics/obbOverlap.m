function tf = obbOverlap(a, b)
%OBBOVERLAP  True when two oriented bounding boxes intersect (SAT).
%
%   TF = OBBOVERLAP(A, B) uses the separating-axis theorem.  For two convex
%   rectangles it is sufficient to test the four edge normals (two per box):
%   if the projections onto any one of them are disjoint, the boxes are
%   disjoint; if none separates them, they intersect.
%
%   Touching exactly counts as overlapping (>= on the projection test), which
%   is the conservative choice for a collision check.
%
%   This decides whether a run is scored as a COLLISION, so it is exact -
%   no inflation, no approximation by circles.
%
%   See also OBBDISTANCE, MAKEOBB.

% Cheap rejection first: bounding circles.  Most pairs in a scenario are far
% apart, and this skips the projection work for them.
dx = b.x - a.x;
dy = b.y - a.y;
ra = 0.5 * hypot(a.L, a.W);
rb = 0.5 * hypot(b.L, b.W);
if (dx*dx + dy*dy) > (ra + rb)^2
    tf = false;
    return
end

Ca = obbCorners(a);
Cb = obbCorners(b);

axesToTest = [ cos(a.yaw), sin(a.yaw)
              -sin(a.yaw), cos(a.yaw)
               cos(b.yaw), sin(b.yaw)
              -sin(b.yaw), cos(b.yaw)];

for k = 1:4
    ax = axesToTest(k, :);
    pa = Ca * ax.';
    pb = Cb * ax.';
    if max(pa) < min(pb) || max(pb) < min(pa)
        tf = false;   % found a separating axis
        return
    end
end

tf = true;
end
