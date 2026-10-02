function [t, info] = ttc(a, va, b, vb, tMax, dt)
%TTC  Time to collision between two oriented boxes holding current velocity.
%
%   T = TTC(A, VA, B, VB) returns the time in seconds until the footprints of
%   A and B first overlap, assuming both keep their current velocity and
%   heading.  If they never overlap within the cap, T is the cap.
%
%   T = TTC(A, VA, B, VB, TMAX, DT) sets the cap (default 10 s) and the
%   sampling step (default 0.02 s).
%
%   [T, INFO] = ... also returns
%       info.collides   true when an overlap was found within the cap
%       info.capped     true when the answer is the cap, not a real overlap
%       info.dMin       minimum box-to-box distance over the searched window
%
%   Convention (Section 6): parallel travel that never intersects yields the
%   cap, and the metric code treats the cap as "no conflict".  Reporting a
%   capped value as though it were a measured TTC would understate risk, so
%   INFO.CAPPED is carried through to the metrics layer rather than discarded.
%
%   Method: the boxes can only touch while their bounding circles overlap.
%   That gives a closed-form quadratic for the candidate time window, and the
%   exact SAT test is sampled only inside it.  Cheap for the many far-apart
%   pairs, exact where it matters.
%
%   See also OBBOVERLAP, OBBDISTANCE.

if nargin < 5 || isempty(tMax), tMax = 10.0; end
if nargin < 6 || isempty(dt),   dt   = 0.05; end

% dMin costs an exact box-to-box distance at every sampled instant, and it is
% the single most expensive thing in the whole simulation when left on: 849k
% segment-distance evaluations in a 32 s run, 14 s of wall clock.  Almost every
% caller wants only the time, so the distance is computed ONLY when asked for.
needDMin = nargout > 1;

info = struct('collides', false, 'capped', true, 'dMin', inf);

% Relative motion of B with respect to A.
p0 = [b.x - a.x, b.y - a.y];
dv = [vb(1) - va(1), vb(2) - va(2)];

ra = 0.5 * hypot(a.L, a.W);
rb = 0.5 * hypot(b.L, b.W);
R  = ra + rb;

% |p0 + dv*t| <= R  ->  |dv|^2 t^2 + 2(p0.dv) t + (|p0|^2 - R^2) <= 0
A = dot(dv, dv);
B = 2 * dot(p0, dv);
C = dot(p0, p0) - R*R;

if C <= 0
    tLo = 0;                     % already within the circle bound
else
    if A < 1e-12
        t = tMax;                % no relative motion and not already close
        if needDMin, info.dMin = obbDistance(a, b); end
        return
    end
    disc = B*B - 4*A*C;
    if disc < 0
        t = tMax;                % circles never meet
        if needDMin, info.dMin = obbDistance(a, b); end
        return
    end
    tLo = (-B - sqrt(disc)) / (2*A);
end

if A < 1e-12
    tHi = tMax;
else
    disc = B*B - 4*A*C;
    if disc < 0
        t = tMax;
        if needDMin, info.dMin = obbDistance(a, b); end
        return
    end
    tHi = (-B + sqrt(disc)) / (2*A);
end

tLo = max(tLo, 0);
tHi = min(tHi, tMax);

if tHi < tLo
    t = tMax;                    % the circle window is entirely in the past
    if needDMin, info.dMin = obbDistance(a, b); end
    return
end

% Exact test inside the candidate window only.
aa = a;
bb = b;
for tk = tLo:dt:tHi
    aa.x = a.x + va(1)*tk;
    aa.y = a.y + va(2)*tk;
    bb.x = b.x + vb(1)*tk;
    bb.y = b.y + vb(2)*tk;

    if obbOverlap(aa, bb)
        t = tk;
        info.collides = true;
        info.capped   = false;
        info.dMin     = 0;
        return
    end
    if needDMin
        info.dMin = min(info.dMin, obbDistance(aa, bb));
    end
end

t = tMax;
end
