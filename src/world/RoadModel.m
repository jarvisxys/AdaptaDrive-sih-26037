classdef RoadModel < handle
    %ROADMODEL  Drivable area and expected-travel-direction field.
    %
    %   A road here is a union of RIBBONS (segments).  Each ribbon is a
    %   centreline with an independent left and right half-width, so edges can
    %   be uneven - a village road is not a constant-width rectangle.
    %
    %   WHAT THE PLANNER IS AND IS NOT GIVEN
    %   The proposed planner may query:
    %       isDrivable        - is this point on the road surface at all
    %       distanceToEdge    - how much room is left before the edge
    %   It is NOT given lane centrelines, a lane graph or an HD map: the whole
    %   point of the problem statement is lane-free planning over a risk field.
    %
    %   The centreline exists for three uses only, all of them declared:
    %       1. constructing the geometry and placing agents,
    %       2. the EXPECTED DIRECTION field, which only the wrong-way detector
    %          (A2) reads,
    %       3. the LaneFollow baseline BL1, which is deliberately handed the
    %          centreline it needs.  ARCHITECTURE.md records that this is
    %          generous to the baseline.
    %
    %   IMPLEMENTATION
    %   Every field is rasterised ONCE at construction onto a world-fixed grid,
    %   because the road is static and the risk map queries tens of thousands
    %   of points at 10 Hz.  Runtime queries are therefore O(1) lookups rather
    %   than repeated nearest-point searches.  Rasterisation stamps each short
    %   sub-segment into its own local window and keeps the nearest hit, which
    %   costs a few hundred thousand operations instead of the hundreds of
    %   millions a brute-force projection over the whole grid would take.
    %
    %   Direction convention: INDIA DRIVES ON THE LEFT.  With t the centreline
    %   tangent and n its left normal, a point at signed lateral offset d > 0
    %   (left of the tangent) is expected to travel along +t, and a point at
    %   d < 0 along -t.  A vehicle heading against that field is what
    %   WrongWayDetector flags.
    %
    %   See also HAZARDSET, AGENTMODEL, WRONGWAYDETECTOR.

    properties (SetAccess = private)
        segments        % struct array, see buildSegment
        res             % m, raster resolution
        x0              % m, world x of raster column 1
        y0              % m, world y of raster row 1
        nx              % raster columns
        ny              % raster rows
        bounds          % [xmin xmax ymin ymax] of the raster

        drivable        % ny x nx logical
        edgeDist        % ny x nx single, >0 inside, distance to nearest edge
        sField          % ny x nx single, arc length along the nearest segment
        dField          % ny x nx single, signed lateral offset (+ = left)
        segField        % ny x nx uint8, index of the nearest segment
        dirCos          % ny x nx single, expected travel direction, x part
        dirSin          % ny x nx single, expected travel direction, y part
        dirDefined      % ny x nx logical, false in junctions
    end

    methods
        function obj = RoadModel(segSpecs, varargin)
            %ROADMODEL  Build the road from one or more ribbon specifications.
            %
            %   segSpecs: struct array with fields
            %       centers          m x 2 control points (required)
            %       width            scalar total width (required unless
            %                        halfWidthL/R given)
            %       name             char, for logs and the UI
            %       twoWay           logical, default true
            %       directionDefined logical, default true; set false for a
            %                        junction box where no direction applies
            %       edgeNoise        m, amplitude of smooth edge unevenness
            %
            %   Options: 'res' (0.25), 'margin' (6), 'rs' (RandStream)

            p = inputParser;
            p.addParameter('res', 0.25, @(x) isscalar(x) && x > 0);
            p.addParameter('margin', 6.0, @(x) isscalar(x) && x >= 0);
            p.addParameter('rs', [], @(x) isempty(x) || isa(x, 'RandStream'));
            p.parse(varargin{:});

            obj.res = p.Results.res;
            rs = p.Results.rs;
            if isempty(rs)
                rs = RandStream('mrg32k3a', 'Seed', 0);
            end

            if numel(segSpecs) > 255
                error('RoadModel:tooManySegments', ...
                    'segField is uint8; at most 255 segments are supported.');
            end

            % --- build each ribbon -------------------------------------
            % Collected in a cell first: indexing into an empty property
            % would try to grow a double array with struct contents.
            segs = cell(1, numel(segSpecs));
            for k = 1:numel(segSpecs)
                segs{k} = obj.buildSegment(segSpecs(k), obj.res, rs);
            end
            obj.segments = [segs{:}];

            obj.rasterise(p.Results.margin);
        end

        % ----------------------------------------------------------------
        function tf = isDrivable(obj, x, y)
            %ISDRIVABLE  True where the point lies on the road surface.
            %   Points outside the raster are not drivable.
            [idx, inside] = obj.worldToIndex(x, y);
            tf = false(size(x));
            tf(inside) = obj.drivable(idx(inside));
        end

        % ----------------------------------------------------------------
        function d = distanceToEdge(obj, x, y)
            %DISTANCETOEDGE  Signed distance to the drivable boundary.
            %   Positive inside the road, negative outside.  Queries beyond
            %   the raster are clamped to the border value, which is already
            %   negative, so the sign is never wrong - only the magnitude
            %   saturates.  Used by the edge-risk layer E and by the planner's
            %   clearance term.
            [idx, ~] = obj.worldToIndex(x, y, true);
            d = double(obj.edgeDist(idx));
        end

        % ----------------------------------------------------------------
        function [c, s, defined] = expectedDirection(obj, x, y)
            %EXPECTEDDIRECTION  Unit vector of expected travel at a point.
            %
            %   [C, S, DEFINED] = ... returns the direction as cos/sin parts
            %   rather than an angle, because angles cannot be interpolated or
            %   averaged across the +pi/-pi wrap.  DEFINED is false inside
            %   junctions, where no single direction applies; the wrong-way
            %   detector must ignore those cells rather than guess.
            [idx, inside] = obj.worldToIndex(x, y);
            c = zeros(size(x));
            s = zeros(size(x));
            defined = false(size(x));
            c(inside) = double(obj.dirCos(idx(inside)));
            s(inside) = double(obj.dirSin(idx(inside)));
            defined(inside) = obj.dirDefined(idx(inside));
        end

        % ----------------------------------------------------------------
        function info = nearest(obj, x, y)
            %NEAREST  Arc length, lateral offset and segment at a point.
            %   Raster lookup, so the cost does not depend on road length.
            [idx, inside] = obj.worldToIndex(x, y, true);
            info.s   = double(obj.sField(idx));
            info.d   = double(obj.dField(idx));
            info.seg = double(obj.segField(idx));
            info.inside = inside;
        end

        % ----------------------------------------------------------------
        function [xy, tangent, normal] = pointAt(obj, segIdx, s, d)
            %POINTAT  World point at arc length S and lateral offset D.
            %
            %   Computed analytically from the stored centreline, not from the
            %   raster, so agent paths and the ego reference path are smooth
            %   rather than quantised to 0.25 m.
            %
            %   D is positive to the LEFT of the direction of travel.
            seg = obj.segments(segIdx);
            s = min(max(s(:), 0), seg.length);
            if nargin < 4 || isempty(d)
                d = zeros(size(s));
            end
            d = d(:);
            if isscalar(d)
                d = repmat(d, size(s));
            end

            px = interp1(seg.s, seg.centerline(:, 1), s, 'linear');
            py = interp1(seg.s, seg.centerline(:, 2), s, 'linear');
            tx = interp1(seg.s, seg.tangent(:, 1), s, 'linear');
            ty = interp1(seg.s, seg.tangent(:, 2), s, 'linear');

            nrm = hypot(tx, ty);
            nrm(nrm < eps) = 1;
            tx = tx ./ nrm;
            ty = ty ./ nrm;

            normal  = [-ty, tx];            % left normal
            tangent = [tx, ty];
            xy = [px, py] + d .* normal;
        end

        % ----------------------------------------------------------------
        function L = segmentLength(obj, segIdx)
            L = obj.segments(segIdx).length;
        end

        % ----------------------------------------------------------------
        function [left, right] = boundaryPolylines(obj, segIdx)
            %BOUNDARYPOLYLINES  Left and right edges of one ribbon, for drawing.
            seg = obj.segments(segIdx);
            n = [-seg.tangent(:, 2), seg.tangent(:, 1)];
            left  = seg.centerline + seg.halfWidthL .* n;
            right = seg.centerline - seg.halfWidthR .* n;
        end

        % ----------------------------------------------------------------
        function P = surfacePolygon(obj, segIdx)
            %SURFACEPOLYGON  Closed polygon of one ribbon's drivable surface.
            [left, right] = obj.boundaryPolylines(segIdx);
            P = [left; flipud(right)];
        end

        % ----------------------------------------------------------------
        function ax = gridVectors(obj)
            %GRIDVECTORS  World coordinates of the raster columns and rows.
            ax.x = obj.x0 + (0:obj.nx-1) * obj.res;
            ax.y = obj.y0 + (0:obj.ny-1) * obj.res;
        end
    end

    % ====================================================================
    methods (Access = private)

        function seg = buildSegment(~, spec, res, rs)
            %BUILDSEGMENT  Resample a ribbon to uniform arc length.

            if ~isfield(spec, 'centers') || size(spec.centers, 2) ~= 2
                error('RoadModel:badSpec', 'Each segment needs an m x 2 "centers" field.');
            end
            name = getOr(spec, 'name', 'segment');
            twoWay = getOr(spec, 'twoWay', true);
            dirDefined = getOr(spec, 'directionDefined', true);
            edgeNoise = getOr(spec, 'edgeNoise', 0);

            ds = max(res * 2, 0.5);     % sub-segment length for stamping
            [C, T, s, L] = resampleByArcLength(spec.centers, ds);

            if isfield(spec, 'halfWidthL') && isfield(spec, 'halfWidthR')
                hL = expandWidth(spec.halfWidthL, s);
                hR = expandWidth(spec.halfWidthR, s);
            else
                if ~isfield(spec, 'width')
                    error('RoadModel:badSpec', 'Segment "%s" needs "width" or half-widths.', name);
                end
                hL = repmat(spec.width / 2, size(s));
                hR = repmat(spec.width / 2, size(s));
            end

            % Uneven edges: smooth, seeded, and independent on the two sides.
            % Three harmonics give unevenness that a planner cannot predict
            % from a single wavelength, without introducing sharp steps that
            % would be geometry noise rather than road character.
            if edgeNoise > 0 && L > 0
                hL = hL + smoothEdgeNoise(s, L, edgeNoise, rs);
                hR = hR + smoothEdgeNoise(s, L, edgeNoise, rs);
            end
            minHalf = 1.0;   % never let noise close the road entirely
            hL = max(hL, minHalf);
            hR = max(hR, minHalf);

            seg = struct( ...
                'name', name, ...
                'centerline', C, ...
                'tangent', T, ...
                's', s, ...
                'length', L, ...
                'halfWidthL', hL, ...
                'halfWidthR', hR, ...
                'twoWay', logical(twoWay), ...
                'directionDefined', logical(dirDefined));
        end

        % ----------------------------------------------------------------
        function rasterise(obj, margin)
            %RASTERISE  Stamp every sub-segment into a world-fixed grid.

            % --- extent ------------------------------------------------
            allXY = vertcat(obj.segments.centerline);
            maxHalf = 0;
            for k = 1:numel(obj.segments)
                maxHalf = max([maxHalf; obj.segments(k).halfWidthL; obj.segments(k).halfWidthR]);
            end
            pad = maxHalf + margin;

            xmin = min(allXY(:, 1)) - pad;   xmax = max(allXY(:, 1)) + pad;
            ymin = min(allXY(:, 2)) - pad;   ymax = max(allXY(:, 2)) + pad;

            obj.x0 = xmin;
            obj.y0 = ymin;
            obj.nx = max(2, ceil((xmax - xmin) / obj.res) + 1);
            obj.ny = max(2, ceil((ymax - ymin) / obj.res) + 1);
            obj.bounds = [obj.x0, obj.x0 + (obj.nx-1)*obj.res, ...
                          obj.y0, obj.y0 + (obj.ny-1)*obj.res];

            % --- allocate ----------------------------------------------
            bestDist  = inf(obj.ny, obj.nx, 'single');
            obj.edgeDist   = -inf(obj.ny, obj.nx, 'single');
            obj.sField     = zeros(obj.ny, obj.nx, 'single');
            obj.dField     = zeros(obj.ny, obj.nx, 'single');
            obj.segField   = zeros(obj.ny, obj.nx, 'uint8');
            obj.dirCos     = zeros(obj.ny, obj.nx, 'single');
            obj.dirSin     = zeros(obj.ny, obj.nx, 'single');
            obj.dirDefined = false(obj.ny, obj.nx);

            gx = obj.x0 + (0:obj.nx-1) * obj.res;
            gy = obj.y0 + (0:obj.ny-1) * obj.res;

            for k = 1:numel(obj.segments)
                seg = obj.segments(k);
                C = seg.centerline;
                nSub = size(C, 1) - 1;
                localMaxHalf = max([seg.halfWidthL; seg.halfWidthR]);

                % End caps, as half-planes perpendicular to the end tangents.
                % These are properties of the SEGMENT, not of a sub-segment:
                % interior sub-segments clamp their projection, so without a
                % global clip they would happily claim cells that lie past the
                % start or the finish of the ribbon.
                capP1 = C(1, :);      capT1 = seg.tangent(1, :);
                capP2 = C(end, :);    capT2 = seg.tangent(end, :);

                for j = 1:nSub
                    P1 = C(j, :);
                    P2 = C(j+1, :);
                    dvec = P2 - P1;
                    len = hypot(dvec(1), dvec(2));
                    if len < 1e-9
                        continue
                    end
                    T = dvec / len;

                    % --- local window -------------------------------
                    reach = localMaxHalf + obj.res;
                    c1 = floor((min(P1(1), P2(1)) - reach - obj.x0) / obj.res) + 1;
                    c2 = ceil( (max(P1(1), P2(1)) + reach - obj.x0) / obj.res) + 1;
                    r1 = floor((min(P1(2), P2(2)) - reach - obj.y0) / obj.res) + 1;
                    r2 = ceil( (max(P1(2), P2(2)) + reach - obj.y0) / obj.res) + 1;
                    c1 = max(c1, 1);  c2 = min(c2, obj.nx);
                    r1 = max(r1, 1);  r2 = min(r2, obj.ny);
                    if c1 > c2 || r1 > r2
                        continue
                    end

                    [QX, QY] = meshgrid(gx(c1:c2), gy(r1:r2));
                    relx = QX - P1(1);
                    rely = QY - P1(2);

                    tpar = (relx * T(1) + rely * T(2)) / len;

                    % Interior joints must clamp, so the wedge on the outside
                    % of a bend is covered by one of the two adjacent
                    % sub-segments.  The ribbon's own end caps are applied
                    % globally instead: this segment never claims a cell that
                    % lies past its start or finish, which leaves such cells
                    % free for another segment (a ramp ending inside a
                    % highway must not punch a hole in the highway).
                    beyondStart = ((QX - capP1(1)) * capT1(1) + ...
                                   (QY - capP1(2)) * capT1(2)) < 0;
                    beyondEnd   = ((QX - capP2(1)) * capT2(1) + ...
                                   (QY - capP2(2)) * capT2(2)) > 0;
                    valid = ~beyondStart & ~beyondEnd;
                    tcl = min(max(tpar, 0), 1);

                    projx = P1(1) + tcl * len * T(1);
                    projy = P1(2) + tcl * len * T(2);
                    dist = hypot(QX - projx, QY - projy);

                    % Sign of the lateral offset from the infinite line: the
                    % z component of T x (Q - P1).  Positive means left of the
                    % direction of travel.
                    sgn = sign(T(1) * rely - T(2) * relx);
                    sgn(sgn == 0) = 1;

                    sHere = seg.s(j) + tcl * len;
                    wL = interp1(seg.s, seg.halfWidthL, sHere, 'linear', 'extrap');
                    wR = interp1(seg.s, seg.halfWidthR, sHere, 'linear', 'extrap');

                    % Distance to the nearer edge, measured from the spine.
                    edgeHere = single(zeros(size(dist)));
                    isLeft = sgn > 0;
                    edgeHere(isLeft)  = wL(isLeft)  - dist(isLeft);
                    edgeHere(~isLeft) = wR(~isLeft) - dist(~isLeft);

                    % Expected travel direction (keep left).
                    if seg.twoWay
                        dirSign = sgn;          % left of tangent -> along +t
                    else
                        dirSign = ones(size(sgn));
                    end

                    % --- merge into the rasters ----------------------
                    sub = single(dist);
                    win = bestDist(r1:r2, c1:c2);
                    better = valid & (sub < win);
                    if ~any(better(:))
                        continue
                    end

                    idxWin = find(better);
                    [rr, cc] = ind2sub(size(better), idxWin);
                    gidx = sub2ind([obj.ny, obj.nx], rr + r1 - 1, cc + c1 - 1);

                    bestDist(gidx)      = sub(idxWin);
                    obj.edgeDist(gidx)  = edgeHere(idxWin);
                    obj.sField(gidx)    = single(sHere(idxWin));
                    obj.dField(gidx)    = single(sgn(idxWin) .* dist(idxWin));
                    obj.segField(gidx)  = uint8(k);
                    obj.dirCos(gidx)    = single(dirSign(idxWin) * T(1));
                    obj.dirSin(gidx)    = single(dirSign(idxWin) * T(2));
                    obj.dirDefined(gidx) = seg.directionDefined;
                end
            end

            % Cells no sub-segment ever reached are far off-road.  Give them a
            % finite, honestly negative edge distance instead of -inf so that
            % downstream arithmetic stays well defined.
            untouched = isinf(obj.edgeDist);
            obj.edgeDist(untouched) = single(-margin);

            obj.drivable = obj.edgeDist > 0;
            obj.dirDefined = obj.dirDefined & obj.drivable;
        end

        % ----------------------------------------------------------------
        function [idx, inside] = worldToIndex(obj, x, y, clampToGrid)
            %WORLDTOINDEX  Nearest raster cell for world points.
            if nargin < 4
                clampToGrid = false;
            end
            c = round((x - obj.x0) / obj.res) + 1;
            r = round((y - obj.y0) / obj.res) + 1;

            inside = c >= 1 & c <= obj.nx & r >= 1 & r <= obj.ny;

            if clampToGrid
                c = min(max(c, 1), obj.nx);
                r = min(max(r, 1), obj.ny);
                idx = sub2ind([obj.ny, obj.nx], r, c);
            else
                idx = ones(size(c));
                idx(inside) = sub2ind([obj.ny, obj.nx], r(inside), c(inside));
            end
        end
    end
end

% ========================================================================
function [C, T, s, L] = resampleByArcLength(centers, ds)
%RESAMPLEBYARCLENGTH  Smooth the control points and resample at uniform ds.
%
%   Two passes: interpolate densely with pchip (which does not overshoot the
%   way a spline does, so the road never bulges outside its control points),
%   measure true arc length on that dense curve, then resample uniformly.

if size(centers, 1) < 2
    error('RoadModel:shortCenterline', 'A segment needs at least two control points.');
end

if size(centers, 1) == 2
    dense = [linspace(centers(1,1), centers(2,1), 200)', ...
             linspace(centers(1,2), centers(2,2), 200)'];
else
    chord = [0; cumsum(hypot(diff(centers(:,1)), diff(centers(:,2))))];
    u = linspace(0, chord(end), max(400, 20 * size(centers, 1)))';
    dense = [interp1(chord, centers(:,1), u, 'pchip'), ...
             interp1(chord, centers(:,2), u, 'pchip')];
end

arc = [0; cumsum(hypot(diff(dense(:,1)), diff(dense(:,2))))];
L = arc(end);

n = max(3, round(L / ds) + 1);
s = linspace(0, L, n)';
C = [interp1(arc, dense(:,1), s, 'linear'), ...
     interp1(arc, dense(:,2), s, 'linear')];

% Central-difference tangents, normalised.
T = zeros(size(C));
T(2:end-1, :) = C(3:end, :) - C(1:end-2, :);
T(1, :)   = C(2, :) - C(1, :);
T(end, :) = C(end, :) - C(end-1, :);
nrm = hypot(T(:,1), T(:,2));
nrm(nrm < eps) = 1;
T = T ./ nrm;
end

% ========================================================================
function w = smoothEdgeNoise(s, L, amp, rs)
%SMOOTHEDGENOISE  Seeded, smooth, zero-mean unevenness along a road edge.
w = zeros(size(s));
for h = 1:3
    phase = rand(rs) * 2 * pi;
    wavelength = L / (1.5 * h + rand(rs));
    w = w + (amp / h) * sin(2*pi*s / max(wavelength, 1) + phase);
end
end

% ========================================================================
function w = expandWidth(spec, s)
%EXPANDWIDTH  Accept a scalar half-width or a two-column [s, w] profile.
if isscalar(spec)
    w = repmat(spec, size(s));
elseif size(spec, 2) == 2
    w = interp1(spec(:,1), spec(:,2), s, 'linear', 'extrap');
else
    w = reshape(spec, size(s));
end
end

% ========================================================================
function v = getOr(s, f, dflt)
if isfield(s, f) && ~isempty(s.(f))
    v = s.(f);
else
    v = dflt;
end
end
