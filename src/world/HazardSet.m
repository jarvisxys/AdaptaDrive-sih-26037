classdef HazardSet < handle
    %HAZARDSET  Road-surface hazards and static obstacles (requirement A1).
    %
    %   Three kinds of thing live here, and they are scored differently:
    %
    %     potholes       ellipses on the road surface.  Driving over one is a
    %                    HAZARD TRAVERSAL, logged and counted - it is not a
    %                    collision and does not fail the run.  A1 asks for
    %                    zero traversals and 0.5 m clearance, so both the
    %                    count and the clearance are measured.
    %
    %     statics        stalls, parked vehicles, debris: oriented boxes that
    %                    are genuinely solid.  Hitting one IS a collision.
    %
    %     edgeBreaks     polygons of broken carriageway at the road edge.
    %                    Not drivable, and they feed the static risk layer.
    %
    %   Keeping potholes distinct from solid obstacles is the point of A1: a
    %   planner that treats a pothole as a wall will refuse to make progress
    %   on a road that is mostly potholes, and a planner that ignores it
    %   entirely damages the vehicle.  It has to be a cost, not a barrier.
    %
    %   See also ROADMODEL, RISKMAP, SCENARIO1.

    properties (SetAccess = private)
        potholes    % struct array: x, y, a, b, yaw, depth
        statics     % struct array: x, y, yaw, L, W, type
        edgeBreaks  % cell array of n x 2 polygons
    end

    methods
        function obj = HazardSet()
            obj.potholes   = emptyPotholeStruct();
            obj.statics    = emptyStaticStruct();
            obj.edgeBreaks = {};
        end

        % ----------------------------------------------------------------
        function addPothole(obj, x, y, a, b, yaw, depth)
            %ADDPOTHOLE  Elliptical surface defect with semi-axes a and b.
            if nargin < 6 || isempty(yaw),   yaw = 0;     end
            if nargin < 7 || isempty(depth), depth = 0.1; end
            n = numel(obj.potholes) + 1;
            obj.potholes(n).x = x;
            obj.potholes(n).y = y;
            obj.potholes(n).a = a;
            obj.potholes(n).b = b;
            obj.potholes(n).yaw = yaw;
            obj.potholes(n).depth = depth;
        end

        % ----------------------------------------------------------------
        function addStatic(obj, x, y, yaw, L, W, type)
            %ADDSTATIC  Solid obstacle (stall, parked vehicle, debris).
            if nargin < 7 || isempty(type), type = 'obstacle'; end
            n = numel(obj.statics) + 1;
            obj.statics(n).x = x;
            obj.statics(n).y = y;
            obj.statics(n).yaw = yaw;
            obj.statics(n).L = L;
            obj.statics(n).W = W;
            obj.statics(n).type = type;
        end

        % ----------------------------------------------------------------
        function addEdgeBreak(obj, poly)
            %ADDEDGEBREAK  Polygon of broken carriageway (n x 2, world frame).
            obj.edgeBreaks{end+1} = poly;
        end

        % ----------------------------------------------------------------
        function tf = inPothole(obj, x, y)
            %INPOTHOLE  True for query points inside any pothole ellipse.
            tf = false(size(x));
            for k = 1:numel(obj.potholes)
                p = obj.potholes(k);
                c = cos(p.yaw); s = sin(p.yaw);
                rx =  (x - p.x) * c + (y - p.y) * s;
                ry = -(x - p.x) * s + (y - p.y) * c;
                tf = tf | ((rx / p.a).^2 + (ry / p.b).^2 <= 1);
            end
        end

        % ----------------------------------------------------------------
        function [hit, ids] = potholesUnderFootprint(obj, obb)
            %POTHOLESUNDERFOOTPRINT  Which potholes the given box drives over.
            %
            %   An ellipse and a box intersect when the ellipse centre is in
            %   the box, or any sampled boundary point is.  16 boundary
            %   samples resolve the smallest pothole in the scenarios (0.6 m)
            %   to about 12 cm, well under the 0.25 m the planner works at.
            ids = [];
            for k = 1:numel(obj.potholes)
                if obj.ellipseHitsOBB(obj.potholes(k), obb)
                    ids(end+1) = k; %#ok<AGROW>
                end
            end
            hit = ~isempty(ids);
        end

        % ----------------------------------------------------------------
        function d = clearanceToPotholes(obj, obb)
            %CLEARANCETOPOTHOLES  Distance from a box to the nearest pothole.
            %   0 when the box is over one; Inf when there are none.
            %   This is the quantity A1 asks to be at least 0.5 m.
            d = inf;
            for k = 1:numel(obj.potholes)
                p = obj.potholes(k);
                [bx, by] = obj.ellipseBoundary(p, 32);
                dk = min(pointOBBDistance(bx, by, obb));
                if pointInOBB(p.x, p.y, obb)
                    dk = 0;
                end
                d = min(d, dk);
            end
        end

        % ----------------------------------------------------------------
        function o = staticOBBs(obj)
            %STATICOBBS  Solid obstacles as OBB structs for collision tests.
            o = repmat(makeOBB(0, 0, 0, 1, 1), 1, 0);
            for k = 1:numel(obj.statics)
                s = obj.statics(k);
                o(k) = makeOBB(s.x, s.y, s.yaw, s.L, s.W); %#ok<AGROW>
            end
        end

        % ----------------------------------------------------------------
        function sev = blockingSeverity(obj, X, Y, marginM)
            %BLOCKINGSEVERITY  Only the hazards that are genuinely impassable.
            %
            %   Stalls, parked vehicles, debris and broken carriageway - the
            %   things a vehicle cannot drive over.  POTHOLES ARE EXCLUDED on
            %   purpose: they belong in the cost, not in the walls.
            %
            %   Treating a pothole as a barrier blocks the road.  Measured: a
            %   pothole's above-threshold region plus the planner's inflation
            %   is a ~2.15 m no-go radius, and two of those on a 6 m
            %   carriageway leave no corridor at all - the global planner
            %   failed on almost every cycle and the ego crawled 65 m in
            %   106 s.  A1 asks for zero traversals AND clearance, which is a
            %   statement about cost, not about impassability.
            if nargin < 4 || isempty(marginM), marginM = 0.5; end
            sev = zeros(size(X));

            for k = 1:numel(obj.statics)
                s = obj.statics(k);
                o = makeOBB(s.x, s.y, s.yaw, s.L, s.W);
                d = pointOBBDistance(X, Y, o);
                sev = max(sev, clip01(1 - d / marginM));
            end

            for k = 1:numel(obj.edgeBreaks)
                poly = obj.edgeBreaks{k};
                in = inpolygon(X, Y, poly(:,1), poly(:,2));
                sev(in) = 1;
            end
        end

        % ----------------------------------------------------------------
        function sev = severity(obj, X, Y, marginM)
            %SEVERITY  Static hazard field S in [0,1] for the risk map.
            %
            %   1 inside a hazard, decaying linearly to 0 at MARGINM beyond
            %   it.  The decay is what makes the planner leave room rather
            %   than graze the edge of every pothole.
            if nargin < 4 || isempty(marginM), marginM = 0.5; end
            sev = zeros(size(X));

            for k = 1:numel(obj.potholes)
                p = obj.potholes(k);
                c = cos(p.yaw); s = sin(p.yaw);
                rx =  (X - p.x) * c + (Y - p.y) * s;
                ry = -(X - p.x) * s + (Y - p.y) * c;
                % Normalised radius; scaled back to metres by the smaller axis
                % so the margin means roughly the same on elongated potholes.
                r = sqrt((rx / p.a).^2 + (ry / p.b).^2);
                approxM = (r - 1) * min(p.a, p.b);
                sev = max(sev, clip01(1 - approxM / marginM));
            end

            for k = 1:numel(obj.statics)
                s = obj.statics(k);
                o = makeOBB(s.x, s.y, s.yaw, s.L, s.W);
                d = pointOBBDistance(X, Y, o);
                sev = max(sev, clip01(1 - d / marginM));
            end

            for k = 1:numel(obj.edgeBreaks)
                poly = obj.edgeBreaks{k};
                in = inpolygon(X, Y, poly(:,1), poly(:,2));
                sev(in) = 1;
            end
        end

        % ----------------------------------------------------------------
        function tf = isBlocked(obj, x, y)
            %ISBLOCKED  True where a solid obstacle or broken edge sits.
            %   Potholes are deliberately NOT blocking - see the class note.
            tf = false(size(x));
            for k = 1:numel(obj.statics)
                s = obj.statics(k);
                tf = tf | pointInOBB(x, y, makeOBB(s.x, s.y, s.yaw, s.L, s.W));
            end
            for k = 1:numel(obj.edgeBreaks)
                poly = obj.edgeBreaks{k};
                tf = tf | inpolygon(x, y, poly(:,1), poly(:,2));
            end
        end

        % ----------------------------------------------------------------
        function n = numPotholes(obj)
            n = numel(obj.potholes);
        end
    end

    methods (Static, Access = private)
        function tf = ellipseHitsOBB(p, obb)
            if pointInOBB(p.x, p.y, obb)
                tf = true;
                return
            end
            [bx, by] = HazardSet.ellipseBoundary(p, 16);
            tf = any(pointInOBB(bx, by, obb));
        end

        function [bx, by] = ellipseBoundary(p, n)
            th = linspace(0, 2*pi, n + 1);
            th(end) = [];
            lx = p.a * cos(th);
            ly = p.b * sin(th);
            c = cos(p.yaw); s = sin(p.yaw);
            bx = p.x + lx * c - ly * s;
            by = p.y + lx * s + ly * c;
        end
    end
end

% ========================================================================
function s = emptyPotholeStruct()
s = struct('x', {}, 'y', {}, 'a', {}, 'b', {}, 'yaw', {}, 'depth', {});
end

function s = emptyStaticStruct()
s = struct('x', {}, 'y', {}, 'yaw', {}, 'L', {}, 'W', {}, 'type', {});
end

function v = clip01(v)
v = min(max(v, 0), 1);
end
