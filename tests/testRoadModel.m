classdef testRoadModel < matlab.unittest.TestCase
    %TESTROADMODEL  Drivable area, edge distance and expected-direction field.
    %
    %   The expected-direction field is what requirement A2 (wrong-way
    %   detection) is built on, so the keep-left convention is pinned down
    %   here rather than left implicit in the detector.

    methods (Test)

        function straightRoadDrivableBand(tc)
            % 100 m straight road, 6 m wide: drivable for |lateral| < 3.
            spec = struct('centers', [0 0; 100 0], 'width', 6, 'name', 'straight');
            rm = RoadModel(spec, 'res', 0.25);

            tc.verifyTrue(rm.isDrivable(50,  0));
            tc.verifyTrue(rm.isDrivable(50,  2.5));
            tc.verifyTrue(rm.isDrivable(50, -2.5));
            tc.verifyFalse(rm.isDrivable(50,  4));
            tc.verifyFalse(rm.isDrivable(50, -4));
        end

        function edgeDistanceIsHalfWidthOnCentreline(tc)
            spec = struct('centers', [0 0; 100 0], 'width', 6, 'name', 'straight');
            rm = RoadModel(spec, 'res', 0.25);

            % On the centreline the nearer edge is 3 m away.
            tc.verifyEqual(rm.distanceToEdge(50, 0), 3, 'AbsTol', 0.2);
            % 2 m to the left, the left edge is 1 m away.
            tc.verifyEqual(rm.distanceToEdge(50, 2), 1, 'AbsTol', 0.2);
            % Outside the road the distance is negative.
            tc.verifyLessThan(rm.distanceToEdge(50, 5), 0);
        end

        function noRoadBeyondTheEnds(tc)
            % A rounded cap past the end would hand the planner drivable area
            % that does not exist.
            spec = struct('centers', [0 0; 100 0], 'width', 6, 'name', 'straight');
            rm = RoadModel(spec, 'res', 0.25);

            tc.verifyTrue(rm.isDrivable(1, 0));
            tc.verifyTrue(rm.isDrivable(99, 0));
            tc.verifyFalse(rm.isDrivable(-2, 0), 'Road must not extend before its start.');
            tc.verifyFalse(rm.isDrivable(102, 0), 'Road must not extend past its end.');
        end

        function keepLeftDirectionConvention(tc)
            % India drives on the left.  Road runs along +x, so the tangent is
            % +x.  A point LEFT of the tangent (+y) is expected to travel +x;
            % a point right of it (-y) is expected to travel -x.
            spec = struct('centers', [0 0; 100 0], 'width', 6, 'name', 'straight');
            rm = RoadModel(spec, 'res', 0.25);

            [c, s, defined] = rm.expectedDirection(50, 1.5);
            tc.verifyTrue(defined);
            tc.verifyEqual(c, 1, 'AbsTol', 1e-3);
            tc.verifyEqual(s, 0, 'AbsTol', 1e-3);

            [c, s, defined] = rm.expectedDirection(50, -1.5);
            tc.verifyTrue(defined);
            tc.verifyEqual(c, -1, 'AbsTol', 1e-3);
            tc.verifyEqual(s, 0, 'AbsTol', 1e-3);
        end

        function oneWaySegmentHasSingleDirection(tc)
            spec = struct('centers', [0 0; 100 0], 'width', 6, ...
                'name', 'ramp', 'twoWay', false);
            rm = RoadModel(spec, 'res', 0.25);

            [c1, ~, ~] = rm.expectedDirection(50,  1.5);
            [c2, ~, ~] = rm.expectedDirection(50, -1.5);
            tc.verifyEqual(c1, 1, 'AbsTol', 1e-3);
            tc.verifyEqual(c2, 1, 'AbsTol', 1e-3, ...
                'A one-way ribbon must have the same direction on both sides.');
        end

        function junctionDirectionIsUndefined(tc)
            % No single travel direction applies inside a junction box, so the
            % wrong-way detector must be told to stay out of it.
            spec = struct('centers', [0 0; 20 0], 'width', 7, ...
                'name', 'box', 'directionDefined', false);
            rm = RoadModel(spec, 'res', 0.25);

            [~, ~, defined] = rm.expectedDirection(10, 0);
            tc.verifyFalse(defined);
        end

        function pointAtRoundTrip(tc)
            % pointAt and nearest must be inverses to within the raster cell.
            spec = struct('centers', [0 0; 40 10; 80 0; 120 20], 'width', 6, ...
                'name', 'curvy');
            rm = RoadModel(spec, 'res', 0.25);

            L = rm.segmentLength(1);
            for sQ = [5, L/3, L/2, 0.8*L]
                for dQ = [-2, 0, 1.5]
                    xy = rm.pointAt(1, sQ, dQ);
                    info = rm.nearest(xy(1), xy(2));
                    tc.verifyEqual(info.s, sQ, 'AbsTol', 0.6, ...
                        sprintf('arc length round trip at s=%.1f d=%.1f', sQ, dQ));
                    tc.verifyEqual(info.d, dQ, 'AbsTol', 0.4, ...
                        sprintf('lateral round trip at s=%.1f d=%.1f', sQ, dQ));
                end
            end
        end

        function curvedRoadFollowsItsCentreline(tc)
            % Points generated on the centreline of a curved road must all be
            % drivable, and points well outside the width must not be.
            spec = struct('centers', [0 0; 40 10; 80 0; 120 20], 'width', 6, ...
                'name', 'curvy');
            rm = RoadModel(spec, 'res', 0.25);

            L = rm.segmentLength(1);
            sQ = linspace(1, L - 1, 60)';
            onCentre = rm.pointAt(1, sQ, 0);
            tc.verifyTrue(all(rm.isDrivable(onCentre(:,1), onCentre(:,2))), ...
                'Every centreline point must be drivable.');

            farOut = rm.pointAt(1, sQ, 8);
            tc.verifyFalse(any(rm.isDrivable(farOut(:,1), farOut(:,2))), ...
                'Points 8 m off a 6 m road must not be drivable.');
        end

        function directionFollowsCurveTangent(tc)
            % On a curved road the expected direction must track the tangent,
            % not stay fixed at the initial heading.
            spec = struct('centers', [0 0; 40 40; 80 80], 'width', 6, 'name', 'diag');
            rm = RoadModel(spec, 'res', 0.25);

            xy = rm.pointAt(1, rm.segmentLength(1)/2, 1.5);
            [c, s, defined] = rm.expectedDirection(xy(1), xy(2));
            tc.verifyTrue(defined);
            % A 45 degree road: both components equal and positive.
            tc.verifyEqual(atan2(s, c), pi/4, 'AbsTol', 0.15);
        end

        function unevenEdgesAreSeededAndReproducible(tc)
            mk = @(seed) RoadModel(struct('centers', [0 0; 150 0], 'width', 6, ...
                'name', 'rough', 'edgeNoise', 0.35), ...
                'res', 0.25, 'rs', RandStream('mrg32k3a', 'Seed', seed));

            a = mk(11);
            b = mk(11);
            c = mk(12);

            tc.verifyEqual(a.segments(1).halfWidthL, b.segments(1).halfWidthL, ...
                'Same seed must reproduce the same edges.');
            tc.verifyNotEqual(a.segments(1).halfWidthL, c.segments(1).halfWidthL, ...
                'A different seed must produce different edges.');

            % Uneven, but never pinched shut.
            tc.verifyGreaterThan(min(a.segments(1).halfWidthL), 0.9);
            tc.verifyGreaterThan(std(a.segments(1).halfWidthL), 0.01, ...
                'edgeNoise > 0 must actually vary the width.');
        end

        function twoSegmentsUnionCorrectly(tc)
            % A crossroads: both arms drivable, and their union covers the
            % centre.  This is the mechanism Scenario 2 is built on.
            specs = [struct('centers', [-50 0; 50 0], 'width', 7, 'name', 'ew', ...
                        'twoWay', true, 'directionDefined', true, 'edgeNoise', 0)
                     struct('centers', [0 -50; 0 50], 'width', 7, 'name', 'ns', ...
                        'twoWay', true, 'directionDefined', true, 'edgeNoise', 0)];
            rm = RoadModel(specs, 'res', 0.25);

            tc.verifyTrue(rm.isDrivable(-40, 0));   % east-west arm
            tc.verifyTrue(rm.isDrivable(0, -40));   % north-south arm
            tc.verifyTrue(rm.isDrivable(0, 0));     % the centre
            tc.verifyFalse(rm.isDrivable(30, 30));  % the quadrant between arms
        end

        function rasterQueriesAreVectorised(tc)
            % The risk map queries tens of thousands of points per cycle, so
            % the array form must work and must agree with scalar calls.
            spec = struct('centers', [0 0; 100 0], 'width', 6, 'name', 'straight');
            rm = RoadModel(spec, 'res', 0.25);

            x = [10 20 30 40]';
            y = [0 2 -2 5]';
            tf = rm.isDrivable(x, y);
            tc.verifyEqual(size(tf), size(x));
            for k = 1:numel(x)
                tc.verifyEqual(tf(k), rm.isDrivable(x(k), y(k)));
            end
        end

        function queriesOutsideRasterAreSafe(tc)
            spec = struct('centers', [0 0; 100 0], 'width', 6, 'name', 'straight');
            rm = RoadModel(spec, 'res', 0.25);

            tc.verifyFalse(rm.isDrivable(1e4, 1e4));
            d = rm.distanceToEdge(1e4, 1e4);
            tc.verifyLessThan(d, 0, 'Far-away points must report negative edge distance.');
            tc.verifyTrue(isfinite(d), 'Edge distance must stay finite for downstream maths.');
        end
    end
end
