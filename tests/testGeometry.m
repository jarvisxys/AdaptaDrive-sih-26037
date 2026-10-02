classdef testGeometry < matlab.unittest.TestCase
    %TESTGEOMETRY  OBB overlap and distance against hand-computed cases.
    %
    %   These primitives decide whether a run is scored as a collision and
    %   what "minimum clearance" means, so every expected value below is
    %   worked out by hand in the comment, not copied from the code's output.

    properties (Constant)
        Tol = 1e-9;
    end

    methods (Test)

        % ---------------- corners ----------------

        function cornersAxisAligned(tc)
            % 4 x 2 box at the origin, no rotation.
            % Corners: FL(2,1) RL(-2,1) RR(-2,-1) FR(2,-1)
            C = obbCorners(makeOBB(0, 0, 0, 4, 2));
            tc.verifyEqual(C, [2 1; -2 1; -2 -1; 2 -1], 'AbsTol', tc.Tol);
        end

        function cornersRotated90(tc)
            % Same box turned 90 deg: forward is +y, left is -x.
            % FL(-1,2) RL(-1,-2) RR(1,-2) FR(1,2)
            C = obbCorners(makeOBB(0, 0, pi/2, 4, 2));
            tc.verifyEqual(C, [-1 2; -1 -2; 1 -2; 1 2], 'AbsTol', 1e-12);
        end

        function cornersAreCounterClockwise(tc)
            % Shoelace area must be positive (CCW) and equal L*W.
            o = makeOBB(3, -2, 0.7, 4.5, 1.8);
            C = obbCorners(o);
            x = C(:, 1); y = C(:, 2);
            signedArea = 0.5 * sum(x .* circshift(y, -1) - circshift(x, -1) .* y);
            tc.verifyGreaterThan(signedArea, 0);
            tc.verifyEqual(signedArea, o.L * o.W, 'AbsTol', 1e-9);
        end

        % ---------------- overlap ----------------

        function overlapIdenticalBoxes(tc)
            a = makeOBB(0, 0, 0, 4, 2);
            tc.verifyTrue(obbOverlap(a, a));
        end

        function overlapClearlySeparated(tc)
            % Two 4x2 boxes 10 m apart along x: gap is 10 - 2 - 2 = 6 m.
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(10, 0, 0, 4, 2);
            tc.verifyFalse(obbOverlap(a, b));
        end

        function overlapExactlyTouching(tc)
            % Half-lengths 2 and 2, centres 4 apart -> faces touch exactly.
            % Touching counts as overlapping (conservative for a safety check).
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(4, 0, 0, 4, 2);
            tc.verifyTrue(obbOverlap(a, b));
        end

        function overlapJustClear(tc)
            % Centres 4.002 apart -> a 2 mm gap -> must NOT overlap.
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(4.002, 0, 0, 4, 2);
            tc.verifyFalse(obbOverlap(a, b));
        end

        function overlapRotatedCross(tc)
            % A long box across a perpendicular long box, both through the
            % origin: they must intersect.
            a = makeOBB(0, 0, 0,    6, 1);
            b = makeOBB(0, 0, pi/2, 6, 1);
            tc.verifyTrue(obbOverlap(a, b));
        end

        function overlapNeedsRotatedAxis(tc)
            % Diagonal case that axis-aligned bounding boxes would call a hit.
            % Both boxes are thin and rotated 45 deg; their AABBs overlap but
            % the boxes themselves are separated along their own normals.
            a = makeOBB(0,   0,   pi/4, 6, 0.4);
            b = makeOBB(2.0, 2.0 + 1.0, pi/4, 6, 0.4);
            tc.verifyFalse(obbOverlap(a, b), ...
                'Separating axis is a box normal, not a world axis.');
        end

        function overlapIsSymmetric(tc)
            a = makeOBB(1, 2, 0.4, 4.5, 1.8);
            b = makeOBB(3, 3, -1.1, 2.0, 1.0);
            tc.verifyEqual(obbOverlap(a, b), obbOverlap(b, a));
        end

        % ---------------- distance ----------------

        function distanceOverlappingIsZero(tc)
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(1, 0, 0, 4, 2);
            tc.verifyEqual(obbDistance(a, b), 0);
        end

        function distanceFaceToFace(tc)
            % Centres 10 apart on x, half-lengths 2 each -> gap 6.
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(10, 0, 0, 4, 2);
            tc.verifyEqual(obbDistance(a, b), 6, 'AbsTol', tc.Tol);
        end

        function distanceSideBySide(tc)
            % Centres 5 apart on y, half-widths 1 each -> gap 3.
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(0, 5, 0, 4, 2);
            tc.verifyEqual(obbDistance(a, b), 3, 'AbsTol', tc.Tol);
        end

        function distanceCornerToCorner(tc)
            % a has a corner at (2,1); b is centred at (6,5) with half-sizes
            % (2,1) so its nearest corner is (4,4).  Gap = hypot(2,3).
            a = makeOBB(0, 0, 0, 4, 2);
            b = makeOBB(6, 5, 0, 4, 2);
            tc.verifyEqual(obbDistance(a, b), hypot(2, 3), 'AbsTol', tc.Tol);
        end

        function distanceIsSymmetric(tc)
            a = makeOBB(0, 0, 0.3, 4.5, 1.8);
            b = makeOBB(9, 4, -0.8, 2.6, 1.4);
            tc.verifyEqual(obbDistance(a, b), obbDistance(b, a), 'AbsTol', tc.Tol);
        end

        function distanceAgreesWithOverlapFlag(tc)
            % The two routines must never disagree: distance 0 exactly when
            % the boxes overlap.  Swept over a range of separations.
            a = makeOBB(0, 0, 0, 4.5, 1.8);
            for gap = [-1, -0.1, 0, 0.001, 0.5, 3]
                b = makeOBB(4.5 + gap, 0, 0, 4.5, 1.8);
                d = obbDistance(a, b);
                if obbOverlap(a, b)
                    tc.verifyEqual(d, 0, sprintf('gap=%g overlapped but d=%g', gap, d));
                else
                    tc.verifyGreaterThan(d, 0, sprintf('gap=%g clear but d=0', gap));
                end
            end
        end

        function distanceRotatedBoxKnownGap(tc)
            % b is rotated 45 deg and centred on the x axis.  Its half-diagonal
            % along x is (L/2)*cos45 + (W/2)*sin45 = (2+1)*sqrt(2)/2.
            % a's face is at x = 2.  Nearest feature of b is its corner.
            L = 4; W = 2;
            a = makeOBB(0, 0, 0, L, W);
            cx = 12;
            b = makeOBB(cx, 0, pi/4, L, W);
            reach = (L/2 + W/2) * sqrt(2)/2;
            tc.verifyEqual(obbDistance(a, b), cx - reach - L/2, 'AbsTol', 1e-9);
        end

        function distanceParallelOffsetSegments(tc)
            % Regression guard for the segment-to-segment helper: two long
            % boxes side by side, offset along their length so the nearest
            % features are an endpoint and an edge interior.
            a = makeOBB(0,  0, 0, 10, 1);
            b = makeOBB(20, 4, 0, 10, 1);
            % a ends at x=5 (y in [-0.5,0.5]); b starts at x=15 (y in [3.5,4.5]).
            % Nearest: corner (5,0.5) to corner (15,3.5) -> hypot(10,3).
            tc.verifyEqual(obbDistance(a, b), hypot(10, 3), 'AbsTol', tc.Tol);
        end
    end
end
