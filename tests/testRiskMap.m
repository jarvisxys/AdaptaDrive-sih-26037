classdef testRiskMap < matlab.unittest.TestCase
    %TESTRISKMAP  B1 integration, plus the B2 and B3 acceptance criteria.
    %
    %   B2 acceptance: a pedestrian with higher positional uncertainty must
    %                  produce a WIDER risk footprint than one with low
    %                  uncertainty.
    %   B3 acceptance: a cow and an auto-rickshaw at equal distance must
    %                  produce DIFFERENT risk, because their class weights
    %                  differ.
    %
    %   Both are the difference between "we have a risk map" and "our risk map
    %   encodes what the problem statement asks it to encode".

    properties
        cfg
        road
        hazards
        ego
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.cfg = defaultConfig();
            tc.road = RoadModel(struct('centers', [-50 0; 250 0], 'width', 8, ...
                'name', 'straight'), 'res', 0.25);
            tc.hazards = HazardSet();
            tc.ego = struct('x', 0, 'y', 0, 'yaw', 0, 'v', 8, 'a', 0, ...
                'steer', 0, 't', 1.0);
        end
    end

    methods (Test)

        % ---------------- basic field properties ----------------

        function singleAgentDoesNotSaturateTheField(tc)
            % Guards the normalisation of the discount weights.  With the
            % literal sum of gamma^t (9.58) one agent peaked at 4-5 and every
            % cell near it clipped to 1, which erased B1, B2 and B3 from the
            % planner's view.  One agent must stay within its class weight.
            for classId = [1 2 5 7]
                peak = tc.peakRiskForClass(classId);
                prior = classPriors(classId);
                tc.verifyLessThanOrEqual(peak, prior.riskWeight * 1.05, sprintf( ...
                    ['Class %d peaks at %.2f, above its weight %.2f: the ' ...
                     'discount weights are not normalised and the field saturates.'], ...
                    classId, peak, prior.riskWeight));
                tc.verifyGreaterThan(peak, 0.1, ...
                    'A stationary agent must still produce meaningful risk.');
            end
        end

        function overlappingAgentsMaySaturate(tc)
            % Saturation is correct when it is earned: two road users in the
            % same place really is maximum risk.
            trA = tc.classTrack(5, 30, 0, 0, 0);
            trB = tc.classTrack(5, 30.3, 0, 0, 0);
            trB.trackId = 2;
            tracks = [trA, trB];
            p = Predictor(tc.cfg, tc.road);
            rm = tc.buildMap();
            rm.update(tc.ego, p.predict(tracks), tracks);
            tc.verifyGreaterThan(max(rm.layers.dynamic(:)), ...
                max(tc.peakRiskForClass(5), 0), ...
                'Two overlapping agents must exceed one.');
        end

        function fieldIsBoundedAndRightShape(tc)
            rm = tc.buildMap();
            rm.update(tc.ego, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());

            r = tc.cfg.risk;
            tc.verifyEqual(size(rm.R), ...
                [round(2*r.lateralM/r.res), round((r.aheadM+r.behindM)/r.res)]);
            tc.verifyTrue(all(rm.R(:) >= 0 & rm.R(:) <= 1), ...
                'Risk must be clipped to [0,1].');
        end

        function layersAreSeparatelyAddressable(tc)
            % The UI toggles and the ablations both need the layers apart.
            rm = tc.buildMap();
            rm.update(tc.ego, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            for f = ["static", "dynamic", "edge"]
                tc.verifyTrue(isfield(rm.layers, f));
                tc.verifyEqual(size(rm.layers.(f)), size(rm.R));
            end
        end

        function offRoadIsMaximumEdgeRisk(tc)
            rm = tc.buildMap();
            rm.update(tc.ego, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            % 10 m to the left of an 8 m road is off the carriageway.
            v = rm.sampleEgo(20, 10);
            tc.verifyEqual(v, 1, 'AbsTol', 1e-9, 'Off-road must be maximum risk.');
        end

        function outsideTheMapIsMaximumRisk(tc)
            % The planner must not find free space by leaving the field.
            rm = tc.buildMap();
            rm.update(tc.ego, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());
            tc.verifyEqual(rm.sampleEgo(500, 0), 1);
            tc.verifyEqual(rm.sampleEgo(-500, 0), 1);
        end

        function potholeRaisesStaticLayer(tc)
            % A1: surface hazards must appear in the field.
            tc.hazards.addPothole(30, 0, 0.7, 0.5, 0, 0.1);
            rm = tc.buildMap();
            rm.update(tc.ego, Predictor.emptyPredictions(), TrackerWrapper.emptyTracks());

            onPothole = rm.sampleEgo(30, 0);
            clearRoad = rm.sampleEgo(50, 0);
            tc.verifyGreaterThan(onPothole, clearRoad, ...
                'A pothole must raise the risk above clear road.');
            tc.verifyGreaterThan(onPothole, 0.5);
        end

        % ---------------- B2 acceptance ----------------

        function higherUncertaintyGivesWiderFootprint(tc)
            % B2: two pedestrians, identical except for track covariance.
            areaLow  = tc.footprintArea(tc.pedTrack(0.2));
            areaHigh = tc.footprintArea(tc.pedTrack(3.0));

            tc.verifyGreaterThan(areaHigh, areaLow * 1.3, sprintf( ...
                ['B2 FAILED: an uncertain pedestrian must occupy more of the ' ...
                 'risk field than a well-localised one (%.1f vs %.1f cells).'], ...
                areaHigh, areaLow));
        end

        function uncertaintyGrowsAlongTheHorizon(tc)
            % Sigma(t) = P + Q*t must actually grow, or B2 is decorative.
            p = Predictor(tc.cfg, tc.road);
            pred = p.predict(tc.pedTrack(0.5));
            S = pred(1).modes(1).Sigma;
            first = det(S(:,:,1));
            last  = det(S(:,:,end));
            tc.verifyGreaterThan(last, first * 2, ...
                'Predicted covariance must grow over the horizon.');
        end

        % ---------------- B3 acceptance ----------------

        function cowAndAutoAtEqualDistanceDifferInRisk(tc)
            % B3: same position, same speed, different class.
            peakCow  = tc.peakRiskForClass(7);   % cow  w_c = 1.00
            peakAuto = tc.peakRiskForClass(3);   % auto w_c = 0.65

            tc.verifyNotEqual(peakCow, peakAuto, ...
                'B3 FAILED: class must change the risk an agent contributes.');
            tc.verifyGreaterThan(peakCow, peakAuto, sprintf( ...
                ['B3 FAILED: the cow carries the higher vulnerability weight ' ...
                 'and must produce more risk than the auto (%.3f vs %.3f).'], ...
                peakCow, peakAuto));
        end

        function pedestrianOutweighsBus(tc)
            % The ordering that matters most for an Indian road.
            tc.verifyGreaterThan(tc.peakRiskForClass(5), tc.peakRiskForClass(2), ...
                'A pedestrian must carry more risk weight than a bus.');
        end

        function busFootprintIsLargerThanPedestrian(tc)
            % Weight is not the only channel: the footprint enters the
            % covariance, so a bus occupies more of the map even though it
            % carries a lower weight.
            areaBus = tc.footprintArea(tc.classTrack(2, 30, 0, 8, 0), 0.05);
            areaPed = tc.footprintArea(tc.classTrack(5, 30, 0, 8, 0), 0.05);
            tc.verifyGreaterThan(areaBus, areaPed, ...
                'A 10.5 m bus must cover more cells than a 0.6 m pedestrian.');
        end

        % ---------------- A2 coupling ----------------

        function wrongWayAgentRaisesRisk(tc)
            % A2 must ELEVATE risk, not merely log an event.
            tr = tc.classTrack(1, 30, 0, 8, 0);
            p = Predictor(tc.cfg, tc.road);
            preds = p.predict(tr);

            % Read the pre-clip dynamic layer: R is clipped to [0,1], and a
            % multiplier applied to something already at 1 is invisible there.
            rmA = tc.buildMap();
            rmA.update(tc.ego, preds, tr, []);
            plain = max(rmA.layers.dynamic(:));

            rmB = tc.buildMap();
            rmB.update(tc.ego, preds, tr, tr.trackId);
            flagged = max(rmB.layers.dynamic(:));

            tc.verifyGreaterThan(flagged, plain, ...
                'A wrong-way flag must raise that agent''s risk contribution.');
        end

        % ---------------- ablations ----------------

        function binaryModeCollapsesTheField(tc)
            % B1 ablation: the same geometry, but only two values.
            cfgB = tc.cfg;
            cfgB.riskMode = 'binary';
            tr = tc.classTrack(5, 25, 1, 1.2, 0);
            p = Predictor(cfgB, tc.road);
            rm = RiskMap(cfgB, tc.road, tc.hazards);
            rm.update(tc.ego, p.predict(tr), tr);

            vals = unique(rm.R(:));
            tc.verifyLessThanOrEqual(numel(vals), 2, ...
                'Binary risk mode must produce at most two distinct values.');
            tc.verifyTrue(all(ismember(vals, [0 1])));
        end

        function classPriorsAblationRemovesClassDistinction(tc)
            % With cfg.classPriors off, a cow and an auto must look the same.
            cfgA = tc.cfg;
            cfgA.classPriors = false;
            peakCow  = tc.peakRiskForClass(7, cfgA);
            peakAuto = tc.peakRiskForClass(3, cfgA);
            tc.verifyEqual(peakCow, peakAuto, 'RelTol', 0.02, ...
                'The classPriors ablation must erase class-specific weighting.');
        end
    end

    % ====================================================================
    methods (Access = private)

        function rm = buildMap(tc, cfg)
            if nargin < 2, cfg = tc.cfg; end
            rm = RiskMap(cfg, tc.road, tc.hazards);
        end

        function tr = pedTrack(~, posVar)
            tr = struct('trackId', 1, 'classId', 5, 'x', 30, 'y', 1, ...
                'vx', 0.2, 'vy', 1.2, 'speed', 1.22, 'heading', atan2(1.2, 0.2), ...
                'P', eye(2) * posVar, 'age', 1, 'updates', 10, 'confirmed', true);
        end

        function tr = classTrack(~, classId, x, y, vx, vy)
            tr = struct('trackId', 1, 'classId', classId, 'x', x, 'y', y, ...
                'vx', vx, 'vy', vy, 'speed', hypot(vx, vy), ...
                'heading', atan2(vy, vx), 'P', eye(2) * 0.3, ...
                'age', 1, 'updates', 10, 'confirmed', true);
        end

        function a = footprintArea(tc, tr, thresh)
            if nargin < 3, thresh = 0.1; end
            cfgL = tc.cfg;
            p = Predictor(cfgL, tc.road);
            rm = RiskMap(cfgL, tc.road, tc.hazards);
            rm.update(tc.ego, p.predict(tr), tr);
            a = sum(rm.layers.dynamic(:) > thresh);
        end

        function peak = peakRiskForClass(tc, classId, cfgIn)
            %PEAKRISKFORCLASS  Peak contribution of one agent, by class.
            %
            %   The agent is STATIONARY on purpose.  A moving agent's
            %   predicted kernels spread along its path, and how much they
            %   overlap depends on its speed and footprint - so a peak taken
            %   from a moving agent mixes the class WEIGHT with those two
            %   effects and cannot isolate B3.  With zero velocity every class
            %   gets identical kernel overlap, and the peak reflects w_c.
            %
            %   Read from layers.dynamic, which is pre-clip: the composed
            %   field R is clipped to [0,1] and two agents that both saturate
            %   would compare equal there.
            if nargin < 3, cfgIn = tc.cfg; end
            tr = tc.classTrack(classId, 30, 0, 0, 0);
            p = Predictor(cfgIn, tc.road);
            rm = RiskMap(cfgIn, tc.road, tc.hazards);
            rm.update(tc.ego, p.predict(tr), tr);
            peak = max(rm.layers.dynamic(:));
        end
    end
end
