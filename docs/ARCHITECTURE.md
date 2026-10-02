# AdaptaDrive — architecture and requirement traceability

Team SECOND INNINGS · SIH 2026 · PS 26037 (MathWorks, Smart Vehicles)

Everything here describes a **simulation**. Architectural coverage is not
validated performance — the team research document says so in §35.1, and this
document is written to keep that distinction visible.

---

## 1. The closed loop

No part of the ego's motion is scripted. The vehicle state is produced only by
`KinematicBicycle` integrating commands the controllers actually issued.

```
                 ┌──────────────────────────────────────────────┐
                 │                                              │
                 ▼                                              │
  Sensors ──▶ Tracking ──▶ Prediction ──▶ Risk map ──▶ Behaviour│
  (camera     (fusion,     (multi-modal,  (B1: one    (7-state  │
   + 2 radar)  class vote)  class-cond.)   field)      FSM)     │
                                │                        │      │
                                ▼                        ▼      │
                          Detectors              Global planner  │
                          (A2 wrong-way,         (Hybrid A*      │
                           A3 merge)              → RRT*)        │
                                                        │       │
                                                        ▼       │
                                                  Local planner  │
                                                  (dynamic       │
                                                   window)       │
                                                        │       │
                                                        ▼       │
                                             Control ──▶ Vehicle ┘
                                    (pure pursuit +   (kinematic
                                     jerk-limited PI)  bicycle, RK4)
```

**Rates** — `cfg.sim.dt = 0.05 s` for control, vehicle and agents;
`cfg.sim.planRate = 10 Hz` for sensing, tracking, prediction, detectors, risk,
behaviour and the local planner. Global replanning is periodic per context plus
event-triggered.

**Step order** (`SimEngine.run`) and why it matters:

1. snapshot the world at time *t* and log it
2. test termination against that snapshot
3. run the planning cycle from that snapshot (timed)
4. apply control, integrate the vehicle
5. advance the agents

So log entry *k* is one consistent instant, and the planner never sees a world
that has already moved in response to it.

---

## 2. Requirement traceability

| Req | What it asks for | Implemented in | Evidenced by |
|---|---|---|---|
| **A1** | Road hazards as planning inputs | `src/world/HazardSet.m`, `RiskMap.staticLayer` | `hazardTraversals`, `minPotholeClearance` in `runs.csv`; `testRiskMap/potholeRaisesStaticLayer` |
| **A2** | Wrong-way vehicle detection | `src/detect/WrongWayDetector.m` | `wrongWayFlags`, event log entries with time-to-flag; `tests/testWrongWay.m` (10 tests) |
| **A3** | Informal merge detection | `src/detect/MergeDetector.m` | `mergeDetections`, TTC at trigger in the event log; `tests/testMerge.m` (9 tests) |
| **B1** | Unified dynamic risk map | `src/risk/RiskMap.m` | `results/figures/*_risk.png`; `tests/testRiskMap.m` (15 tests) |
| **B2** | Uncertainty-shaped risk | `src/predict/Predictor.m` + covariance growth | `testRiskMap/higherUncertaintyGivesWiderFootprint` |
| **B3** | Class-specific risk weighting | `src/config/classPriors.m` | `testRiskMap/cowAndAutoAtEqualDistanceDifferInRisk` |
| **B4** | Context-adaptive parameters | `src/config/contextParams.m` | `testFSM/contextChangesTheThresholds`; per-context rows in `summary.csv` |
| **B5** | Baselines, ablations, stress | `configPreset.m` (3 baselines, 5 ablations, `STRESS-sensor`), `run_experiments.m`, `run_stress.m`, `ScenarioUnseen.m` | `summary.csv` (90 runs incl. all five ablations), `results/stress/stress_summary.csv`, `results/figures/ablations.png` |
| Latency | < 200 ms replanning | timed end-to-end and **per stage** in `SimEngine.proposedCycle` | `latencyP50/P95/Max` and `stage_*_p50/p95` in `runs.csv`; `latency.png`, `stages.png` |

### A1 — potholes are a cost, not a wall

The most consequential design decision in A1: a pothole is expensive to drive
over, not impossible. `HazardSet.blockingSeverity` deliberately **excludes**
potholes, so they never enter the global planner's hard constraint; only
`HazardSet.severity` (which includes them) feeds the risk field the local
planner integrates.

Treating them as walls was tried and measured: a pothole's above-threshold
region plus the planner's inflation is a ~2.15 m no-go radius, and two of them
close a 6 m carriageway completely. The global planner then failed on almost
every cycle and the ego crawled 65 m in 106 s.

### A2 — where the detector abstains

`WrongWayDetector` stays silent in two places, both deliberate:

- **junctions**, where `RoadModel` reports the expected-direction field as
  undefined — no single direction applies, and a detector that guessed one
  would flag every turning vehicle;
- **near the centreline**, where the direction field flips sign. Tracked
  positions carry over a metre of error, so a lawful oncoming vehicle can be
  placed on the far side by noise. Measured before this guard: 2 false
  wrong-way flags on a scenario containing no wrong-way vehicle.

### B1 — one field, many causes

`R = clip(w_s·S + w_e·E + w_ws·W + Σ_i w_c(class)·m_ww·Σ_k Σ_t γ̂^t·p_k·N(...), 0, 1)`

A pothole, a wrong-way auto and an uncertain pedestrian all become the same
quantity, so the planner needs one cost rather than a rule per hazard type.
Layers stay separately addressable for the UI toggles and the ablations.

Two departures from the literal formula, both forced by measurement and both
recorded in `DEVIATIONS.md`:

- **D22** — kernels are peak-normalised, not probability densities. A density
  peaks *inversely* with uncertainty, so after clipping, more uncertainty
  would have meant *less* risk — the opposite of B2.
- **D23** — the time-discount weights are normalised to sum to 1. The literal
  `Σ γ^t` is 9.58, so a single agent peaked at 4.8–9.6 and everything within a
  few metres of anything clipped to exactly 1. A saturated field silently
  degrades the unified risk map into the binary grid it exists to improve on.

---

## 3. Deliberate asymmetries

These materially affect the results and are stated so the comparison cannot be
read as rigged.

**BL1 is given the road centreline.** The lane-follow baseline receives the
reference path directly (`SimEngine.laneFollowPlan`). The proposed planner never
sees it — it plans over free space against the risk field. This is *generous to
the baseline*: it hands BL1 exactly the map structure the problem statement says
is unavailable on these roads.

**Pedestrians, carts and cows do not yield to a moving ego.** They are the
hazard the planner exists to handle; making them polite would quietly convert
every avoidance test into a test of someone else's caution. Vehicle classes do
brake for what is directly ahead, which is ordinary traffic behaviour.

The one exception, added after measurement: a pedestrian or cow **already
mid-crossing will stop short of a vehicle that is already stationary**
(`AgentModel.egoIsBlockingCrossing`). Walking into the flank of a stopped car
is not hazard modelling — it is the pedestrian causing a collision the planner
had already avoided.

**Duplicate tracks are merged, but only when unambiguous.** This reverses an
earlier decision in this document, and the reasoning that was wrong is worth
keeping. The argument for leaving duplicates alone was that a duplicate inflates
an object's risk, which is conservative, while a bad merge deletes a road user
who is genuinely there. The first half of that is false. A duplicate is not
conservative, because every copy is predicted forward *independently* and every
copy is painted into the risk map: six confirmed tracks on one 0.9 m/s pushcart
produced a wall of predicted occupancy that made a 6 m road impassable, and the
ego yielded to a crowd that was not there (D31).

The second half is true, and it sets the threshold. Merging distinct objects is
worse than keeping a duplicate — it invents one obstacle and loses two — so the
test is only ever "closer together than the sensors can distinguish": a single
tight 1.0 m radius, not a class-scaled one. A class-scaled radius reaches 1.8 m
for two cars, and on the highway that fused tracks belonging to different
vehicles and took position recall to 0.48.

Clusters are **fused** in information form rather than thinned to a survivor,
and the fused covariance is inflated by the spread of the members, so merging
never claims more certainty than the inputs had. `duplicateTrackRate` is still
reported.

**Camera class comes from an explicit classifier model.** Measured on R2026a,
`visionDetectionGenerator` returns `ObjectClassID = 0` for every detection, and
`drivingScenario` only accepts six actor classes — so an auto-rickshaw is
stored as a Car and a cow as a Pedestrian (D11, D20). The seven-class labels
B3 needs cannot come from the toolbox. `cameraClassModel` supplies them:
association comes from the simulator, the **class** passes through a confusion
and abstention model, and position/velocity always come from the noisy sensor.

---

## 4. The binary-check caveat

`plannerHybridAStar` takes a **binary** validity check. The global planner
therefore cannot tell risk 0.61 from risk 0.99 — both are simply blocked — and
cannot prefer the cheaper of two passable routes.

The continuous risk field enters through the **local** planner, which
integrates `R` along every rollout. The division of labour is deliberate:
Hybrid A* supplies a kinematically feasible corridor; the dynamic-window
planner chooses where within and around it to actually drive.

This is stated because a reader could otherwise assume the risk map grades the
global search. It does not.

---

## 4a. Who decides to stop

The layering is: the **FSM** decides *whether* the vehicle proceeds, the **local
planner** decides *how*. That boundary was not actually enforced, and two
deadlocks came out of the gap. Both fixes are stated here because they change
the contract between layers, not just a constant.

**The planner may not veto a decision to go.** Standing still occupies one cell,
so on clear ground its risk term is exactly 0 while every moving rollout scores
above 0. For any positive risk weight there is a risk level at which `v = 0` is
the cheapest candidate — and because the ego then does not move, the same
comparison holds forever. Measured: village seed 1 held position 30 m from the
goal for 77 s, `v = 0` at cost 0.3668 against 0.3853 for the cheapest moving
candidate, until the run timed out.

`speedCap = 0` is how the FSM says "stay put", so a non-zero cap is an explicit
decision to move. When the cap is non-zero and a *feasible* moving trajectory
exists, the planner takes the cheapest moving one. It never overrides a hard
constraint, it arms only after 3 s of standing still — waiting for a cow to cross
is not a deadlock — and it latches until the vehicle is genuinely rolling (D32).

**Rejection governs entering a violating state, not leaving one.** When the ego's
own footprint is already at or above `rejectRisk`, every candidate is rejected
*including holding position*. "Everything was rejected" is then not information
about where the ego may go; it is information about where the ego already is, and
refusing to move preserves the violation instead of ending it. Measured: village
seed 1 sat in STOP with 77 of 77 candidates rejected for 122 s.

Leaving therefore needs its own rule: the rejected candidate with the lowest peak
risk, ordered by peak then *mean* then speed, capped at 1.0 m/s, and only when
strictly better than holding. The mean-risk tie-break is what makes it work — deep
off-road every peak saturates at 1.0, so peak alone gives no gradient (D33).

**What is still missing.** The dynamic window contains no reverse speeds, so a
vehicle that has driven into a pocket with no forward-reachable lower-risk cell
cannot recover. A real system would reverse. This is a known gap, not a solved
problem.

---

## 5. Module map

| Path | Role |
|---|---|
| `startup.m`, `check_env.m` | path setup; three-probe toolbox detection and backend selection |
| `src/config/` | `defaultConfig`, `configPreset` (baselines/ablations), `classPriors` (B3), `contextParams` (B4) |
| `src/world/` | `RoadModel` (drivable area + expected-direction field), `HazardSet` (A1), `AgentModel`, `Scenario1..5` |
| `src/sensing/` | `SensorSuite` (toolbox + synthetic backends), `SyntheticSensor`, `cameraClassModel` |
| `src/tracking/` | `TrackerWrapper` (multiObjectTracker + class voting), `SimpleKFTracker` (fallback), `initAdaptaDriveCV` |
| `src/predict/` | `Predictor` (B2/B3, multi-modal) |
| `src/risk/` | `RiskMap` (B1) |
| `src/detect/` | `WrongWayDetector` (A2), `MergeDetector` (A3) |
| `src/behavior/` | `BehaviorFSM` (runtime), `BehaviorState`, `buildStateflowChart` (display artefact) |
| `src/plan/` | `GlobalPlanner` (Hybrid A* → RRT*), `DWAPlanner` |
| `src/control/` | `PurePursuit`, `SpeedController`, `KinematicBicycle` |
| `src/metrics/` | `MetricsLogger`, OBB geometry, `capsuleGap`, `ttc` |
| `src/sim/` | `SimEngine` (the loop), `SimLog` |
| `src/ui/` | `theme`, `BEVRenderer` |
| `AdaptaDriveApp.m` | the presentation UI |

---

## 6. Graceful degradation

Every toolbox-dependent component sits behind an interface with a pure-MATLAB
fallback, selected automatically by `check_env`:

| Component | Preferred | Fallback |
|---|---|---|
| Scenario container | `drivingScenario` | native (AgentModel alone) |
| Sensors | `visionDetectionGenerator` + `drivingRadarDataGenerator` | `SyntheticSensor` |
| Tracker | `multiObjectTracker` | `SimpleKFTracker` (CV Kalman + GNN via `matchpairs`) |
| Global planner | `plannerHybridAStar` | `plannerRRTStar` → grid A* |
| Pure pursuit | `controllerPurePursuit` | native |
| Behaviour chart | Stateflow | none — parity test reports **SKIPPED**, not passed |

The synthetic sensor backend does **not** model occlusion, false alarms,
range-dependent noise growth or radar resolution cells. A run made with it is
therefore an easier perception problem, and `check_env` records which backend
produced every result so the two are never averaged together.

---

## 7. Honesty mechanisms in the code

- `MetricsLogger.blank()` initialises every metric to **NaN**, never 0. A zero
  reads as a good score; NaN prints as "not measured" and does not average.
- `run_tests` counts **passed / FAILED / SKIPPED** as three separate numbers.
- `run_experiments` records a run that errored with outcome `"error"` and NaN
  metrics — never drops it, because silently dropping failures is how a
  completion rate becomes a lie.
- Wilson 95% intervals accompany every completion rate, because N is small.
- Every figure and CSV row carries the measured CPU, MATLAB release and
  timestamp. No latency number is ever labelled with hardware it was not
  measured on.
- `tests/testNoHardcodedResults.m` scans the UI for numeric literals assigned
  to metric fields and checks the Results tab renders honestly without a CSV.
