# AdaptaDrive — build status

What is working, what is not, and the evidence for each. Read this before
quoting any number from this project.

Every claim here is backed by a command in this repository that can be re-run.
Numbers come from `results/runs.csv` (90 runs) and `results/summary.csv`.

---

## One-line summary

A complete closed-loop prototype with a working UI, five scenarios plus a
held-out variant, nine configurations, 158 passing tests and a measured results
pipeline.

On the 90-run matrix **PROPOSED beats all three baselines, at 60% against 40%,
40% and 50%**, and it is the only configuration that completes both village and
intersection. But it is *not* the best configuration measured: the
`-classpriors` ablation — PROPOSED with class-specific risk weighting removed —
reaches **70%**. So the proposed system is the best of the four configurations
the comparison was designed around, and one of its own components is making it
worse.

One scenario (market) fails for every configuration, one seed of village stalls,
and cattle is the one scenario where a baseline still wins. All of it is
characterised below rather than hidden.

## Headline numbers (measured, 90 runs, seeds 1–2)

Collision-free completion. Two seeds per cell, so every figure carries a Wilson
interval spanning roughly 9–91% — **nothing here is statistically significant**,
and the table is a description of what happened, not a proof.

| scenario | PROPOSED | BL1 lane-follow | BL2 reactive-CV | BL3 static-only |
|---|---|---|---|---|
| village | **50%** | 0% | 50% | 0% |
| intersection | **100%** | 0% | 0% | **100%** |
| highway | **100%** | 100% | 100% | 100% |
| market | 0% | 0% | 0% | 0% |
| cattle | 50% | **100%** | 50% | 50% |
| **overall** | **60%** | 40% | 40% | 50% |

**What can be said.** PROPOSED is the only configuration that finishes both
village and intersection. It is never worse than every baseline on any scenario
except cattle, where BL1 completes both seeds and PROPOSED one. With two seeds
that difference is one run.

**What changed from the previous matrix.** All four configurations used to tie
at 30%, and highway was a defeat: BL1 completed it twice while PROPOSED stalled
at 380 m of 392 both times. Highway is now 100% for everything, because the
cause was a perception bug rather than a planner one (D29). Intersection went
from 50% to 100%.

**Where PROPOSED is still beaten.** Cattle: BL1 100% against PROPOSED 50%.
PROPOSED collides on seed 2 at 107 m. BL1 has no risk map, no prediction and no
local planner — it follows the centreline and brakes for what is in front of it,
which on a straight road with one crossing cow is enough. That is worth stating
plainly: on the scenario built specifically to need prediction, the baseline
that has none does better on these two seeds.

## Ablations (measured, first time this build)

Each row is PROPOSED with exactly one component removed. Overall completion
across all five scenarios:

| configuration | overall | vs PROPOSED |
|---|---|---|
| PROPOSED | 60% | — |
| `-classpriors` | **70%** | **+10** |
| `-context` | 60% | 0 |
| `-riskmap` | 50% | −10 |
| `-wrongside` | 40% | −20 |
| `-uncertainty` | **30%** | **−30** |

**Uncertainty (B2) is the most valuable component in the system** — removing it
halves completion. The wrong-side term (D25, added as a soft cost) is worth 20
points, and the graded risk map (B1) 10.

**Class priors (B3) make things worse, not better.** Removing them *improves*
completion by 10 points overall and by 50 points on village (100% against 50%).
This is reported because it is what the runs show. The likely mechanism is that
per-class risk weighting inflates the footprint of the classes the village is
full of — pedestrians and carts both carry weight 1.00 and 0.75 on a 6 m road —
so the planner treats a passable gap as impassable. It is evidence against a
design decision this project argued for, and it belongs in the presentation
rather than in a footnote.

**Context parameters (B4) change nothing measurable** at this sample size.

## Stress conditions (B5, measured for the first time this build)

`cfg.stress` had existed since early in the build and **nothing read it** — the
configuration advertised a capability that did not exist. It is wired up now, and
the three conditions are each a single change from nominal so the result is
attributable. Reported separately, never pooled into the headline rate.

| condition | intersection | highway | cattle |
|---|---|---|---|
| nominal | **100%** | **100%** | 50% |
| density ×2 | **0%** | **100%** | 50% |
| sensor stress (20% dropout, 0.8 m σ) | **0%** | **100%** | 50% |

**The intersection result is fragile and the highway result is not.** Intersection
completes both seeds nominally and neither seed under *either* stress. Highway
holds at 100% through both, and cattle is unmoved at 50%.

That distinction is the most useful thing in this table. Intersection's 100% is
the headline improvement of the last day, and it turns out to depend on
conditions being nominal: double the agents, or degrade the camera, and it fails
completely. Highway's 100% survives both, so it is the one result here that can
be called robust rather than merely achieved.

Market is excluded because every configuration already scores 0% on it, and
village because it is by far the slowest scenario (~165 s per run against ~40 s
for cattle). Village is the omission most worth adding back given more machine
time.

### The held-out variant

The market plus a wrong-way bus and a crossing cow (`ScenarioUnseen.m`). Nothing
was tuned on it, and the two added agents draw from their own random substream so
every market agent stays exactly where the market puts it.

| config | completion | min clearance |
|---|---|---|
| PROPOSED | 0% | 0.00 m |
| BL1 | 0% | 0.01 m |

**Both fail.** This is the market's own failure inherited — a 10.5 m bus on a
5.5 m corridor cannot be passed, and the ego was already colliding within 5 m on
the market itself. The variant therefore tests nothing the market does not
already fail, which makes it a poor held-out case rather than a hard one. A
held-out variant built on a *passable* scenario would have been informative; this
one only confirms that an impassable scenario stays impassable when made harder.
Stated plainly because the run count alone would suggest the held-out test was
meaningful.

## Latency

Measured in isolation, median of **3 repeats** per scenario with the p95 range.
The run is deterministic — the same path every time — so the spread is
measurement noise, not different driving:

| scenario | p50 | p95 | p95 range | meets 200 ms? |
|---|---|---|---|---|
| village | 45 | **146** | 145–154 | ✅ |
| cattle | 42 | **98** | 96–98 | ✅ |
| highway | 103 | **179** | 177–180 | ✅ |
| market | 208 | 278 | 272–280 | ❌ |
| intersection | 70 | 314 | 310–319 | ❌ |

**3 of 5 meet the target**, up from 2 of 5, and no verdict is borderline — none
of the ranges crosses 200 ms. Highway is the one that changed, from p95 320 ms
to 179 ms, as a side effect of the D29 phantom-track fix.

The repeats are not decoration. A single pass had put highway at 178 ms and then
217 ms, either side of the target, and intersection at 313 ms then 401 ms. Three
clean back-to-back repeats give spreads of 1–3%, so those earlier swings were a
contaminated pass rather than real variance — but a verdict taken from one pass
would have been noise presented as a result.

### Where the time goes

Per-stage medians, PROPOSED, during the batch:

| scenario | sense | track | risk | global | dwa | total |
|---|---|---|---|---|---|---|
| village | 25.1 | **60.6** | 5.5 | 3.0 | 6.6 | 102.2 |
| intersection | 22.9 | **38.0** | 6.1 | 4.0 | 12.3 | 84.9 |
| highway | 21.4 | **46.0** | 6.7 | 0.0 | 9.6 | 85.9 |
| market | 40.0 | **99.3** | 12.4 | 32.2 | 16.4 | 202.8 |
| cattle | 12.9 | 14.9 | 4.8 | 0.0 | 7.3 | 41.2 |

**Tracking dominates every scenario**, at 36–49% of the cycle. The local
planner, which absorbed most of the earlier optimisation effort, costs 7–16 ms.
This table is the single most useful diagnostic added on the last day: it is what
redirected the search from the planner to perception, and every fix below
followed from it. `results/figures/stages.png` is the figure.

Batch figures are higher than isolated ones because this machine has 7.7 GB of
RAM and under 1 GB free while a matrix runs. Both are in `RESULTS.md`; the
isolated ones are the honest measure of the planner.

---

## Per-scenario outcome (PROPOSED, both seeds)

| Scenario | seed 1 | seed 2 |
|---|---|---|
| **village** | ✅ goal, 243.2 m in 99.3 s | ❌ timeout at 81.8 m |
| **intersection** | ✅ goal, 100.8 m in 31.6 s | ✅ goal, 101.1 m in 29.0 s |
| **highway** | ✅ goal, 382.6 m in 45.8 s | ✅ goal, 382.8 m in 48.8 s |
| **market** | ❌ collision at 4.9 m | ❌ collision at 5.6 m |
| **cattle** | ✅ goal, 285.9 m in 56.5 s | ❌ collision at 107.0 m |

```matlab
cfg = configPreset('PROPOSED','scenario','highway','seed',1);
s   = buildScenario('highway',1,1.0,cfg);
r   = SimEngine(s,cfg).run()
```

---

## What still fails

### 1. Market is unpassable for every configuration

**0% for all nine configurations on both seeds — 18 collisions out of 18 runs.**
PROPOSED, all three baselines and all five ablations fail in the same place,
within about 5 m of the start.

When no configuration can pass, the scenario is what is wrong, not the planner.
As built it has a ~5.5 m corridor, 14 agents, 8 stalls and crossing pedestrians
on a 150 m street: there is no gap for a 1.8 m vehicle to be in. It is a fair
*picture* of an Indian market and a fair stress case, but it is not currently a
test any planner can pass, and no completion figure from it means anything.

The failure mode is worth recording: the ego stops correctly for a crossing
pedestrian and is then walked into. Two attempted fixes made it worse, both
instructive:

- widening the pedestrian's yield radius to 10 m created a **mutual deadlock** —
  ego waits for pedestrian, pedestrian waits for ego — collapsing village from
  244 m to 39 m;
- moving the ego to a 1.75 m keep-left offset put its body inside the stalls.

The ego needs somewhere to *go*, not just somewhere to stop. Either the corridor
widens or the planner needs a "creep to a gap" behaviour.

### 2. Village seed 2 stalls at 81.8 m of 244 m

Seed 1 completes the same scenario. The difference is measurable and points at
perception, not planning: on seed 2 the tracking stage costs **156 ms median
against 59 ms on seed 1**, which means many more tracks. Duplicate-track fusion
(D31) reduced this a great deal but has not eliminated it on every seed.

**Where to look:** dump the track list at the stall and check how many tracks
sit on each truth agent, the way D31 did for seed 1. If the count is again 3–6
per object, the merge radius is too tight for whatever the seed-2 geometry does
to the sensors.

### 3. Cattle seed 2 collides at 107 m, and BL1 does not

PROPOSED hits the cow on seed 2; BL1 completes both seeds. This is the one place
a baseline still beats the proposed system, and it is the scenario designed to
require prediction, so it is the most uncomfortable result in the set. It needs
the collision replayed frame by frame before anything is claimed about why.

### 4. Latency misses 200 ms on market and intersection

Market 278 ms p95, intersection 314 ms p95, both measured in isolation over 3
repeats with spreads under 3%, so neither is a marginal call.

Market's cycle is 199 ms median: tracking 114 ms, the global planner 8.5 ms, and
it is the only scenario where the global planner costs anything much, because its
replanning period is the shortest of the five (0.25 s). Intersection's p95 is
high relative to its 70 ms median, which is the signature of occasional
expensive cycles rather than a uniformly slow loop — the junction produces bursts
of tracks when several approaches are occupied at once.

Both point at the same place as everything else: **tracking, not planning.**


---

## What was fixed on the last day, and how it was found

The four fixes below came from one change: per-stage cycle timing. Until then
latency was a single end-to-end number, which said *that* a cycle missed the
200 ms budget and nothing about *which stage*. The split was measured, and it
pointed somewhere nobody had been looking.

This is the **first** measurement, taken *before* any of the four fixes. The
post-fix figures are in the Latency section above, and highway is the row to
compare: 208.8 ms here against 85.9 ms after.

| scenario | sense | track | risk | dwa | total |
|---|---|---|---|---|---|
| village | 16.6 | 15.1 | 5.3 | 11.3 | 49.9 |
| intersection | 29.1 | **62.5** | 7.5 | 18.6 | 128.5 |
| highway | 25.0 | **135.2** | 10.4 | 33.7 | 208.8 |
| market | 38.5 | **87.0** | 12.9 | 12.9 | 161.2 |
| cattle | 13.3 | 18.5 | 5.5 | 19.5 | 58.7 |

Median ms per stage, PROPOSED, isolated. **Tracking was 49–65% of every cycle
that missed the budget.** The local planner — which had absorbed most of the
earlier optimisation effort — was 11–34 ms throughout and was never the problem.

Tracker cost scales with the number of tracks, so the question became why the
highway carried 29.6 tracks for 5 agents. That led to all four fixes:

**1. Departed agents never left perception (D29).** `AgentModel` deactivates an
agent at the end of the modelled stretch, and every consumer of agent truth
respected that — the collision check, the clearance sweep, the metrics, the
renderer — except `syncContainer`, which is the one path feeding the toolbox
sensors. A departed agent stayed parked in the `drivingScenario` actor list at
its last pose, still reporting 7–16 m/s, and the sensors kept detecting it.
Four highway agents pile up within a few metres of the ribbon end, two pairs at
identical coordinates, and became a wall of ~30 phantom tracks 16 m in front of
the ego. **That was the highway stall** — reported for days as an unexplained
local-planner defect.

**2. Duplicate tracks were fused (D31).** Six confirmed tracks on one pushcart.
Every copy is predicted forward and painted into the risk map independently, so
six copies of a 0.9 m/s cart made a 6 m road impassable and the ego yielded to a
crowd that was not there. Two obvious survivor rules were tried and both failed
in opposite directions — keeping the oldest track kept a drifted one (highway
recall 0.55), keeping the most confident made track ids churn and degraded
control. Information-form fusion gives both, and the merge radius is a tight
1.0 m because merging two *real* vehicles is worse than keeping a duplicate.

**3. Standing still was cheaper than driving (D32).** A stopped vehicle occupies
one cell, so on clear ground its risk term is exactly 0 while every moving
rollout scores above 0. For *any* positive risk weight there is a risk level at
which `v = 0` wins — and since the ego then does not move, the same comparison
holds forever. Measured on village: `v = 0` at cost 0.3668 against 0.3853 for
the cheapest moving candidate, held for 77 s, 30 m from the goal.

This is the fifth instance of this family after the four bounded-term defects,
and the first that is not a scaling error: every term was correctly in [0,1].
The fix is therefore architectural, not a re-weighting. The FSM decides *whether*
to proceed and says so with `speedCap`; the planner decides *how* and may not
veto that on cost. It arms only after 3 s — waiting for a cow to cross is not a
deadlock — and latches until the vehicle is actually rolling.

**4. There was no way out of a violating cell (D33).** When the ego's own
footprint is already above `rejectRisk`, every candidate is rejected *including
holding position*, and returning BLOCKED preserves the violation instead of
ending it. Village sat in STOP with 77 of 77 candidates rejected for 122 s.
Rejection governs *entering* a bad state; leaving one needs its own rule.

---

## Soft-constraint settings in this build

These change the vehicle's behaviour. They never change how it is scored —
every metric is measured from the run that actually happened.

| Setting | Value | Spec | Why |
|---|---|---|---|
| `plan.rejectRisk` | 0.95 | 0.85 | lets the planner use ground it would otherwise refuse on narrow roads |
| `sim.maxTimeMult` | 5.0 | 3.0 | the timeout detects deadlock; a vehicle moving steadily at 97% of the route is not deadlocked |
| `risk.wWrongSide` | 0.25 | (added) | discourages the oncoming half without forbidding overtakes |
| `plan.dwaSafeGap` | 1.6 m | — | vehicles meeting on a narrow road pass with ~1.3 m; the term must act while there is still room |
| `plan.stallBreakTime` | 3.0 s | (added) | how long the planner may sit still while the FSM asks it to proceed (D32) |
| `plan.escapeSpeed` | 1.0 m/s | (added) | walking pace, for leaving a cell the ego should not be on (D33) |
| `tracking.mergeMinGap` | 1.0 m | (added) | below this two tracks are one object (D31) |

---

## Known gaps

- **The local planner cannot reverse.** The dynamic window holds no negative
  speeds. The escape rule (D33) can only go forwards, so a vehicle in a pocket
  with no forward-reachable lower-risk cell still cannot recover.
- **Seed discipline was violated.** The freeze rule says tune on seeds 101–110
  and report on 1–10. Tuning during this build used seed 1, a reported seed.
  Re-running does not undo it, so it is stated rather than omitted. Every
  parameter choice above is also justified by a mechanism, not only by an
  outcome, which is the partial mitigation.
- **Scenario 1's oncoming car does not vary its spawn position** (D34) — only
  its speed. The squeeze geometry therefore varies less across seeds than the
  design intended. Found while the reported matrix was running and deliberately
  not changed, so the numbers remain the ones the shipped code produced.
- **Ghost tracks were never implemented.** A deleted confirmed track was to be
  retained with growing covariance for 3 s (5 s for pedestrian and cow) and
  still written into the risk map, with `PATH_CLEAR` requiring positive
  evidence. The motivating failure — the intersection collision with a vehicle
  that was never tracked — was resolved by D29 and D31 instead, so this was not
  built. It remains the right mechanism for the general case.
- **M8 (RoadRunner)** — not implemented. RoadRunner is not installed; only the
  MATLAB-side import API is present, which says nothing about the application.
- **M9 (learned prediction)** — not possible here. Deep Learning Toolbox is
  absent from this machine (and its licence still tests `true`, which is D3).

---

## How to re-check any of this

```matlab
startup
check_env                                  % backends actually in use
run_tests                                  % full suite, pass/FAIL/SKIPPED
run_demo('cattle','PROPOSED',1)            % a single run end to end
run_experiments('seeds',1:2,'ablations',true)
run_stress                                 % the B5 stress conditions
measure_latency                            % isolated + per-stage
finalize                                   % every artefact from the CSVs
AdaptaDriveApp                             % the UI
```
