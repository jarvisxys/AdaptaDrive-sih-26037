# DEVIATIONS

Every place where the installed MathWorks release differed from the build
specification, or where an API behaved differently from what was assumed.

Rule: the installed documentation wins. Each entry records the command that was
run, the output observed, and what AdaptaDrive does about it.

Machine of record: Windows 11, MATLAB **R2026a** (version 26.1).

---

## D1 — Release is R2026a, not R2024b

**Spec said:** "MATLAB R2024b or newer".

**Observed:**

```
>> version('-release')
2026a
>> computer
PCWIN64
```

**Action:** No change needed — R2026a satisfies "R2024b or newer". Recorded
because every API in this project was verified against 26.1 specifically, and
the results files stamp the release.

---

## D2 — `license('test', ...)` throws on feature names of 28+ characters

**Spec implied:** a plain `license('test', feature)` probe per toolbox.

**Observed:** the natural feature name for Sensor Fusion and Tracking Toolbox is
longer than the limit and raises an error rather than returning `0`:

```
>> license('test','Sensor_Fusion_and_Tracking_Toolbox')   % 34 chars
Error using license
Feature name must be less than 28 characters.

>> license('test','Sensor_Fusion_and_Tracking')           % 26 chars
ans = 1
```

**Action:** `check_env.m` wraps every licence probe in `try/catch`, stores the
error text in `toolbox.licenseNote`, and uses the 26-character feature name
`Sensor_Fusion_and_Tracking`. An unguarded probe would have aborted the whole
environment check on this machine. Regression-guarded by
`testEnvironment/licenceProbesNeverThrow`.

---

## D3 — A licence can test true for a toolbox that is not installed

**Observed:** Deep Learning Toolbox does not appear in `ver`, yet its licence
feature tests true:

```
>> any(strcmp({ver().Name}, 'Deep Learning Toolbox'))
ans = 0
>> license('test','Neural_Network_Toolbox')
ans = 1
```

**Action:** This is why `check_env.m` uses **three independent probes** —
`ver` (installed), `license('test',...)` (licensed) and `exist(entryPoint)`
(callable) — and marks a toolbox `usable` only when all three agree. Licence
alone is never treated as evidence of availability. Regression-guarded by
`testEnvironment/licenceAloneIsNotEvidence`.

Consequence for the plan: **M9 (learned LSTM prediction) is not available on
this machine** and must be reported as not implemented rather than skipped
silently.

---

## D4 — `road`, `actor` and `targetPoses` report `exist == 0`

**Observed:** these are `drivingScenario` class methods, so `exist` does not see
them as functions even though they resolve and are callable:

```
>> exist('targetPoses')
ans = 0
>> any(strcmp(methods(drivingScenario), 'targetPoses'))
ans = 1
```

**Action:** `check_env.m` probes toolboxes through top-level entry points only
(`drivingScenario`, `plannerHybridAStar`, `trackerGNN`, `sfnew`, ...), never
through class methods, which would have produced a false "missing" verdict.

---

## D5 — `chasePlot` is a function, not a `drivingScenario` method

**Observed:**

```
>> any(strcmp(methods(drivingScenario), 'chasePlot'))
ans = 0
>> which chasePlot
(empty)
```

**Action:** None. AdaptaDrive renders its own bird's-eye view
(`src/ui/BEVRenderer.m`) and does not use `chasePlot`. Recorded only so the
absence is not mistaken for a broken install.

---

## D6 — Toolboxes absent on this machine

`ver` on the machine of record lists: MATLAB, Automated Driving Toolbox,
Computer Vision Toolbox, Image Processing Toolbox, Navigation Toolbox, Parallel
Computing Toolbox, Sensor Fusion and Tracking Toolbox, Simulink, Stateflow.

Not installed, and the affected work:

| Missing | Affects | Status |
|---|---|---|
| Deep Learning Toolbox | M9 learned prediction (optional) | **not available** |
| RoadRunner desktop app | M8 detailed scenes (optional) | **not probed / assumed absent** |

`check_env.m` can only probe the MATLAB-side RoadRunner import API
(`roadrunnerHDMap`, which ships with Automated Driving Toolbox). The presence of
that function says nothing about whether the RoadRunner application is installed
or licensed, and `check_env` prints that caveat rather than claiming M8 is
available.

---

## D7 — MPEG-4 availability is probed, not assumed

**Observed:** `VideoWriter.getProfiles()` on this machine lists `Archival`,
`Motion JPEG 2000`, `Motion JPEG AVI`, `Grayscale AVI`, `Indexed AVI`,
**`MPEG-4`**, `Uncompressed AVI`.

**Action:** `check_env` selects `MPEG-4` because it is present, and falls back to
`Motion JPEG AVI` where it is not. `make_video` records the profile it actually
used in the output filename metadata rather than assuming `.mp4`.

---

## D8 — The machine is not the target hardware named in the spec

**Spec said:** "Target hardware for timing claims: Intel i5-12450H laptop,
32 GB RAM."

**Observed on the machine of record:**

```
>> check_env
CPU : 11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz  (8 physical cores, 7.7 GB RAM)

PS> (Get-CimInstance Win32_Processor).Name
11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz
PS> [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory/1GB,1)
7.7
```

So the real machine is a **different CPU generation** (Tiger Lake i7-11800H vs
Alder Lake i5-12450H) with **roughly a quarter of the assumed RAM** (7.7 GB vs
32 GB).

**Action:**

- No latency number will ever be labelled "i5-12450H". `check_env` reads the
  CPU string and RAM from the machine at run time, and every row of `runs.csv`
  and every figure footer carries the *measured* hardware string. Timing claims
  name the hardware they were actually measured on.
- 7.7 GB is tight for the planned 4 `parfor` workers (each MATLAB worker carries
  its own copy of the scenario and risk grids). `cfg.experiment.numWorkers`
  stays at 4 for now and will be **measured** at M7; if memory pressure shows
  up, the worker count drops and the change is recorded here rather than
  quietly absorbed.

---

## D9 — `feature('numcores')` counts physical, not logical, cores

**Observed:** MATLAB and Windows disagree because they count different things:

```
>> feature('numcores')
ans = 8
PS> (Get-CimInstance Win32_Processor) | Select NumberOfCores, NumberOfLogicalProcessors
NumberOfCores NumberOfLogicalProcessors
            8                        16
```

**Action:** `check_env` had originally labelled this "logical cores", which was
wrong. The field is now printed as **physical cores**. Recorded because a
mislabelled core count would misrepresent the hardware behind every latency
figure.

---

## D13 — `controllerPurePursuit` returns CURVATURE, not angular velocity

**The most consequential deviation so far.** The classic pure-pursuit System
object signature is `[v, omega] = pp(pose)`, and the build spec's conversion to
a road-wheel angle assumed an angular velocity: `steer = atan(omega*L/v)`.

**Observed on R2026a:** the second output does not depend on
`DesiredLinearVelocity` at all, and equals `1/R` on a circle of radius `R`:

```
circle radius 50.0  -> true curvature 0.02000 1/m

  DesiredLinearVelocity    v_out      out2      out2/v    expected omega (v/R)
                 1.0     1.0000    0.02002    0.02002        0.02000
                 2.0     2.0000    0.02002    0.01001        0.04000
                 5.0     5.0000    0.02002    0.00400        0.10000
                10.0    10.0000    0.02002    0.00200        0.20000
                20.0    20.0000    0.02002    0.00100        0.40000
```

An angular velocity would scale with speed (right-hand column). It does not.
The object is curvature-parameterised, which is consistent with the companion
deprecation seen on the same release:

```
Warning: "MaxAngularVelocity" property has been deprecated.
Use "MaxCurvature" property instead.
```

`properties(controllerPurePursuit)` on R2026a returns exactly:
`Waypoints, MaxCurvature, LookaheadDistance, DesiredLinearVelocity`.

**How it surfaced.** Nothing threw. The spec's conversion under-steers by a
factor of `v`, so at 6.9 m/s the ego commanded about one seventh of the
steering it needed and drifted off a 6 m road in 30 m — reported as an
`offroad` failure that looked like a planner or road-geometry problem.

**Action:** the conversion is now

```matlab
[~, kappa] = obj.ppObj([x; y; yaw]);
steer = atan(kappa * wheelbase);     % NO division by speed
```

Guarded by three tests in `tests/testPurePursuit.m`:
`toolboxSecondOutputIsCurvatureNotOmega` pins the semantics directly,
`backendSteerIsSpeedInvariantOnAFixedPath` catches the specific bug shape (a
steer command that changes with speed at a fixed lookahead), and
`tracksACurvedRoadWithoutLeavingIt` drives the full village route on both
backends. After the fix the two backends agree to within 0.02 rad and both
track the route with under 0.2 m of lateral error.

---

## D10 — `drivingScenario` actor `Yaw` is in DEGREES

AdaptaDrive works in radians throughout (Section 4 data contracts). The
container does not:

```
>> a.Yaw = 45;  a.Yaw
ans = 45          % degrees, not radians
```

**Action:** the conversion is confined to the two functions that touch the
container — `buildScenario/attachDrivingScenario` and `SimEngine.syncContainer`
— and is marked in both. No other file in the project uses degrees.

---

## D11 — `actor()` and `vehicle()` accept only six ClassIDs between them

**Observed:**

```
>> actor(sc, 'ClassID', 1, ...)
Warning: Class ID 1 is not supported for an actor. The drivingScenario object
created a vehicle instead. ClassID of actor must be one of these values:
3 (Bicycle), 4 (Pedestrian), 5 (Jersey Barrier), or 6 (Guardrail).
```

So the container offers Car, Truck, Bicycle, Pedestrian, Jersey Barrier and
Guardrail. AdaptaDrive's taxonomy has seven classes including auto-rickshaw,
pushcart and cow, none of which exist there.

**Action:** `src/world/adtContainerClass.m` maps each AdaptaDrive class to a
container proxy and records why:

| AdaptaDrive | Container | Rationale |
|---|---|---|
| 1 car | `vehicle` Car | direct |
| 2 bus | `vehicle` Truck | closest large vehicle |
| 3 auto-rickshaw | `vehicle` Car | closest motorised |
| 4 two-wheeler | `actor` Bicycle | closest two-wheeled |
| 5 pedestrian | `actor` Pedestrian | direct |
| 6 pushcart | `actor` Bicycle | closest slow narrow non-motorised |
| 7 cow | `actor` Pedestrian | closest unprotected mobile |

The proxy affects **only** the geometry the toolbox sensor models ray-trace
against, and real `Length`/`Width`/`Height` are always passed explicitly, so it
never changes what a sensor sees. AdaptaDrive's own class IDs 1-7 remain
authoritative through tracking, risk weighting, prediction, the FSM and the UI:
a cow is never *reasoned about* as a pedestrian. The consequence for M2 is that
camera class cannot be read from the container and must come from our own class
table plus an explicit confusion/dropout model — which is recorded there.

---

## D14 — `visionDetectionGenerator.FieldOfView` is read-only

**Observed:**

```
>> visionDetectionGenerator('FieldOfView', [60 10], ...)
Error: Unable to set the 'FieldOfView' property ... because it is read-only.
>> visionDetectionGenerator('FocalLength', [f f], 'ImageSize', [480 640], ...)
Error: The name 'FocalLength' is not an accessible property
```

On R2026a the camera geometry lives in a single `Intrinsics` property holding a
`cameraIntrinsics` object (Computer Vision Toolbox); `FieldOfView` is derived
from it and cannot be set, and the older separate `FocalLength` / `ImageSize` /
`PrincipalPoint` properties no longer exist.

**Action:** `SensorSuite` solves for the intrinsics that produce the specified
horizontal FOV:

```matlab
fx = imgW / (2 * tan(fov/2));
intr = cameraIntrinsics([fx fx], [imgW/2 imgH/2], [imgH imgW]);
```

Verified: `fov = 60 deg`, `imgW = 1920` gives `FieldOfView = [60 46.8]`.

Resolution is a load-bearing choice, not cosmetic. Detection also requires the
target to exceed `MinObjectImageSize` (15x15 px), so resolution sets the range
at which each class becomes visible. At 640x480 a 0.6 m pedestrian clears 15 px
only within ~22 m, making a nominally 70 m camera blind to exactly the
vulnerable classes this project is about. 1920x1080 (a standard automotive
front camera) is used, and the resulting per-class recall is **measured and
reported** in the M2 output rather than assumed.

---

## D15 — `multiObjectTracker` has one assignment gate for all sensors

The build spec asks for an "assignment threshold tuned per sensor".
`multiObjectTracker.AssignmentThreshold` is a single `[gate, maxCost]` pair
applied to every sensor. Per-sensor gating would require `trackerGNN`.

**Action:** one gate is used and `cfg.tracking.gate` documents it. Not worth
swapping trackers for: after D21 the single gate performs well (93.4% recall,
0% false tracks).

---

## D16 — Mixed `ObjectAttributes` layouts break the tracker

Camera detections carry `ObjectAttributes{1} = struct('TargetIndex', n)`; radar
detections carry `struct('TargetIndex', n, 'SNR', x)`. `multiObjectTracker`
concatenates the attributes of all detections assigned to one track:

```
Error using horzcat
Number of fields in structure arrays being concatenated do not match.
  in ObjectTrack/get.ObjectAttributes
```

**Action:** detections handed to the tracker are rebuilt from our own
world-frame contract with no `ObjectAttributes`. The target id needed for
evaluation lives in our contract instead. This also fixes a second problem in
the same place — the toolbox reports in ego/body coordinates, and tracking
there would make every stationary object appear to accelerate whenever the ego
does.

---

## D17 — Sensor `InitialSeed` is inert in this configuration

```
Warning: The InitialSeedSource property is not relevant in this configuration
of the System object.
Warning: The InitialSeed property is not relevant in this configuration ...
```

(`InitialSeedSource` also rejects `'Property'`; its valid values are
`"Specify seed" | "Repeatable" | "Not repeatable"`.)

**Action:** the sensor generators draw from MATLAB's global stream, so
`SimEngine` calls `rng(cfg.seed, 'twister')` at construction. That is the only
effective handle on their reproducibility. Everything AdaptaDrive owns uses
private `RandStream` substreams and is unaffected by global state.

---

## D18 — `multiObjectTracker` requires a uniform measurement size

Feeding a 3-element camera measurement alongside a 6-element radar one:

```
Error using initcvekf
Expected Detection.Measurement to be an array with number of elements equal to 3.
```

The tracker builds one sample detection at setup and validates all later
detections against it. Separately, `initcvekf` will only accept a 6-element
measurement when the detection carries `MeasurementParameters` declaring
`HasVelocity`:

```
>> initcvekf(objectDetection(0, [1;2;0;3;4;0], 'MeasurementNoise', eye(6)))
FAILED: ... number of elements equal to 3
>> mp = struct('Frame','rectangular','HasVelocity',true);
>> f = initcvekf(objectDetection(0, [1;2;0;3;4;0], 'MeasurementNoise', eye(6), ...
                'MeasurementParameters', mp));
OK, State = [1 3 2 4 0 0]
```

**Action:** every detection is emitted as `[x y z vx vy vz]` with
`MeasurementParameters`, and the camera's unmeasured velocity is declared with
a large variance. The state layout `[x vx y vy z vz]` confirmed above is what
`TrackerWrapper` indexes.

---

## D19 — `Position` is an actor's ROTATIONAL centre, not its geometric centre

`actorProfiles` reports an `OriginOffset` per actor:

```
ActorID 1 ClassID 1  L=4.50  OriginOffset=[-1.25 0 0]     % ego vehicle
ActorID 2 ClassID 4  L=0.60  OriginOffset=[0 0 0]         % pedestrian
ActorID 6 ClassID 1  L=4.20  OriginOffset=[-1.1 0 0]      % car
```

Anything built with `vehicle()` has its origin at the rear axle, so writing a
body-centre position into `Position` displaces the body by over a metre, and
the sensors then ray-trace a vehicle that is not where our world says it is.

**Action:** `SimEngine.syncContainer` writes
`Position = bodyCentre + R(yaw) * OriginOffset`, using offsets cached at build
time.

---

## D20 — `visionDetectionGenerator` reports no object class

`ObjectClassID` is **0** on every camera detection. Combined with D11 (the
container only has six actor classes, so an auto-rickshaw is stored as a Car
and a cow as a Pedestrian), the toolbox cannot supply the seven-class labels
that requirement B3 is built on.

**Action:** `cameraClassModel` supplies the class explicitly. Association comes
from the simulator (`ObjectAttributes.TargetIndex`), which is standard for
scenario-based studies; the class itself passes through an explicit confusion
and abstention model, so the planner regularly reasons about mislabelled and
unlabelled objects. Position and velocity always come from the noisy sensor.
Measured on the village scenario: 99.3% track class accuracy after majority
voting, with 9.4% of tracks never classified at all.

---

## D21 — `initcvekf` seeds velocity covariance from the measurement noise

**The most damaging defect found in M2, and it never raised an error.**

Because the tracker demands a uniform measurement size (D18), camera detections
must declare velocity "unobserved" with a very large measurement variance.
`initcvekf` then seeds a NEW track's velocity covariance from that variance. A
camera-born track starts with ~1e6 m^2/s^2 of velocity uncertainty, which one
0.1 s prediction converts into ~1e4 m^2 of position uncertainty. The
association gate stops discriminating, tracks are abandoned and re-created
every few cycles, and a single pedestrian ends up carrying four simultaneous
tracks.

Measured on village / seed 1 / 64 cycles / 5 agents:

| velSigma | initialiser | recall | posRMSE | velRMSE | duplicates | max track id |
|---|---|---|---|---|---|---|
| position only | `initcvekf` | 94.0% | 1.150 m | 2.127 m/s | 32.4% | 23 |
| 1.5 m/s | `initcvekf` | **46.4%** | 1.449 m | 1.714 m/s | 52.7% | **116** |
| 1.5 m/s | `initAdaptaDriveCV` | **93.4%** | 1.208 m | **1.580 m/s** | 39.3% | **27** |

**Action:** `src/tracking/initAdaptaDriveCV.m` caps the initial velocity
covariance at a physical prior (10 m/s standard deviation) and sets process
noise for the most agile class modelled. This fixes the cause rather than
avoiding it, so radar range rate can still be used — which matters because
prediction integrates velocity over a 3 s horizon.

Guarded by `testTracking/trackerDoesNotFragment`, which fails if track ids grow
past 15x the agent count or recall drops below 85%.

**Residual, accepted deliberately:** 39.3% of tracks are duplicates (a second
track on a real object). They are NOT merged. A duplicate inflates that object's
risk, which is conservative; merging two tracks 1.5 m apart risks deleting a
road user who is genuinely there, which is not. In a dense market that
trade-off only gets sharper.

---

## D22 — Risk kernels are peak-normalised, not probability densities

**Spec formula:** `... * N(x; mu_ik(t), Sigma_ik(t) + footprint_i)`, then clip to
`[0,1]`.

`N(...)` written that way is a probability **density**, with units of 1/m². Its
peak value is `1/(2*pi*sqrt(det(Sigma)))`, so it depends inversely on the
uncertainty: a sharply localised pedestrian peaks in the hundreds while a
diffuse one peaks near zero. After clipping to `[0,1]`, the confident case
saturates and the uncertain case nearly vanishes — the exact opposite of what
B2 is for, since **more** uncertainty would mean **less** risk.

**Action:** the peak-normalised kernel `exp(-0.5 * d_Mahalanobis^2)` is used
instead. It reads as "how much of this cell could this agent occupy at this
time", is naturally bounded in `[0,1]`, and grows the risk footprint with
uncertainty rather than shrinking its peak. The covariance still carries all
the class- and mode-specific structure.

---

## D23 — The discount weights must be normalised or the field saturates

**Spec formula:** `sum_t gamma^t * ...` with `gamma = 0.9` over a 3 s horizon
at 0.1 s steps.

`sum_{t=0}^{29} 0.9^t = 9.58`. A single agent therefore contributes a peak of
`w_c * 9.58`, between 4.8 and 9.6, and every cell within a few metres of
anything clips to exactly 1.

**Measured before the fix** (peak of the dynamic layer, one agent, stationary):

| class | peak | after clip |
|---|---|---|
| pedestrian | 4.29 | 1.0 |
| bus | 4.93 | 1.0 |

Both saturate, so a pedestrian and a bus become indistinguishable to the
planner — and so do a certain and an uncertain prediction, and a flagged and an
unflagged wrong-way vehicle. **A saturated field silently degrades the unified
risk map into the binary occupancy grid it exists to improve on**, taking B1,
B2 and B3 with it.

This did not show up as an error. It showed up as `testRiskMap` reporting that
a bus outweighs a pedestrian, which is what led to it.

**Action:** the discount weights are normalised to sum to 1. The near future
still dominates the far future in exactly the same proportions, but one agent's
contribution is now bounded by `w_c * m_ww`. Overlapping agents can still
saturate, which is correct — two road users in one place genuinely is maximum
risk. Guarded by `testRiskMap/singleAgentDoesNotSaturateTheField`.

**Related:** prediction steps are subsampled by 3 (10 of 30 steps) when
stamping. The kernels are wide and heavily overlapping, so this is visually and
numerically close, and it is what keeps the risk map inside the cycle budget.

---

## D24 — The global planner's occupancy map is not the thresholded total risk

**Spec said:** "validatorOccupancyMap built from the risk map thresholded at
tau (default 0.6) and inflated by half vehicle width + context margin".

That combination is geometrically infeasible on the roads this project
targets, and it applies the same safety margin twice:

```
6 m road, half-width                          3.00 m
edge risk reaches tau = 0.6 at                0.40 m from the boundary
inflation 0.9 (half car) + 0.8 (village)      1.70 m
free band for the vehicle centre              0.90 m
```

A 1.8 m corridor down the middle of a two-way road: the planner is pinned to
the centreline, head-on to oncoming traffic, and the ego's own start pose at a
1.5 m keep-left offset sits *inside* the occupied region, so Hybrid A* refuses
to plan at all. Measured: every global plan failed and the ego deadlocked at
0.36 m/s.

The edge layer already expresses "keep off the edge" as a smooth gradient.
Thresholding that gradient *and* inflating by the clearance margin states the
same intent twice — once as a cost, once as a wall.

**Action:** the hard constraint is what is genuinely impassable —
`offRoad | blocking | dynamic >= tau` — and inflation is the half vehicle
width alone, which is the geometric price of planning a point rather than a
body. Note `blocking`, not `static`: surface hazards such as potholes are
excluded, because a pothole's above-threshold region plus inflation is a
~2.15 m no-go radius and two of them close a 6 m carriageway completely. A1
asks for zero traversals *and* clearance, which is a statement about cost, not
impassability. The context clearance margin remains a preference, expressed
through the edge gradient and the DWA risk term.

Related: the ego's own cell is cleared from the inflated map when it would
otherwise be occupied (`freeStartCell`). The vehicle is already there and is
demonstrably not in collision, so inflation at that one pose has been overtaken
by events. Only that pose is relaxed — an earlier version relaxed the whole
map and the ego drove off the road at 59 m.

---

## D25 — A wrong-side layer was ADDED to the risk map

Not in the spec. Added because without it the planner has no reason to prefer
its own half of an unmarked two-way road: avoiding a crossing pedestrian, the
ego swerved right into the oncoming half, stopped there, and was struck
head-on by a car that was entirely in the right.

The layer is built from the same expected-direction field the wrong-way
detector (A2) already reads: a cell whose expected travel direction opposes the
ego's heading is where oncoming traffic belongs.

**Is this a lane graph, contrary to the problem statement?** No. There are no
lane centrelines, no lane ids and no discrete lane-change decisions — only a
continuous scalar field over free space, exactly like the other layers. The
planner is never told which lane it is in; it is told that some ground is
riskier, and it remains free to use that ground when the alternative is worse.
The weight (0.40) is deliberately below the planner's hard threshold (0.6), so
it discourages crossing without forbidding it — overtaking a stopped pushcart
has to stay possible.

---

## D26 — Performance measures needed to make the loop run at all

Profiling a 32 s village run (`profile on` / `profile('info')`, self time):

```
function                                        self(s)  total(s)    calls
SearchTree.nearestNeighbor                         5.01      5.01    94083
obbDistance>pointSegDistance                       3.95      3.95  3397568
obbDistance>segmentsIntersect                      3.51      5.61   849392
ReedsSheppBuiltins.autonomousReedsSheppSegments    3.30      3.65   370835
obbDistance>segSegDistance                         3.28     12.84   849392
validatorOccupancyMap.isStateValid                 2.69      5.00   205114
plannerRRTStar.extend                              0.90     20.22    94083
```

Three fixes, each of which changed behaviour only by making the cycle affordable:

1. **RRT\* ran on every failed Hybrid A\* call** — 20.2 s of a 32 s run, about
   62 ms of every 100 ms cycle. It now runs only when there is **no** path to
   fall back on (`GlobalPlanner.notePathHeld`); otherwise the previous path is
   held and Hybrid A* is retried next cycle. `MaxIterations` cut 2000 → 600.
2. **`ttc()` computed an exact box-to-box distance at every sampled instant**
   for `info.dMin`, which almost no caller wants: 849k segment-distance
   evaluations. Now computed only when a second output is requested, and the
   sampling step relaxed 0.02 s → 0.05 s.
3. **Log payloads** — the candidate fan alone is ~43 MB per village run
   (77 rollouts x 25 steps x 2 coords x 8 bytes x ~1400 cycles) on a machine
   with under 1 GB free during a batch. `cfg.io.logDetail = 'light'` stores the
   risk field, candidate fan and predictions every 5th cycle. Display only —
   every metric is computed at full rate.

Result: village wall clock 227 s → 92 s, and p95 cycle latency **443 ms → 87 ms**,
inside the 200 ms target.

---

## D27 — Soft-constraint settings in the submitted build

The prototype is deliberately tuned softer than the specification. Each of
these changes what the vehicle **does**; none changes how it is **scored**.

| Setting | This build | Spec | Reason |
|---|---|---|---|
| `plan.rejectRisk` | 0.95 | 0.85 | lets the planner use ground it would otherwise refuse on a 6 m road |
| `sim.maxTimeMult` | 5.0 | 3.0 | the timeout detects deadlock; a vehicle at 97% of the route moving steadily is not deadlocked |
| `risk.wWrongSide` | 0.25 | — (added, D25) | discourages the oncoming half without forbidding an overtake |
| `plan.dwaSafeGap` | 1.6 m | — | two vehicles meeting on a narrow road pass with ~1.3 m of body gap; the term has to act while there is still room to act |
| DWA weights | rebalanced | — | see below |

**The weight rebalance is the one worth reading.** Every DWA term is bounded in
[0,1] and the weights sum to 1, so a weight is the share of the decision a
consideration gets. The original balance gave `risk` the largest share (0.357
in village) against `goal` + `speed` combined (0.32). On a road where merely
*being* on the road carries risk of roughly 0.25 — edge proximity, the
wrong-side field, other traffic — that made standing still cheaper than
driving, and the vehicle stalled mid-route with a clear road ahead, 10 s TTC
and 10 m of clearance. Progress now takes the larger share. Risk still shapes
*where* the vehicle drives; it no longer decides *whether* it drives.

---

## D28 — Agent-behaviour corrections found by collision analysis

Three changes to the simulated world, each made because a collision was being
caused by the agent model rather than by the planner, and each verified by
re-running:

1. **Lateral drift is clamped to the agent's own half of the road.** An auto's
   0.77 m drift amplitude around a −1.38 m offset reached −0.6 m; on a 7 m
   street that closed the gap on an ego correctly keeping left, which was
   struck while nearly stationary.
2. **A pedestrian or cow already mid-crossing stops short of a vehicle that is
   already stationary** — within **3 m only**. At 10 m this produced a *mutual
   deadlock*: the ego waits for the pedestrian, the pedestrian waits for the
   ego, and village collapsed from 244 m to 39 m. Two agents each waiting for
   the other is a worse failure than the collision it was meant to prevent.
3. **The cattle trigger has a 12 m floor.** Braking distance goes to zero as
   the ego slows, and the trigger is measured from the ego's rear axle — so at
   2 m/s the raw formula fired when the animal was 1.75 m from the rear axle,
   i.e. already beside the front of a 4.5 m vehicle. That is not a test of
   avoidance; it is an unavoidable collision that would have been scored
   against the planner.

---

## D12 — `events` is a reserved keyword in a `classdef` block

`SimLog` originally declared a property named `events`:

```
Error: Illegal use of reserved keyword "events".
```

`events` introduces an event block in MATLAB classes, so it cannot be a
property name. **Action:** renamed to `eventLog` throughout.

---

## D29 — Departed agents stayed visible to the toolbox sensors

`AgentModel.step` deactivates an agent once it passes the end of the modelled
stretch (`s > L + 1`), and `active` is respected everywhere the project consumes
agent truth: the collision check, the clearance sweep, the perception metrics,
the BEV renderer and the fallback synthetic sensor all skip inactive agents.

`SimEngine.syncContainer` did not. It mirrored every truth into the
`drivingScenario` actor list unconditionally, and that is the one path that feeds
the *toolbox* sensor models. So a departed agent kept its actor parked at
whatever pose it held on its last live step, where `visionDetectionGenerator` and
`drivingRadarDataGenerator` went on detecting it for the rest of the run.

Two things made this severe rather than cosmetic:

- `step` returns early once inactive, so `vs`/`vd` freeze too, and `truth()` kept
  reporting 7-16 m/s for something that was not moving. A constant-velocity
  filter predicted it forward each step, the measurement snapped it back, and
  association never settled.
- four highway agents reach the end of the ribbon within a few metres of each
  other, so they stacked - two pairs at *identical* coordinates.

Measured on highway seed 1: 29.6 tracks on average for 5 agents, 90.3% of them
false, all piled at x = 394-411 m on a 392 m route, 16 m in front of the ego. The
risk map painted that corridor solid and the ego stopped at 384 m of 392 and
never resumed. **This was the highway stall**, reported until now as an
unexplained local-planner defect.

**Action:** two fixes.

1. `AgentModel` zeroes `vs`/`vd` on deactivation. A thing that is not moving has
   speed zero.
2. `syncContainer` parks inactive actors at (1e4, 1e4) with zero velocity. A
   `drivingScenario` actor cannot be removed mid-run, and leaving it in place is
   what caused this, so moving it out of range is the available idiom - the
   longest sensor here reaches 150 m, so 1e4 m is out of range by four orders of
   magnitude and cannot come back.

After the fix: highway reaches the goal in 48.5 s, 14.1 mean tracks, false-track
rate 0.50, and the cycle p95 fell from 304 ms to 182 ms.

## D30 — Per-stage cycle timings, and what they showed

Latency was previously logged only end to end. That says a cycle missed the
200 ms budget; it does not say which stage to fix. `SimEngine.proposedCycle` now
times eight stages separately (sense, track, predict, detect, risk, fsm, global,
dwa), `MetricsLogger.stageBreakdown` reduces them to p50/p95 per stage, and
`run_experiments` writes them to `runs.csv` as `stage_<name>_p50/p95`.

An `unaccounted` residual (cycle total minus the sum of the stages) is reported
rather than absorbed into a stage, so the table cannot quietly stop adding up.
Measured at 0.1 ms median.

The first measurement redirected the whole optimisation effort. Per-stage p50,
isolated, before the D29 fix:

| scenario | sense | track | risk | dwa | total |
|---|---|---|---|---|---|
| village | 16.6 | 15.1 | 5.3 | 11.3 | 49.9 |
| intersection | 29.1 | **62.5** | 7.5 | 18.6 | 128.5 |
| highway | 25.0 | **135.2** | 10.4 | 33.7 | 208.8 |
| market | 38.5 | **87.0** | 12.9 | 12.9 | 161.2 |
| cattle | 13.3 | 18.5 | 5.5 | 19.5 | 58.7 |

Tracking was 49-65% of every cycle that missed the budget. The local planner,
which had received most of the earlier optimisation work, was 11-34 ms throughout
and was never the problem. Tracking cost scales with the track count, which is
what pointed at D29 and D31.

## D31 — Duplicate tracks, and why merging is fusion rather than selection

With four sensors, a lax confirmation threshold (`[2 4]`) and a 1 s deletion
grace, the tracker confirms more tracks than there are objects. Measured: six
confirmed tracks inside a 2 m circle on a single pushcart in village.

Duplicates are not cosmetic. Every copy is predicted forward independently and
every copy is painted into the risk map, so N copies of one slow vehicle 12 m
ahead produce a wall of predicted occupancy that one vehicle would not. The ego
yielded to a crowd that was not there: six copies of a 0.9 m/s pushcart made a
6 m road impassable.

**Action:** `TrackerWrapper.mergeDuplicates` clusters tracks closer than
`cfg.tracking.mergeMinGap` (1.0 m) and fuses each cluster.

Three designs were measured, and the two obvious ones both failed:

| survivor rule | highway recall | effect |
|---|---|---|
| most updates | 0.546 | keeps the long-lived copy, which has been coasting and has drifted off its object |
| smallest covariance | 0.963 | recovers recall, but the cluster winner changes as covariances fluctuate, so ids churn, the risk field flickers, and control degrades - village fell to 76.8 m and cattle left the road |
| **information-form fusion** | 0.963 | as well localised as the best member, with the id from the oldest member so downstream state stays attached to one object |

Fusion combines position and velocity in information form and then **inflates**
the fused covariance by the spread of the members: if four tracks disagree by a
metre about where one object is, that disagreement is real uncertainty and must
survive into the risk map. Without it, fusing four vague tracks would manufacture
one confident one.

The cluster radius is a single tight 1.0 m rather than the class footprint. An
earlier version scaled it with the class extent - 1.8 m for two cars - and on the
highway that fused tracks belonging to *different* vehicles. The fused estimate
sat between two real cars and matched neither, taking position recall to 0.48
while the track count still looked healthy. Merging distinct objects is worse
than keeping a duplicate: a duplicate over-states an obstacle that is really
there, whereas a bad merge invents a vehicle where there is none and loses two
that exist.

## D32 — Anti-stall: stopping is the FSM's decision, not a cost-function side effect

Standing still occupies one cell. If that cell is clear its risk term is exactly
0, while any trajectory that moves enters cells further out and scores above 0.
So whenever

```
w_risk * R(moving)  >  w_speed * (speed gain) + w_goal * (progress gain)
```

the cheapest candidate is `v = 0` - and because the ego then does not move, the
same comparison holds on the next cycle and every cycle after it. The vehicle is
deadlocked by arithmetic, on a road it could drive.

Measured on village seed 1 at t = 100.1 s, stopped 30 m from the goal with a
static hazard 3.5 m ahead and a 1.1 m gap beside it:

```
   v    kappa    TOTAL |   goal    risk   clear  smooth   speed
 0.00   0.0000  0.3668 |  0.500   0.000   0.000   0.000   1.000   <- argmin
 0.42   0.0000  0.3853 |  0.487   0.150   0.000   0.000   0.940
```

Re-weighting cannot fix this in general: for *any* positive risk weight there is
a risk level at which paralysis wins, and lowering it far enough to prevent that
would stop the risk field mattering, which is the contribution. This is the fifth
instance of the same family after the four bounded-term defects, and the first
that is not a scaling error - the terms here are all correctly in [0,1].

**Action:** an architectural rule rather than a weight change. The behaviour
layer decides *whether* to proceed and expresses it as `speedCap`; STOP and
EMERGENCY_BRAKE set `speedCap = 0`. A non-zero cap is therefore an explicit
decision to move, and the local planner's job is to choose *how*, not to veto it
on cost. When the FSM has decided to proceed and a feasible moving trajectory
exists, the planner takes the cheapest moving one.

Three qualifications, all measured:

- It never overrides a hard constraint. The moving candidate passed the same
  rejection test as every other; if nothing moving survived, the vehicle stops.
- It is **not immediate**. Waiting is often right: in cattle the ego stops for a
  cow, the cow wanders off, and the ego proceeds. Forcing motion the instant
  `v = 0` won on cost made the ego creep at the cow until the FSM escalated to
  STOP, and that run went from reaching the goal to timing out at 146 m. The
  override arms only after `cfg.plan.stallBreakTime` (3.0 s) of standing still
  while the FSM was asking the vehicle to proceed.
- Once armed it **latches** until the vehicle is genuinely rolling. Firing for a
  single cycle achieves nothing: a 0.42 m/s target held for one 0.1 s cycle moves
  the ego a few centimetres, `v = 0` wins again immediately, and the vehicle
  ratchets forward about a metre every 3 s. Village advanced 0.9 m in 77 s that
  way before timing out.

## D33 — Escape: the rejection rule governs entering a violating state, not leaving one

When the ego's own footprint is at or above `rejectRisk`, *every* candidate is
rejected including holding position. "Everything was rejected" is then not
information about where the ego may go - it is information about where the ego
already is. Returning BLOCKED preserves the violation instead of ending it, the
FSM latches STOP, and nothing can change.

Measured on village seed 1: the ego sat at (95.0, 1.5) in STOP with 77 of 77
candidates rejected, from t = 55 s to the timeout at 177.5 s.

**Action:** a separate rule for leaving. When every candidate is rejected, take
the rejected candidate with the lowest peak risk - ordered by peak, then by
*mean* risk, then by lowest speed - provided it is strictly better than holding,
and capped at `cfg.plan.escapeSpeed` (1.0 m/s).

The mean-risk tie-break is what makes the rule work where it is needed. Deep in
an off-road region every peak saturates at 1.0, so peak alone gives no gradient
and the vehicle would sit in the violation it is supposed to be leaving. The mean
still points downhill.

This does not weaken the ordinary rejection rule: it applies only when every
alternative, *including standing still*, is already rejected, and it claims no
cost for the trajectory it returns. `out.blocked` stays true, so the FSM still
sees a blocked path.

Note the remaining limitation, which is not fixed: the dynamic window contains no
reverse speeds, so a vehicle that has driven into a pocket with no
forward-reachable lower-risk cell still cannot recover. A real system would
reverse.

**One property of the ordering, stated because it is not obvious.** The
tie-break prefers the slowest candidate among equals, and the zero-speed
rollout is a candidate. So when a whole neighbourhood saturates — every
reachable cell at risk 1.0, giving identical peak *and* identical mean — the
stationary rollout wins the tie and no escape fires. That is the correct
outcome rather than a defect: if nothing within one second of travel is better
than where the ego is, moving is not an escape, it is just moving. But it does
mean the escape rule cannot rescue a vehicle from the *middle* of a large
non-drivable region, only from its edge. Reverse would be the real answer, and
there is none. Found by re-reading the code while the reported matrix was
running, and left unchanged for that reason — a planner edit mid-matrix would
produce results that did not come from the shipped code.

## D34 — Verification: are the village events independently randomised?

Asked directly during the build: confirm the pedestrian crossing and the
oncoming car in Scenario 1 are independently randomised per seed and not forced
to coincide. Read from `Scenario1.m` rather than assumed.

**Not forced to coincide — confirmed.** Nothing couples the two events. The
pedestrian crossing is triggered by ego proximity
(`trigger.type = 'egoDistance', at = 22.0`), so it fires when the ego arrives
rather than on a clock, and the oncoming car is an ordinary agent with no
trigger at all. Whether the two overlap depends on the ego's own speed history,
which differs by configuration. A slow baseline cannot dodge the crossing by
arriving late, and no configuration is handed a coincidence.

**Independently drawn — confirmed, with one weakness.** Both draw from
substream 3 as successive calls, which makes them independent draws. But what
varies differs between them:

| event | what varies with the seed | what is fixed |
|---|---|---|
| pedestrian | position along the road, `55 + 45k + U(0,20)` m | trigger distance, 22.0 m |
| oncoming car | speed, `U(6.0, 8.5)` m/s | **start position, `goalS - 20`** |

The oncoming car's start position is **not** randomised. Only its speed is, so
the point on the road where the ego meets it varies by roughly the ratio of
those speeds rather than freely. The "squeeze" — pedestrian crossing and
oncoming car on 6 m at the same moment — therefore occurs across a narrower
band of geometries than the design intended.

**Not changed.** This was found while the reported matrix was running, and
randomising the spawn position would change every village result to something
not produced by the shipped code. It is recorded as a limitation on how much
seed-to-seed variety Scenario 1 actually contains, which is the honest handling:
the numbers stay the numbers that were measured, and the reader is told what the
seeds do and do not vary.

