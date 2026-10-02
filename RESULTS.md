# AdaptaDrive -- measured results

**Generated from `results/summary.csv` by `make_results_md`.**
Not typed by hand. Re-run `run_experiments` then `make_results_md` to refresh.

> Simulation results. Architectural coverage is not validated performance.

- runs: **90**    seeds: **[1 2]**    generated: 2026-09-30 19:53
- machine: 11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz
- MATLAB: R2026a

---

## Collision-free completion

Wilson 95% intervals, because N is small: a bare percentage from a
handful of runs implies precision the sample cannot support.

| scenario | config | n | completion | 95% CI | collisions | timeouts | off-road |
|---|---|---|---|---|---|---|---|
| cattle | -classpriors | 2 | 50% | 9-91% | 1 | 0 | 0 |
| cattle | -context | 2 | 100% | 34-100% | 0 | 0 | 0 |
| cattle | -riskmap | 2 | 50% | 9-91% | 1 | 0 | 0 |
| cattle | -uncertainty | 2 | 50% | 9-91% | 1 | 0 | 0 |
| cattle | -wrongside | 2 | 50% | 9-91% | 1 | 0 | 0 |
| cattle | BL1 | 2 | 100% | 34-100% | 0 | 0 | 0 |
| cattle | BL2 | 2 | 50% | 9-91% | 1 | 0 | 0 |
| cattle | BL3 | 2 | 50% | 9-91% | 1 | 0 | 0 |
| cattle | PROPOSED | 2 | 50% | 9-91% | 1 | 0 | 0 |
| highway | -classpriors | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | -context | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | -riskmap | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | -uncertainty | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | -wrongside | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | BL1 | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | BL2 | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | BL3 | 2 | 100% | 34-100% | 0 | 0 | 0 |
| highway | PROPOSED | 2 | 100% | 34-100% | 0 | 0 | 0 |
| intersection | -classpriors | 2 | 100% | 34-100% | 0 | 0 | 0 |
| intersection | -context | 2 | 50% | 9-91% | 1 | 0 | 0 |
| intersection | -riskmap | 2 | 100% | 34-100% | 0 | 0 | 0 |
| intersection | -uncertainty | 2 | 0% | 0-66% | 1 | 1 | 0 |
| intersection | -wrongside | 2 | 50% | 9-91% | 1 | 0 | 0 |
| intersection | BL1 | 2 | 0% | 0-66% | 2 | 0 | 0 |
| intersection | BL2 | 2 | 0% | 0-66% | 1 | 1 | 0 |
| intersection | BL3 | 2 | 100% | 34-100% | 0 | 0 | 0 |
| intersection | PROPOSED | 2 | 100% | 34-100% | 0 | 0 | 0 |
| market | -classpriors | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | -context | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | -riskmap | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | -uncertainty | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | -wrongside | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | BL1 | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | BL2 | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | BL3 | 2 | 0% | 0-66% | 2 | 0 | 0 |
| market | PROPOSED | 2 | 0% | 0-66% | 2 | 0 | 0 |
| village | -classpriors | 2 | 100% | 34-100% | 0 | 0 | 0 |
| village | -context | 2 | 50% | 9-91% | 0 | 1 | 0 |
| village | -riskmap | 2 | 0% | 0-66% | 0 | 2 | 0 |
| village | -uncertainty | 2 | 0% | 0-66% | 0 | 2 | 0 |
| village | -wrongside | 2 | 0% | 0-66% | 0 | 2 | 0 |
| village | BL1 | 2 | 0% | 0-66% | 2 | 0 | 0 |
| village | BL2 | 2 | 50% | 9-91% | 0 | 1 | 0 |
| village | BL3 | 2 | 0% | 0-66% | 0 | 2 | 0 |
| village | PROPOSED | 2 | 50% | 9-91% | 0 | 1 | 0 |

## Safety and comfort (mean +/- std)

| scenario | config | completion time (s) | mean speed (m/s) | min clearance (m) | min TTC (s) | jerk RMS (m/s^3) |
|---|---|---|---|---|---|---|
| cattle | -classpriors | 27.32 +/- 19.34 | 7.42 +/- 0.65 | 0.41 +/- 0.58 | 0.52 +/- 0.73 | 10.90 +/- 2.05 |
| cattle | -context | 67.65 +/- 2.47 | 4.23 +/- 0.16 | 1.19 +/- 0.02 | 1.68 +/- 0.20 | 7.98 +/- 2.80 |
| cattle | -riskmap | 31.87 +/- 25.92 | 6.99 +/- 1.85 | 0.37 +/- 0.53 | 0.92 +/- 1.30 | 9.67 +/- 2.45 |
| cattle | -uncertainty | 28.65 +/- 20.79 | 7.15 +/- 0.80 | 0.41 +/- 0.58 | 0.86 +/- 1.21 | 12.19 +/- 0.04 |
| cattle | -wrongside | 27.85 +/- 19.73 | 7.36 +/- 0.75 | 0.55 +/- 0.78 | 1.16 +/- 1.64 | 10.52 +/- 2.06 |
| cattle | BL1 | 45.42 +/- 0.11 | 6.29 +/- 0.01 | 0.59 +/- 0.50 | 0.62 +/- 0.26 | 1.90 +/- 0.00 |
| cattle | BL2 | 39.22 +/- 22.73 | 5.31 +/- 0.20 | 0.66 +/- 0.93 | 0.43 +/- 0.61 | 2.21 +/- 0.02 |
| cattle | BL3 | 38.95 +/- 35.50 | 6.14 +/- 2.34 | 0.39 +/- 0.55 | 0.77 +/- 1.09 | 15.77 +/- 4.89 |
| cattle | PROPOSED | 35.15 +/- 30.26 | 6.41 +/- 1.91 | 0.83 +/- 1.17 | 0.70 +/- 0.98 | 10.51 +/- 3.12 |
| highway | -classpriors | 47.45 +/- 1.56 | 8.07 +/- 0.27 | 4.84 +/- 1.12 | 4.39 +/- 0.41 | 4.99 +/- 2.91 |
| highway | -context | 54.97 +/- 0.04 | 6.95 +/- 0.01 | 39.47 +/- 6.68 | 8.85 +/- 1.63 | 1.56 +/- 0.18 |
| highway | -riskmap | 46.47 +/- 0.60 | 8.23 +/- 0.11 | 3.42 +/- 0.91 | 4.45 +/- 0.88 | 7.24 +/- 0.15 |
| highway | -uncertainty | 47.55 +/- 1.34 | 8.05 +/- 0.22 | 3.84 +/- 0.01 | 4.13 +/- 0.79 | 7.00 +/- 0.58 |
| highway | -wrongside | 47.32 +/- 2.16 | 8.09 +/- 0.37 | 2.18 +/- 0.55 | 4.14 +/- 0.07 | 8.03 +/- 0.49 |
| highway | BL1 | 54.00 | 7.07 | 39.46 +/- 6.70 | 10.00 | 0.58 |
| highway | BL2 | 54.17 +/- 0.11 | 7.06 +/- 0.00 | 39.47 +/- 6.68 | 8.87 +/- 1.60 | 0.71 +/- 0.01 |
| highway | BL3 | 45.65 +/- 1.48 | 8.43 +/- 0.29 | 3.35 +/- 0.41 | 6.35 +/- 5.16 | 8.11 +/- 3.07 |
| highway | PROPOSED | 47.32 +/- 2.16 | 8.09 +/- 0.37 | 2.18 +/- 0.55 | 4.14 +/- 0.07 | 8.03 +/- 0.49 |
| intersection | -classpriors | 52.85 +/- 32.17 | 2.35 +/- 1.43 | 0.60 +/- 0.82 | 0.83 +/- 0.82 | 4.81 +/- 4.66 |
| intersection | -context | 21.55 +/- 14.35 | 3.38 +/- 0.28 | 0.62 +/- 0.88 | 0.84 +/- 1.18 | 10.30 +/- 3.01 |
| intersection | -riskmap | 33.33 +/- 2.09 | 3.03 +/- 0.20 | 1.13 +/- 0.12 | 1.77 +/- 0.42 | 5.02 +/- 3.52 |
| intersection | -uncertainty | 54.67 +/- 55.12 | 1.42 +/- 1.34 | 0.65 +/- 0.92 | 0.61 +/- 0.86 | 8.15 +/- 4.07 |
| intersection | -wrongside | 25.10 +/- 13.22 | 2.66 +/- 0.37 | 0.25 +/- 0.36 | 0.61 +/- 0.87 | 9.11 +/- 2.03 |
| intersection | BL1 | 10.50 +/- 4.74 | 5.15 +/- 2.17 | 0.00 | 0.00 | 2.34 +/- 1.03 |
| intersection | BL2 | 41.95 +/- 46.60 | 3.14 +/- 3.57 | 0.60 +/- 0.85 | 0.47 +/- 0.66 | 2.06 +/- 1.14 |
| intersection | BL3 | 32.43 +/- 2.37 | 3.12 +/- 0.23 | 0.31 +/- 0.38 | 0.67 +/- 0.45 | 9.60 +/- 2.88 |
| intersection | PROPOSED | 30.28 +/- 1.80 | 3.33 +/- 0.20 | 0.86 +/- 0.73 | 1.65 +/- 0.65 | 8.10 +/- 0.39 |
| market | -classpriors | 11.03 +/- 1.59 | 1.08 +/- 0.81 | 0.00 | 1.08 +/- 1.53 | 2.09 +/- 0.67 |
| market | -context | 13.88 +/- 9.86 | 0.86 +/- 0.15 | 0.00 | 1.11 +/- 1.58 | 9.57 +/- 10.26 |
| market | -riskmap | 6.05 +/- 0.64 | 0.95 +/- 0.19 | 0.00 | 0.00 | 2.38 +/- 0.22 |
| market | -uncertainty | 8.68 +/- 3.64 | 1.38 +/- 0.63 | 0.00 | 1.13 +/- 1.60 | 1.98 +/- 0.22 |
| market | -wrongside | 6.35 +/- 0.64 | 0.88 +/- 0.16 | 0.00 | 0.00 | 9.16 +/- 9.35 |
| market | BL1 | 8.20 +/- 8.84 | 3.85 +/- 0.96 | 0.00 | 0.00 | 3.07 +/- 0.26 |
| market | BL2 | 6.00 +/- 2.19 | 2.07 +/- 0.89 | 0.00 | 1.57 +/- 0.12 | 2.98 +/- 0.40 |
| market | BL3 | 9.45 +/- 4.24 | 1.24 +/- 0.58 | 0.00 | 1.09 +/- 1.55 | 9.49 +/- 10.29 |
| market | PROPOSED | 6.82 +/- 0.81 | 0.77 +/- 0.19 | 0.00 | 0.00 | 8.96 +/- 9.29 |
| village | -classpriors | 57.60 +/- 0.92 | 4.22 +/- 0.06 | 0.35 +/- 0.00 | 2.18 +/- 0.23 | 9.21 +/- 0.97 |
| village | -context | 138.43 +/- 55.26 | 1.44 +/- 1.42 | 0.32 +/- 0.29 | 1.46 +/- 0.42 | 4.31 +/- 2.00 |
| village | -riskmap | 177.50 | 0.77 +/- 0.62 | 0.60 +/- 0.50 | 1.64 +/- 0.72 | 3.33 +/- 3.52 |
| village | -uncertainty | 177.50 | 0.47 +/- 0.02 | 0.64 +/- 0.22 | 1.35 +/- 0.04 | 4.95 +/- 0.37 |
| village | -wrongside | 177.50 | 0.47 +/- 0.04 | 0.85 +/- 0.35 | 1.26 +/- 0.16 | 3.36 +/- 0.27 |
| village | BL1 | 6.25 +/- 0.21 | 6.93 +/- 0.01 | 0.00 | 0.00 | 1.07 +/- 0.10 |
| village | BL2 | 116.05 +/- 86.90 | 2.50 +/- 2.79 | 0.80 +/- 0.81 | 0.96 +/- 1.07 | 1.70 +/- 1.32 |
| village | BL3 | 177.50 | 0.46 +/- 0.01 | 0.66 +/- 0.19 | 1.23 +/- 0.12 | 5.00 +/- 0.47 |
| village | PROPOSED | 138.43 +/- 55.26 | 1.44 +/- 1.42 | 0.32 +/- 0.29 | 1.46 +/- 0.42 | 4.31 +/- 2.00 |

## Planning-cycle latency vs the 200 ms target

Interpreted MATLAB on 11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz. Not an embedded target.

**Measured in isolation** -- one scenario at a time with nothing else
running. These are the figures to quote.

Median of **3 repeats** per scenario, with the p95 range across
them. The run is deterministic - same path every time - so the
spread is measurement noise, not different driving. It is shown
because a single pass put highway at 178 ms and then 217 ms, either
side of the target, and a verdict from one pass would be noise
presented as a result.

| scenario | p50 (ms) | p95 (ms) | p95 range | max (ms) | under 200 ms | meets target? |
|---|---|---|---|---|---|---|
| village | 45 | 146 | 145-154 | 822 | 99.6% | YES |
| intersection | 70 | 314 | 310-319 | 639 | 66.1% | **NO** |
| highway | 103 | 179 | 177-180 | 557 | 97.6% | YES |
| market | 208 | 278 | 272-280 | 615 | 41.9% | **NO** |
| cattle | 42 | 98 | 96-98 | 478 | 99.8% | YES |

**3 of 5 scenarios meet the 200 ms p95 target.**
The target is not relaxed to improve that count; the misses are
reported as misses.

**Where the time goes**, per stage, same isolated runs. A single
end-to-end number says a cycle missed the budget; it does not say
which stage to fix. Median milliseconds.

| scenario | sense | track | predict | detect | risk | fsm | global | dwa | unacct | total |
|---|---|---|---|---|---|---|---|---|---|---|
| village | 16.8 | 13.6 | 0.3 | 0.1 | 4.6 | 0.6 | 0.0 | 6.3 | 0.1 | 42.3 |
| intersection | 22.0 | 26.1 | 0.5 | 0.2 | 5.2 | 0.9 | 4.4 | 12.0 | 0.1 | 71.2 |
| highway | 21.6 | 47.0 | 0.5 | 0.2 | 6.3 | 1.4 | 0.0 | 10.2 | 0.0 | 87.2 |
| market | 41.1 | 113.3 | 1.0 | 0.4 | 12.0 | 1.5 | 8.2 | 19.7 | 0.0 | 197.2 |
| cattle | 12.4 | 14.0 | 0.4 | 0.1 | 4.0 | 0.5 | 0.0 | 6.7 | 0.0 | 38.0 |

The unaccounted column is the cycle total minus the sum of the
stages, per cycle. It is reported rather than absorbed into a stage,
so this table cannot quietly stop adding up.

**Measured during the batch** -- shown for completeness, and higher.
This machine has 7.7 GB of RAM and under 1 GB free during a run of the
matrix, so batch figures include time the process spent contending for
memory rather than planning. They are reported rather than dropped, but
the isolated numbers above are the honest measure of the planner.

| scenario | p50 (ms) | p95 (ms) |
|---|---|---|
| village | 107 | 214 |
| intersection | 90 | 305 |
| highway | 97 | 162 |
| market | 202 | 312 |
| cattle | 44 | 112 |

## Requirement A1 -- surface hazards

| scenario | pothole traversals (total) | min pothole clearance (m) |
|---|---|---|
| village | 0 | 0.16 |
| intersection | 0 | 0.26 |
| highway | 2 | 0.00 |
| market | 0 | 53.85 |
| cattle | 2 | 0.00 |

A1 asks for zero traversals and at least 0.5 m clearance. The numbers
above are reported as measured, pass or fail.

## Requirements A2 and A3 -- detectors

| scenario | wrong-way flags | merge detections |
|---|---|---|
| village | 5 | 25 |
| intersection | 9 | 9 |
| highway | 0 | 19 |
| market | 1 | 6 |
| cattle | 0 | 6 |

Village and cattle contain no wrong-way vehicle and no merging vehicle:
zero there is the correct answer, and is the false-positive check.

---

## How to reproduce

```matlab
startup
run_experiments('seeds', [1 2])
make_figures
make_results_md
```

Every row above came from a simulation that ran. Runs that errored are
recorded with outcome `error` and NaN metrics rather than dropped.
