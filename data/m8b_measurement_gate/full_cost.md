# M8b -- the full-trajectory cost of the heading gate, and what the "~= zero cost" claim missed

> Written 2026-09-20 after a reviewer pointed out that the "bounded at every gain" claim for the
> measurement-side gate is contradicted by the recording itself.  It is.  This file reports the
> full-run cost of all four arms, lists every excursion above 0.10 m with its cause, and dissects
> the episode that the earlier exclusion rule removed.  Sources: `full_cost_table.csv`,
> `full_cost_excursions.csv`, `full_cost_injection.csv`, `episode_1527_1550.csv` in this directory,
> written by `analysis/m8b_post/full_cost.py` and `analysis/m8b_post/episode_1527_1550.py`.
> Figure: `fig/Fig10_hold_in_turn.pdf` (`analysis/m8b_post/fig10_hold_in_turn.py`).
> Nothing under `analysis/m8`, `analysis/m8b`, `data/m8_replay` or any pre-existing file in
> `data/m8b_measurement_gate` was modified.

## 0 Time origin, and the one that the earlier script used

Every offset in this file is `t_abs - T0_REF` with **T0_REF = 1788911806.950244 s**, the minimum `t` of
`runs/bas_meas_q060/rtk_ref.csv`.  `analysis/m8b/baseline_cost_m8b.py` prints its episodes against
**T0_REC = 1788911802.737 s**, the first row of the source recording `data/csv/run_20260909_2026-09-09-07-56-42.csv`
(which is also the origin of `data/m8_replay/events.csv`):

    offset_REF = offset_REC -4.213244 s

So the episode that script calls 1531.5-1533.5 s is **1527.3-1529.3 s** here, and the 45 injections,
which start at offset_REC 1590 s, start at offset_REF 1585.8 s -- all of them **after** the episode.

## 1 Full-trajectory cost of the eight baseline runs

Deviation = the consumer `/odometry/gps` against the recorded `/rtk_odom`, constant frame offset
removed as the median over the whole run (identical definition to `analysis/m8/analyze_m8.py`).
Three exclusion rules:

* **(a) raw** -- nothing removed.
* **(b) frozen frames only** -- the 85 recorded frames whose heading is frozen or exactly zero are
  removed and nothing else (a consumer sample is dropped when the nearest such frame is within 0.1 s,
  i.e. half a heading period).  No lead, no tail.
* **(c) old guard** -- exactly what `baseline_cost_m8b.py` does: drop `exclude.csv` +-1 s, then drop
  each of the ten frozen episodes with a -2 s lead and a +12 s tail.  Reproduced here to the digit
  (summary.md table 7).

| run | (a) med | (a) p99 | (a) max | (b) med | (b) p99 | (b) max | (c) med | (c) p99 | (c) max | frames kept (b)/(c)/all |
|---|---|---|---|---|---|---|---|---|---|---|
| STOCK, q=0.06 | 0.0001 | 0.0170 | 0.5303 | 0.0001 | 0.0170 | 0.5303 | 0.0001 | 0.0162 | 0.1036 | 9668/9049/9750 |
| GATED (node side), q=0.06 | 0.0001 | 0.0363 | 0.7295 | 0.0001 | 0.0266 | 0.7295 | 0.0001 | 0.0171 | 0.1234 | 9641/9059/9714 |
| MEAS (measurement side), q=0.06 | 0.0003 | 0.3134 | 0.7257 | 0.0003 | 0.0580 | 0.7257 | 0.0003 | 0.0168 | 0.1234 | 9666/9047/9748 |
| BOTH (stacked), q=0.06 | 0.0002 | 0.4988 | 0.7310 | 0.0002 | 0.3238 | 0.7310 | 0.0002 | 0.0240 | 0.7310 | 9659/9040/9741 |
| STOCK, q=1e-5 | 0.0026 | 0.0819 | 0.6856 | 0.0026 | 0.0673 | 0.6856 | 0.0025 | 0.0444 | 0.1224 | 9675/9056/9757 |
| GATED (node side), q=1e-5 | 0.0026 | 0.2148 | 0.7310 | 0.0026 | 0.0980 | 0.7310 | 0.0025 | 0.0445 | 0.1281 | 9647/9050/9729 |
| MEAS (measurement side), q=1e-5 | 0.0023 | 0.2678 | 0.7248 | 0.0023 | 0.0749 | 0.7248 | 0.0022 | 0.0307 | 0.1261 | 9665/9046/9747 |
| BOTH (stacked), q=1e-5 | 0.0026 | 0.2889 | 0.7310 | 0.0026 | 0.2797 | 0.7310 | 0.0025 | 0.0535 | 0.7310 | 9650/9037/9732 |

Read (b) with care: it removes only the frames that are provably frozen, so it **keeps** the
1427-1438 s window, in which the recorded heading wanders instead of freezing and the reference is
wrong for all four arms -- that window supplies the ~0.7 m maximum in every row, STOCK included.
The gate cost proper is isolated in section 3, in a window where the reference is independently
shown to be correct.

## 2 Every excursion above 0.10 m, baseline runs

Contiguous runs of deviation > 0.10 m, merged across gaps below 0.6 s.  The cause is read from the
source recording (frozen heading, `hd_ok`, `dpsi_deg` against the gyro increment `dpsi_imu_deg`) and
from the gates' own logs (`meas_gate.csv`, `warns.csv`).  Full machine-readable list including the
injection runs: `full_cost_excursions.csv`.

| run | start (s) | end (s) | dur (s) | peak (m) | t_peak (s) | cause | node-gate warnings in window |
|---|---|---|---|---|---|---|---|
| STOCK, q=0.06 | 989.59 | 989.59 | 0.00 | 0.1147 | 989.59 | heading jump, gyro contradicts | -- |
| STOCK, q=0.06 | 1315.66 | 1315.66 | 0.00 | 0.2902 | 1315.66 | frozen heading | -- |
| STOCK, q=0.06 | 1317.67 | 1317.67 | 0.00 | 0.5303 | 1317.67 | heading jump, gyro contradicts | -- |
| STOCK, q=0.06 | 1530.53 | 1530.53 | 0.00 | 0.2466 | 1530.53 | frozen heading | -- |
| STOCK, q=0.06 | 1684.36 | 1684.36 | 0.00 | 0.1036 | 1684.36 | injection | -- |
| GATED (node side), q=0.06 | 989.59 | 989.99 | 0.40 | 0.1150 | 989.99 | heading jump, gyro contradicts | hold;stale |
| GATED (node side), q=0.06 | 1313.85 | 1314.26 | 0.40 | 0.2220 | 1314.26 | frozen heading | hold;reanchor_gyro;stale |
| GATED (node side), q=0.06 | 1315.46 | 1315.86 | 0.40 | 0.4927 | 1315.66 | frozen heading | hold;reanchor_gyro;stale |
| GATED (node side), q=0.06 | 1317.07 | 1317.07 | 0.00 | 0.5161 | 1317.07 | frozen heading | hold |
| GATED (node side), q=0.06 | 1430.93 | 1435.95 | 5.02 | 0.7261 | 1433.54 | frozen heading | hold;stale |
| GATED (node side), q=0.06 | 1436.75 | 1436.75 | 0.00 | 0.6900 | 1436.75 | frozen heading | -- |
| GATED (node side), q=0.06 | 1438.15 | 1438.15 | 0.00 | 0.6456 | 1438.15 | frozen heading | -- |
| GATED (node side), q=0.06 | 1440.36 | 1440.36 | 0.00 | 0.6873 | 1440.36 | frozen heading | -- |
| GATED (node side), q=0.06 | 1502.41 | 1502.61 | 0.20 | 0.1438 | 1502.61 | frozen heading | hold |
| GATED (node side), q=0.06 | 1504.22 | 1504.22 | 0.00 | 0.1281 | 1504.22 | frozen heading | hold;reanchor_gyro;stale |
| GATED (node side), q=0.06 | 1527.11 | 1527.51 | 0.40 | 0.2296 | 1527.31 | frozen heading | hold;stale |
| GATED (node side), q=0.06 | 1529.52 | 1536.15 | 6.63 | 0.7295 | 1535.15 | frozen heading | hold;reanchor_gyro;stale |
| GATED (node side), q=0.06 | 1536.95 | 1536.95 | 0.00 | 0.6525 | 1536.95 | other | -- |
| GATED (node side), q=0.06 | 1537.76 | 1538.16 | 0.40 | 0.5859 | 1537.76 | other | -- |
| GATED (node side), q=0.06 | 1538.96 | 1538.96 | 0.00 | 0.4778 | 1538.96 | other | -- |
| GATED (node side), q=0.06 | 1684.16 | 1684.16 | 0.00 | 0.1234 | 1684.16 | injection | -- |
| MEAS (measurement side), q=0.06 | 989.59 | 990.19 | 0.60 | 0.1149 | 989.99 | heading jump, gyro contradicts | -- |
| MEAS (measurement side), q=0.06 | 1313.85 | 1317.47 | 3.62 | 0.5205 | 1316.87 | frozen heading | -- |
| MEAS (measurement side), q=0.06 | 1430.93 | 1440.36 | 9.44 | 0.7257 | 1433.54 | frozen heading | -- |
| MEAS (measurement side), q=0.06 | 1502.41 | 1502.61 | 0.20 | 0.1438 | 1502.61 | frozen heading | -- |
| MEAS (measurement side), q=0.06 | 1504.22 | 1504.22 | 0.00 | 0.1301 | 1504.22 | frozen heading | -- |
| MEAS (measurement side), q=0.06 | 1527.11 | 1527.51 | 0.40 | 0.2315 | 1527.51 | frozen heading | -- |
| MEAS (measurement side), q=0.06 | 1529.52 | 1539.56 | 10.04 | 0.7141 | 1529.52 | gate hold in turn (re-anchored on frozen heading) | -- |
| MEAS (measurement side), q=0.06 | 1684.16 | 1684.16 | 0.00 | 0.1234 | 1684.16 | injection | -- |
| BOTH (stacked), q=0.06 | 989.59 | 989.99 | 0.40 | 0.1149 | 989.99 | heading jump, gyro contradicts | -- |
| BOTH (stacked), q=0.06 | 1313.85 | 1317.47 | 3.62 | 0.5203 | 1316.87 | frozen heading | reanchor_gyro;stale |
| BOTH (stacked), q=0.06 | 1380.12 | 1380.12 | 0.00 | 0.1011 | 1380.12 | other | -- |
| BOTH (stacked), q=0.06 | 1430.93 | 1440.36 | 9.44 | 0.7257 | 1433.54 | frozen heading | -- |
| BOTH (stacked), q=0.06 | 1502.41 | 1502.61 | 0.20 | 0.1438 | 1502.61 | frozen heading | -- |
| BOTH (stacked), q=0.06 | 1504.22 | 1504.22 | 0.00 | 0.1297 | 1504.22 | frozen heading | -- |
| BOTH (stacked), q=0.06 | 1527.11 | 1527.51 | 0.40 | 0.2315 | 1527.51 | frozen heading | -- |
| BOTH (stacked), q=0.06 | 1529.52 | 1549.40 | 19.87 | 0.7310 | 1548.79 | gate hold in turn (re-anchored on frozen heading) | hold;stale |
| BOTH (stacked), q=0.06 | 1684.16 | 1684.16 | 0.00 | 0.1237 | 1684.16 | injection | -- |
| STOCK, q=1e-5 | 558.10 | 558.91 | 0.80 | 0.1224 | 558.50 | other | -- |
| STOCK, q=1e-5 | 1313.85 | 1313.85 | 0.00 | 0.1116 | 1313.85 | frozen heading | -- |
| STOCK, q=1e-5 | 1315.66 | 1316.87 | 1.20 | 0.2549 | 1315.66 | frozen heading | -- |
| STOCK, q=1e-5 | 1317.67 | 1319.28 | 1.61 | 0.4205 | 1317.67 | heading jump, gyro contradicts | -- |
| STOCK, q=1e-5 | 1430.93 | 1434.14 | 3.21 | 0.6856 | 1431.53 | frozen heading | -- |
| STOCK, q=1e-5 | 1435.75 | 1437.95 | 2.21 | 0.6650 | 1435.75 | frozen heading | -- |
| STOCK, q=1e-5 | 1440.56 | 1442.77 | 2.21 | 0.6001 | 1440.56 | frozen heading | -- |
| STOCK, q=1e-5 | 1527.11 | 1527.51 | 0.40 | 0.1874 | 1527.11 | frozen heading | -- |
| STOCK, q=1e-5 | 1529.52 | 1531.73 | 2.21 | 0.6289 | 1529.52 | frozen heading | -- |
| STOCK, q=1e-5 | 1683.56 | 1683.56 | 0.00 | 0.1010 | 1683.56 | injection | -- |
| STOCK, q=1e-5 | 1684.16 | 1684.16 | 0.00 | 0.1078 | 1684.16 | injection | -- |
| GATED (node side), q=1e-5 | 558.10 | 558.71 | 0.60 | 0.1186 | 558.50 | other | -- |
| GATED (node side), q=1e-5 | 830.76 | 830.97 | 0.20 | 0.1060 | 830.76 | other | -- |
| GATED (node side), q=1e-5 | 831.77 | 831.77 | 0.00 | 0.1008 | 831.77 | other | -- |
| GATED (node side), q=1e-5 | 988.79 | 988.79 | 0.00 | 0.1059 | 988.79 | gate action | hold |
| GATED (node side), q=1e-5 | 1313.85 | 1315.86 | 2.01 | 0.4855 | 1315.86 | frozen heading | stale |
| GATED (node side), q=1e-5 | 1317.87 | 1319.48 | 1.61 | 0.3658 | 1318.07 | heading jump, gyro contradicts | reanchor_two |
| GATED (node side), q=1e-5 | 1430.93 | 1440.56 | 9.64 | 0.7309 | 1434.54 | frozen heading | hold;stale |
| GATED (node side), q=1e-5 | 1441.37 | 1442.97 | 1.61 | 0.2982 | 1441.57 | heading jump, gyro contradicts | reanchor_two |
| GATED (node side), q=1e-5 | 1527.11 | 1527.51 | 0.40 | 0.1818 | 1527.11 | frozen heading | -- |
| GATED (node side), q=1e-5 | 1529.52 | 1534.74 | 5.22 | 0.7016 | 1534.74 | frozen heading | hold;stale |
| GATED (node side), q=1e-5 | 1535.55 | 1539.16 | 3.61 | 0.7310 | 1535.75 | other | -- |
| GATED (node side), q=1e-5 | 1684.16 | 1684.16 | 0.00 | 0.1281 | 1684.16 | injection | -- |
| MEAS (measurement side), q=1e-5 | 989.59 | 989.99 | 0.40 | 0.1143 | 989.99 | heading jump, gyro contradicts | -- |
| MEAS (measurement side), q=1e-5 | 1313.85 | 1317.47 | 3.62 | 0.5219 | 1316.87 | frozen heading | -- |
| MEAS (measurement side), q=1e-5 | 1430.93 | 1440.36 | 9.44 | 0.7248 | 1433.54 | frozen heading | -- |
| MEAS (measurement side), q=1e-5 | 1502.61 | 1502.61 | 0.00 | 0.1209 | 1502.61 | frozen heading | -- |
| MEAS (measurement side), q=1e-5 | 1503.21 | 1503.21 | 0.00 | 0.1337 | 1503.21 | frozen heading | -- |
| MEAS (measurement side), q=1e-5 | 1504.22 | 1504.22 | 0.00 | 0.1067 | 1504.22 | frozen heading | -- |
| MEAS (measurement side), q=1e-5 | 1527.11 | 1527.51 | 0.40 | 0.2229 | 1527.51 | frozen heading | -- |
| MEAS (measurement side), q=1e-5 | 1529.52 | 1539.56 | 10.04 | 0.7018 | 1529.52 | gate hold in turn (re-anchored on frozen heading) | -- |
| MEAS (measurement side), q=1e-5 | 1683.56 | 1683.56 | 0.00 | 0.1112 | 1683.56 | injection | -- |
| MEAS (measurement side), q=1e-5 | 1684.16 | 1684.16 | 0.00 | 0.1261 | 1684.16 | injection | -- |
| BOTH (stacked), q=1e-5 | 558.10 | 558.71 | 0.60 | 0.1164 | 558.50 | other | -- |
| BOTH (stacked), q=1e-5 | 989.59 | 989.99 | 0.40 | 0.1143 | 989.99 | heading jump, gyro contradicts | -- |
| BOTH (stacked), q=1e-5 | 1313.85 | 1317.47 | 3.62 | 0.5228 | 1316.87 | frozen heading | reanchor_gyro;stale |
| BOTH (stacked), q=1e-5 | 1430.93 | 1440.36 | 9.44 | 0.7253 | 1433.54 | frozen heading | -- |
| BOTH (stacked), q=1e-5 | 1502.61 | 1502.61 | 0.00 | 0.1206 | 1502.61 | frozen heading | -- |
| BOTH (stacked), q=1e-5 | 1503.21 | 1503.21 | 0.00 | 0.1307 | 1503.21 | frozen heading | -- |
| BOTH (stacked), q=1e-5 | 1504.22 | 1504.22 | 0.00 | 0.1055 | 1504.22 | frozen heading | -- |
| BOTH (stacked), q=1e-5 | 1527.11 | 1527.51 | 0.40 | 0.2218 | 1527.51 | frozen heading | -- |
| BOTH (stacked), q=1e-5 | 1529.52 | 1539.96 | 10.44 | 0.7060 | 1529.52 | gate hold in turn (re-anchored on frozen heading) | hold;stale |
| BOTH (stacked), q=1e-5 | 1544.78 | 1549.40 | 4.62 | 0.7310 | 1548.79 | other | -- |
| BOTH (stacked), q=1e-5 | 1684.16 | 1684.16 | 0.00 | 0.1048 | 1684.16 | injection | -- |

Counts: STOCK, q=0.06 5 excursions / 0.0 s; GATED (node side), q=0.06 16 excursions / 13.9 s; MEAS (measurement side), q=0.06 8 excursions / 24.3 s; BOTH (stacked), q=0.06 9 excursions / 33.9 s; STOCK, q=1e-5 11 excursions / 13.9 s; GATED (node side), q=1e-5 12 excursions / 24.9 s; MEAS (measurement side), q=1e-5 10 excursions / 23.9 s; BOTH (stacked), q=1e-5 11 excursions / 29.5 s.

## 3 The 1527-1550 s episode: the gate re-anchors on a frozen heading and then holds through a turn

Per-frame dump: `episode_1527_1550.csv` (200 frames, 1520-1560 s).  Figure: `fig/Fig10_hold_in_turn.pdf`.

### 3.1 What the recording does

| span (s) | recorded heading | evidence |
|---|---|---|
| 1525.5-1527.1 | degrading: `sats` 21 -> 13 -> 10, `quality` 4 -> 5 -> 2, already ~30 deg from the course over ground, `hd_ok` = 1 throughout | `episode_1527_1550.csv` columns `sats`, `quality`, `course_minus_hdgyaw` |
| 1527.11-1527.51 | frozen at 242.585 deg (3 frames) | `dpsi_deg` = 0.000 with `hd_ok` = 1 |
| 1527.71-1529.32 | frozen at 270.584 deg (9 frames) while the vehicle turns | `dpsi_deg` = 0.000, `gz_max` 0.07 -> 0.28 rad/s |
| 1529.52-1530.53 | -155.3 deg step onto a **wrong** branch | course over ground disagrees by 76-121 deg |
| 1530.53 | +91.5 deg step back onto the correct branch | from here the recorded heading follows the course over ground: median 3.2 deg, max 7.5 deg while speed > 0.5 m/s, 1530.6-1549.5 s |
| 1530.7-1549.5 | correct; **the reference is good** | same check |

### 3.2 What the measurement-side gate does, frame by frame

Parameters actually in force (verified in `analysis/m8b/m8b.launch` line 61 and `analysis/m8/m8.launch`
line 133; the node default in `analysis/m8b/m8b_gate_node.py` is the same):
`max_yaw_rate` 1.234 rad/s, `nominal_dt` 0.2 s, `yaw_gate_margin` 0.05 rad (budget 17.005 deg/frame),
`max_hold_frames` 5, `max_gap_frames` 3, `use_gyro` true, `gyro_drift_rate` 0.002 rad/s,
**`max_gyro_ref_time` = 10.0 s**.  The offline re-run of `fig/gate.py` over this window reproduces the
online node's `state` for every frame (asserted in `episode_1527_1550.py`).

1. **1527.11-1527.51** -- the frozen 242.585 deg is 36.7 deg from the anchor: HOLD, HOLD, then the
   gap rule (`max_gap_frames` x `nominal_dt` = 0.6 s without an accept) invalidates the reference.
2. **1527.71 -- the decisive frame.** The second frozen value, 270.584 deg, is compared with the
   gyro-propagated anchor and misses it by only **9.44 deg**, inside the 17.10 deg allowance, because
   the vehicle had barely started to turn.  The gate **re-anchors on a frozen heading** (`reanchor` = 1).
3. **1527.92-1529.32** -- the same value repeats nine times.  Each repeat is 0.000 deg from the last
   accepted value, so each is an ACCEPT, and **each ACCEPT re-bases the gyro reference**.  The anchor
   therefore swallows the rotation that happens meanwhile: 0.3111 rad = **17.82 deg** between 1527.71
   and 1529.32 s.
4. **1529.52** -- the -155.3 deg step: 155.30 deg from the anchor, HOLD; 1529.72 HOLD; 1529.92 the gap
   rule invalidates again.  From here the gate publishes nothing to the EKF.
5. **1529.92-1539.36** -- re-anchor rule (a) is used on every frame, because `use_gyro` is true, the
   gyro is present and the age of the anchor is still below `max_gyro_ref_time`.  Rule (a) compares
   the incoming heading with the **frozen** anchor propagated by the gyro, so the residual is the
   anchor's own error and does not shrink: **44.5-57.1 deg** against a budget that creeps only from
   17.07 to 18.13 deg (the drift term).  At 1530.53 s, where the recording is back on the correct
   branch, the residual is 47.6 deg -- the anchor error, which decomposes as 17.8 deg absorbed while
   frozen + 9.4 deg accepted at the re-anchor frame + ~20 deg that the pre-freeze heading already
   carried (the course-over-ground check puts it ~28 deg off at 1526.91 s; the split of the last
   two terms is arithmetic, not an independent measurement).
6. **Rule (a) blocks rule (b).**  In `gate_series_v3` the two-consecutive-samples rule is only reached
   when the gyro reference is unavailable **or stale**.  So the gate cannot recover until the anchor
   is older than `max_gyro_ref_time`:

        last (bad) ACCEPT        t = 1529.321 s
        + max_gyro_ref_time      10.0 s
        first frame with age > 10 s: 1539.361 s (age 10.04 s) -> fallback_b, pending set, still NOPUB
        next frame 1539.561 s (age 10.24 s): |y - pending| = 4.51 deg <= 17.005 deg -> ACCEPT, re-anchor

   **10.24 s blocked**, of which **8.83 s (44 frames) rejected a correct heading**; 49 frames were
   withheld from the EKF (`orientation_covariance[0] = -1`) and 2 were HOLD.  The vehicle turned
   245.4 deg (gyro) between the bad anchor and the re-anchor.

### 3.3 What it costs the consumer, q = 0.06

Window **B = 1530.73-1539.56 s**: the gate is blocked and the reference is independently correct.

| arm | frames published | median dev (m) | max dev (m) |
|---|---|---|---|
| STOCK, q=0.06 | 42 | 0.0000 | 0.0146 |
| GATED (node side), q=0.06 | 28 | 0.5802 | 0.7295 |
| MEAS (measurement side), q=0.06 | 42 | 0.3214 | 0.3746 |
| BOTH (stacked), q=0.06 | 42 | 0.3192 | 0.3570 |

At q = 1e-5 the same window gives

| arm | frames published | median dev (m) | max dev (m) |
|---|---|---|---|
| STOCK, q=1e-5 | 42 | 0.0082 | 0.2145 |
| GATED (node side), q=1e-5 | 35 | 0.6044 | 0.7310 |
| MEAS (measurement side), q=1e-5 | 42 | 0.2762 | 0.3199 |
| BOTH (stacked), q=1e-5 | 42 | 0.2854 | 0.3352 |

The MEAS plateau is the lever arm rotated by a wrong yaw.  Inverting 2|L| sin(dpsi/2) with
|L| = 0.3655 m gives a yaw error of 52.2 deg at the median and 61.6 deg at the peak, against the
gate's 47.6 deg anchor error -- the remainder is the filter's own lag, since the EKF is coasting on
the gyro with no absolute heading.  STOCK, which simply uses the recorded heading, stays at
0.0000 m median / 0.0146 m max in the same window: **in this window the gate is the whole error**.

### 3.4 The stacked arm: where the 0.731 m comes from

At 1539.561 s the measurement-side gate re-anchors and hands the EKF a correct heading; the filter
yaw swings by 0.9808 rad = 56.20 deg.  0.08 s later the node-side gate reads the filter's own yaw,
sees that increment against its 0.2968 rad budget and **holds** (warning at 1539.642 s), then declares
the reference stale at 1540.039 s.  It keeps publishing `/odometry/gps` with the lever arm rotated by
the held (pre-correction, ~56 deg wrong) yaw while the vehicle keeps turning, so the error grows
along 2|L| sin(dpsi/2) until it saturates at the geometric bound 2|L| = 0.7311 m.  It re-anchors on two
consecutive agreeing samples at 1549.642 s -- **10.00 s after the hold**, again `max_gyro_ref_time`.

Window **C = 1539.60-1549.70 s**:

| arm | frames published | median dev (m) | max dev (m) |
|---|---|---|---|
| STOCK, q=0.06 | 50 | 0.0012 | 0.0276 |
| GATED (node side), q=0.06 | 50 | 0.0000 | 0.0354 |
| MEAS (measurement side), q=0.06 | 50 | 0.0007 | 0.0176 |
| BOTH (stacked), q=0.06 | 50 | 0.7147 | 0.7310 |

So the "0.73 m single point" of summary.md is not a point: for BOTH it is a **19.9 s excursion**
(1529.52-1549.40 s) that sits at 0.71-0.73 m for the last ten seconds, with every frame published and
every covariance nominal.  The GATED arm alone shows the same failure one window earlier
(hold 1529.587 s, stale 1529.982 s, `reanchor_two` 1539.476 s, again 10 s), with a median of
0.5802 m and 14 of the 42 consumer frames STOCK published in that window missing.

## 4 Injection runs: does Table 6 hide anything?

Per-event peaks are measured within 2 s of each of the 45 injections.  The table below puts the
largest of those next to the maximum over the **whole** run.

| run | events | per-event max (m) | full-run max (m) | at (s) | full-run max, frozen frames removed (m) | at (s) |
|---|---|---|---|---|---|---|
| STOCK, q=0.06 | 45 | 0.5170 | 0.5201 | 1530.53 | 0.5201 | 1530.53 |
| GATED (node side), q=0.06 | 45 | 0.0922 | 0.7311 | 1535.35 | 0.7311 | 1535.35 |
| MEAS (measurement side), q=0.06 | 45 | 0.0720 | 0.7257 | 1433.54 | 0.7257 | 1433.54 |
| BOTH (stacked), q=0.06 | 45 | 0.0691 | 0.7310 | 1548.79 | 0.7310 | 1548.79 |
| STOCK, q=1e-5 | 45 | 0.3244 | 0.7278 | 1431.53 | 0.7278 | 1431.53 |
| GATED (node side), q=1e-5 | 45 | 0.3227 | 0.7311 | 1535.75 | 0.7311 | 1535.75 |
| MEAS (measurement side), q=1e-5 | 45 | 0.1289 | 0.7252 | 1433.54 | 0.7252 | 1433.54 |
| BOTH (stacked), q=1e-5 | 45 | 0.1084 | 0.7311 | 1548.99 | 0.7311 | 1548.99 |
| STOCK, q=1e-3 | 45 | 0.5168 | 0.6541 | 1440.56 | 0.6541 | 1440.56 |
| GATED (node side), q=1e-3 | 45 | 0.1290 | 0.7260 | 1434.54 | 0.7130 | 1529.52 |
| MEAS (measurement side), q=1e-3 | 45 | 0.0662 | 0.7257 | 1433.54 | 0.7257 | 1433.54 |
| BOTH (stacked), q=1e-3 | 45 | 0.0664 | 0.7311 | 1548.79 | 0.7311 | 1548.79 |
| STOCK, q=1e-4 | 45 | 0.4848 | 0.6972 | 1436.35 | 0.6972 | 1436.35 |
| GATED (node side), q=1e-4 | 45 | 0.2546 | 0.7311 | 1431.93 | 0.7311 | 1431.53 |
| MEAS (measurement side), q=1e-4 | 45 | 0.0689 | 0.7257 | 1433.54 | 0.7257 | 1433.54 |
| BOTH (stacked), q=1e-4 | 45 | 0.0683 | 0.7311 | 1548.99 | 0.7311 | 1548.99 |

Every injection run reaches 0.65-0.73 m somewhere, and in **every** case the full-run maximum sits
outside the injections, in the 1313-1320 s, 1427-1443 s or 1527-1550 s natural windows.  Across the
16 injection runs there are 147 excursions above 0.10 m that are not attributable to an injection
(`full_cost_excursions.csv`).  The per-event numbers themselves are unaffected -- the injections start
at 1585.8 s, after all three windows -- but the sentence they support ("the consumer is bounded at
every gain") is not something the injection table can carry.

## 5 What the earlier "~= zero cost" claim got wrong

`data/m8b_measurement_gate/summary.md` table 7 and `design.md` section 7c state that after removing the
ten frozen-heading episodes the measurement-side gate costs p99 0.0168 m against STOCK's 0.0162 m,
i.e. nothing.  Three things are wrong with that reading.

1. **The guard removed the cost, not just the bad reference.**  The exclusion is -2 s / +12 s around
   each episode.  Episode 9 is 1527.31-1529.32 s, so the guard deletes 1525.31-1541.32 s -- which is
   exactly the 10.24 s in which the gate, having re-anchored on the frozen value, rejects correct
   headings and leaves the consumer 0.32-0.37 m off.  The reference is bad for the first ~1 s of
   that span and good for the remaining ~8.8 s.  The guard was justified for the frozen frames; it
   was not justified for the recovery that follows, and it is the recovery that carries the cost.
2. **"Bounded at every gain" is the wrong claim.**  What the four-gain sweep shows is that the gate
   *decision* is gain-independent (0 differences across 162 249 frames, table 6), and that the
   *injected* 90 deg outliers are suppressed at every gain.  It does not show that the consumer is
   bounded, because the gate can lock onto a wrong anchor and then withhold for as long as
   `max_gyro_ref_time`; the resulting error is bounded only by 2|L| = 0.731 m, which BOTH reaches.
3. **The 0.73 m of the stacked arm is not an isolated sample.**  It is a 19.9 s excursion with a
   10 s plateau at the geometric bound, and the same mechanism -- a 10 s re-anchor timeout applied to
   an anchor that is itself wrong -- produces it on the GATED arm alone as well.  The conclusion of
   section 4.9 (put the guard on the measurement side, do not stack the two) survives; the claim
   that the measurement side is free does not.

What can honestly be said, on this recording: the measurement-side gate suppresses every injected
outlier at every gain, its decision does not depend on the filter gain, its median cost over the run
is 0.0003 m (q = 0.06) and 0.0023 m (q = 1e-5), and it has one failure mode with a measured cost of
0.37 m for 8.8 s: a heading that freezes at a value the gyro cannot distinguish from the truth at
that instant is adopted as the anchor, and `max_gyro_ref_time` then sets how long the gate stays
wrong.

## 6 Not verified

* **Why the node-side gate keeps publishing while it is invalid.**  The patch (`patch/heading_gate.patch`)
  sets `heading_gate_publish_ = false` in the invalid branch, yet BOTH publishes 50/50 consumer frames
  through 1539.6-1549.7 s on a held yaw (GATED does withhold 14 frames in its own window).  The node only
  logs `ROS_WARN_THROTTLE(1.0, ...)`, so its per-frame state was not recorded and the reconstruction in
  `episode_1527_1550.csv` (`node_state_both`, `node_state_gated`) is from the throttled warnings only.
  **UNVERIFIED** -- settling it needs a re-run with per-frame logging on the node side.
* **The absolute truth of the heading before 1525.5 s.**  The course-over-ground check needs motion;
  the vehicle is nearly still before 1524.9 s, so the ~20 deg pre-existing anchor error in section 3.2
  is inferred from the gyro arithmetic and corroborated, not measured. **UNVERIFIED**.
* **Whether a different `max_gyro_ref_time` fixes it.**  No sweep was run; a shorter timeout would
  shorten this block but also weaken rule (a) against real faults. **UNVERIFIED**.
* **Speeds quoted from the reference** (3-4 m/s through the turn) come from differencing the recorded
  `/rtk_odom`; they were not checked against wheel odometry. **UNVERIFIED** (nothing in this file
  depends on them).
* Everything here is replay of one recording (bag9, 2026-09-09 orchard); no vehicle in the loop.

