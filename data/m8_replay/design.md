# M8 — Replay design: heading-continuity gate on the robot_localization live chain

> Written 0920, **frozen before any run**. Purpose: test the "untested" statement in
> manuscript §4.8 / §4.10 -- navsat_transform takes its heading from the filter, so an accepted
> outlier may leak in at a rate **below the fixed budget**.
> All machine-side files are written under a scratch directory (`$P2_WORK`) on the vehicle PC; the production
> install is not touched, no sudo/apt, no bag leaves the machine.

## 1 Chain

```
bag ──(rosbag play --clock, /rtk_odom /ch110/data_raw /odom)──> m8_republish.py
   ├─ /gps/fix          NavSatFix   antenna LLH (lever arm inverted, then back to geodetic) 5 Hz
   ├─ /rtk_heading_imu  Imu         RTK heading as orientation (**the only injected stream**) 5 Hz
   ├─ /ch110_imu        Imu         CH110 ωz                                        ~93 Hz
   └─ /navsat_imu       Imu         heading + ωz (navsat's imu/data)                ~93 Hz
        │
        ├──> ekf_localization_node (2D) ──> /odometry/filtered + tf odom→base_link
        │        odom0=/odom (vx, vyaw)  imu0=/rtk_heading_imu (yaw)  imu1=/ch110_imu (yaw rate)
        │
        └──> navsat_transform_node ──> /odometry/gps  ← downstream consumer, where the
                 tf base_link→gps = (0.235, −0.280, −0.175) m      lever-arm rotation lands
                 (measured, rtk_odom.launch 2026-08-14)
```

The antenna point is recovered by inverting §4.2: `p_ant = p_pub + R_z(ψ)·L`, then converted back
to LLH with the driver's ENU origin. The injection **changes only the heading stream**, not the
antenna position -- the lever-arm pseudo-step is a pure heading fault. navsat_transform takes its
yaw from tf (world→base_link), i.e. **the filter's yaw**, which is exactly the difference under test.

## 2 Three arms

| arm | workspace | gate | EKF Mahalanobis gate `imu0_pose_rejection_threshold` |
|---|---|---|---|
| STOCK | `ws_stock` (unpatched, 040dd17) | none | off (1e12) |
| GATED | `ws_gate` (patched) | v3, vehicle parameters | off (1e12) |
| MAHAL | `ws_stock` | none | **3.0** (3σ) |

Gate parameters (same as manuscript §4.8): `max_yaw_rate` 1.234 rad/s, `nominal_dt` 0.2 s,
`yaw_gate_margin` 0.05 rad ⇒ fixed budget g = 0.2968 rad = **17.0°/frame**;
`max_hold_frames` 5, `max_gap_frames` 3, `use_gyro` true, `gyro_drift_rate` 0.002,
`max_gyro_ref_time` 10 s. STOCK runs the **unpatched binary**, not a loosened parameter set.

## 3 Gain sweep (answering the leakage statement)

The EKF Kalman gain on heading ≈ P/(P+R), with R = the driver-declared (0.5°)² = 7.615e-5 rad²
and P growing each frame with the process noise q. So **EKF yaw process noise q** is the gain
knob, everything else left at robot_localization defaults:

| q (`process_noise_covariance[5][5]`) | note | per-frame gain (0.2 s, order of magnitude) |
|---|---|---|
| 0.06 | robot_localization factory default | ≈0.99 (near pass-through) |
| 1e-3 | tuned attitude channel | ≈0.7 |
| 1e-4 | good gyro, heading down-weighted | ≈0.2 |
| 1e-5 | heading strongly down-weighted | ≈0.03 |

STOCK and GATED each run all four q values; MAHAL runs q=0.06 and 1e-4 only.

## 4 Input and injection

- Main bag: `run_20260909_2026-09-09-07-56-42.bag` (orchard operation, 33 min, 5 Hz `/rtk_odom`).
- **Synthetic injections** (explicitly labelled replay injections, not recorded faults):
  amplitudes 5/10/20/45/90°, durations 1/2/5 frames, 3 epochs each = **45 injections**,
  all inside IMU-classified straight segments (`cls_imu == straight`, classified by gyro, not by
  heading), spaced 12 s apart; 984-994 s is avoided because it holds 4 natural pseudo-steps.
  List in `events.csv`.
- **Natural events**: two windows of `run_20260906_2026-09-06-15-26-15.bag`
  (bag offset ≈1790-1890 s, containing the headline −67.5° event; ≈460-560 s, containing the
  −167.5° event), no injection.
- Each arm is also run once with `inject:=false` as its own baseline.

## 5 Metrics (consumer side, `/odometry/gps`)

The reference is the **recorded `/rtk_odom`** (base_link position of the same frame, not
contaminated by the injection). The two frames differ by a constant translation only (datum
fixed), estimated away with the whole-run median:

- `dev(t) = |(p_gps(t) + o) − p_rtk(t)|`, with `o` = median of `(p_rtk − p_gps)` over the run;
- **event-frame step** = increment of `dev` at the first event frame over the preceding frame;
- **peak deviation** = max `dev` within 2 s after the event; the same-window peak from the arm's
  baseline run is reported alongside;
- **recovery time** = time after the peak for `dev` to fall back to max(2 × baseline median, 0.02 m);
- **withheld** = a fix at a timestamp that is present in the STOCK baseline and absent in this arm;
- **gate action count** = navsat hold / stale / invalid / reanchor in `/rosout_agg`
  (`ROS_WARN_THROTTLE(1.0)` suppresses repeats within 1 s, so this is qualitative only;
  quantitative statements use withheld frames and ψ_used);
- **false hold** = gate actions and withheld frames further than ±3 s from any injection.

**Consistency check of the C++ against `fig/gate.py`**: the heading the gate actually used can be
inverted from the published result -- `p_ant − p_gps = R(ψ_used)·L`, solved for `ψ_used` in 2D;
the `/odometry/filtered` yaw is linearly interpolated onto the fix timestamps (that is how the tf
lookup interpolates) and fed to `gate.py`, then `published` and `used_yaw` are compared frame by
frame. The two rule sets must agree; both are given the same parameter values.

## 6 Run discipline

- Each arm gets its own `ROS_MASTER_URI` port and `ROS_HOME=$P2_WORK/roshome/<tag>`
  (the default ROS home is never written).
- Each process runs under `systemd-run --user --scope -q -p MemoryMax=4G --`;
  `rosbag play --clock --hz=1000 -r 6` (see §7).
- Subscriptions use raw messages (`recorder.py`); `rostopic echo` text is never parsed.
- Only CSVs are pulled back into `data/m8_replay/`; bags stay on the machine, and geodetic
  coordinates stay on the machine (the datum is read on the vehicle PC only).

## 7 Replay rate and determinism check (measured 0920, before the production matrix)

Fast replay is used only after showing it costs nothing. Check bag `bag9_slim`, window
1100-1220 s (no natural event, no injection), one arm (STOCK, q=0.06, no injection), varying
only `rosbag play -r`.

**Two sources of nondeterminism were fixed first** (without the fixes the conclusion would be false):

1. `--clock` defaults to 100 Hz (wall clock). At `-r 6` each tick advances simulated time by
   60 ms, so the EKF's 30 Hz cycle cannot keep up and the `/odometry/filtered` count drops with
   the rate (1×/4×/6×/8× = 2780/2332/1629/1258). With `--clock --hz=1000` it becomes
   2780/2762/2754/2753.
2. With `wait_for_datum:=false` the datum is taken as "first fix + the filter yaw at that moment",
   so a different start time rotates the whole cartesian→world transform and distant points move
   by metres. With a **fixed datum** (the driver's ENU origin, yaw=0), the constant offset of
   (p_rtk − p_gps) is the same value (0.432, 0.056) m at every rate.

**No input frames dropped** (republisher count = recorder count):

| rate | /rtk_odom received | recorder logged | /odometry/gps | /odometry/filtered |
|---|---|---|---|---|
| 1× | 600 | 599 | 594 | 2225 |
| 2× | 597 | 597 | 590 | 2253 |
| 4× | 597 | 597 | 586 | 2127 |
| 6× | 597 | 596 | 584 | 2057 |
| 8× | 595 | 593 | 579 | 1935 |

(Window 120 s × 5 Hz = 600 frames. Higher rates miss 1-3 frames at the start because startup
consumes more simulated time, not because frames are dropped: within each run in ≈ rec, and the
largest frame interval is 0.401 s with no gap. The final frame is lost when the nodes shut down.)

**Trajectory consistency**: **a 1 mm criterion is unreachable on this chain at any rate, 1× against
itself included**. The same arm run twice (both 1×): median difference 0.000000 m, p99 9.2 mm,
max 21.5 mm, 135 of 577 frames >1 mm. The cause is non-reproducible message scheduling and tf
interpolation, not the replay rate. The criterion is therefore restated as
**"the difference introduced by fast replay must not exceed the difference between repeats at the
same rate"**:

| comparison | median (m) | p99 (m) | max (m) |
|---|---|---|---|
| 1× vs 1× (baseline) | 0.000000 | 0.0092 | 0.0215 |
| 1× vs 4× | 0.000000 | 0.0109 | 0.0327 |
| 4× vs 4× | 0.000000 | 0.0118 | 0.0327 |
| 1× vs 6× | 0.000001 | 0.0078 | 0.0254 |
| 6× vs 6× | 0.000001 | 0.0147 | 0.0454 |

The 1× vs 6× difference (p99 7.8 mm) is smaller than the baseline of 6× repeated against itself
(14.7 mm). **6× is adopted.** Baseline noise ≈ 1 cm (p99) / 4.5 cm (max), an order of magnitude
below the effect under test (0.2-0.52 m) and below the 0.18 m upper bound on hold cost. The
production matrix is 22 replays, 8 in parallel, each with its own master port and output directory.
