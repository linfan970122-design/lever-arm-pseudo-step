# M8b — replay design: the same gate rule moved to the **measurement side** (EKF input)

> Written 2026-09-20, **finalised before the runs** (the verification numbers in §7 were filled in afterwards; nothing else was changed).
> Motivation: M8 found that the node-side gate (inside `navsat_transform`, reading the **filter** yaw) never fires when the gain is low —
> at q=1e-5 the filter spreads a single 90° outlier into 14.0–14.7° per frame, below the fixed budget of 17.0°/frame, so GATED ≡ STOCK.
> Hypothesis: **moving the same rule to the measurement side** (judging on the raw heading stream, before the outlier reaches the EKF) makes the criterion independent of the filter gain
> and bounds the consumer position at every q.
> On the machine side only the scratch directory `$P2_WORK` on the vehicle PC is written; the deployed fusion stack is untouched, no sudo/apt is used, and neither the recordings nor the geodetic datum leave the machine.

## 1 Where the new arm sits in the chain

```
bag ──> m8_republish.py
   ├─ /gps/fix            NavSatFix  antenna LLH                            5 Hz
   ├─ /rtk_heading_imu    Imu        RTK heading (**the only injected stream**)  5 Hz
   │        │
   │        └──> ⭐ m8b_gate_node.py (MEAS-GATE, new) ──> /rtk_heading_gated ───────┐
   ├─ /ch110_imu          Imu        CH110 ωz (gyro reference of the gate)  ~93 Hz  │
   └─ /navsat_imu         Imu        heading + ωz (navsat imu/data)         ~93 Hz  │
                                                                                    │
   imu0 = /rtk_heading_gated (MEAS/BOTH) or /rtk_heading_imu (STOCK/GATED)  ────────┘
        └──> ekf_localization_node(2D) ──> /odometry/filtered + tf
                 └──> navsat_transform_node ──> /odometry/gps  ← consumer
```

Apart from pointing imu0 at the gated topic and adding one Python node, **everything else is parameter for parameter identical to the M8 STOCK**:
the unpatched `ws_stock` binaries, `max_yaw_rate=1e9` (the node-side gate is effectively absent),
`imu0_pose_rejection_threshold=1e12` (Mahalanobis gate off), fixed datum, `use_local_cartesian:=true`.

Four arms:

| arm | Workspace | Measurement-side gate | Node-side gate (navsat patch) | EKF Mahalanobis gate |
|---|---|---|---|---|
| STOCK (already in M8, quoted directly) | `ws_stock` | none | none | off |
| GATED (already in M8, quoted directly) | `ws_gate` | none | v3 | off |
| MAHAL (already in M8, quoted directly) | `ws_stock` | none | none | 3σ |
| **MEAS-GATE** (this run) | `ws_stock` | **v3** | none | off |
| **MEAS+NODE** (this run) | `ws_gate` | **v3** | v3 | off |

STOCK / GATED / MAHAL are **not re-run**; the comparison uses the existing results in `data/m8_replay/per_event.csv`
(same recording, same `events.csv`, same 6× rate, same metric scripts).

## 2 Gate rule and parameters: the same implementation as GATED, word for word

🔴 **No second copy of the rule is written**: the verbatim `fig/gate.py` (md5 in §7) is copied as a whole to `$P2_WORK/gate.py` on the vehicle PC,
and the node calls `gate.gate_series_v3(...)` after `import gate`, with parameters exactly as in GATED:

```
max_yaw_rate 1.234 rad/s   nominal_dt 0.2 s   yaw_gate_margin 0.05 rad
  ⇒ fixed budget g = 0.2968 rad = 17.0°/frame
max_hold_frames 5   max_gap_frames 3   use_gyro true
gyro_drift_rate 0.002 rad/s   max_gyro_ref_time 10 s
```

`gate_series_v3` is a whole-segment batch function; an online node **cannot** re-run the whole segment for every frame (O(n²)).
A **state equivalence** turns it into O(1) amortised without touching a single line of the function:

> After `gate_series_v3` processes an ACCEPT frame, its internal state is always
> `(last_accepted=y_i, last_t=t_i, anchor=i, hold_count=0, valid=True, pending=None)`,
> which is **exactly the same** as its state after handling the **first frame** (accepted unconditionally).
> Therefore: truncate the buffer at the most recent ACCEPT frame, call the function once on `[k..i]`,
> and the result for frame `i` matches the whole-segment call frame by frame.

The node does exactly that: the buffer starts at the most recent ACCEPT, every incoming frame is appended, the function is called once on the whole buffer, and the last element is taken;
when that last element is an ACCEPT the buffer is truncated to one frame. Buffer length = number of consecutive non-ACCEPT frames (normally ≤5);
above 500 frames a warning is logged (for observation only, the rule is unchanged). cum_gyro is used only as a difference, so truncation has no effect.

The gyro is integrated the same way as in `republish.py` (rectangular integration, same sign as the C++ `gyroCallback`):
`gz_cum += ω_z·Δt`, taking the latest value at the arrival of that heading frame.

## 3 Withholding mechanism: `orientation_covariance[0] = -1`, not an inflated covariance

In the source version actually used here (`robot_localization` noetic-devel **040dd17**,
`src/ros_filter.cpp:516`, `RosFilter::imuCallback`):

```cpp
if (::fabs(msg->orientation_covariance[0] + 1) < 1e-9)
{ RF_DEBUG("Received IMU message with -1 as its first covariance value for orientation. Ignoring orientation..."); }
else { /* use orientation as a pose measurement */ }
```

That is: when `orientation_covariance[0] == -1` the **whole orientation measurement is skipped**, while the message itself still arrives
(no effect on `sensor_timeout`, no effect on the imu1 angular-rate channel). This is the convention of the `sensor_msgs/Imu` message
specification itself, which the node follows explicitly, and it is exactly **"do not fuse yaw on this frame"**.

**Why not inflate the covariance**: the gain is ≈ P/(P+R), so inflating R only shrinks the gain without zeroing it;
at q=0.06 P is large, and however large R is some residual fusion remains, which leaves the bound unstated. `-1` is an exact, citable switch.

The three gate decisions → what is published on `/rtk_heading_gated`:

| `gate_series_v3` state | What is published | What the EKF gets |
|---|---|---|
| ACCEPT | the heading of that frame, covariance unchanged (σ=0.5°) | fused normally |
| HOLD | **the last accepted heading** (`used_yaw`), covariance unchanged | fuses one frame of a "stale but steady" heading |
| NOPUB (after 5 consecutive hold frames, or invalid and not re-anchored) | the original message but with `orientation_covariance[0] = -1` | **yaw is not fused on this frame** |

HOLD still publishes `used_yaw` so that the rule is **word for word the same** as the node-side gate (which also publishes the last accepted heading on HOLD).
🔴 This carries a known cost: while the vehicle is turning, HOLD pushes a stale absolute heading into the EKF against the gyro,
for at most 5 frames = 1 s; this is exactly what the hold-cost upper bound (≤0.18 m) describes, and it is measured here as well.

## 4 Inputs and injection: item by item the same as M8

- Main recording `run_20260909_2026-09-09-07-56-42.bag` (`$P2_WORK/bag9_slim.bag`),
  45 synthetic injections (5/10/20/45/90° × 1/2/5 frames × 3 epochs), the list being the M8 `$P2_WORK/events.csv`;
- Natural events: two windows of the 0906 recording (−67.5° at bag offset 1790–1900 s, −167.5° at 460–570 s), no injection;
- Each arm gets one `inject:=false` baseline run;
- Replay at 6×, `--clock --hz=1000`, fixed datum, `use_local_cartesian:=true`.

## 5 Replay matrix (24 runs + 1 smoke test)

| Batch | Runs | Recording | Notes |
|---|---|---|---|
| b1 | `inj_meas_q060/q1e3/q1e4/q1e5` + `bas_meas_*` | bag9 | MEAS-GATE at four q settings, injection + baseline (8 runs) |
| b2 | `inj_both_q060/q1e3/q1e4/q1e5` + `bas_both_*` | bag9 | MEAS+NODE at four q settings (8 runs) |
| b3 | `nat1_meas_q060`, `nat2_meas_q060`, `nat1_meas_q1e4`, `nat1_both_q060`, `nat2_both_q060` | bag6 | natural events (5 runs) |
| b3 | `ver_meas_6x_a`, `ver_meas_6x_b` | bag9 window 1100–1220 s | the two 6× runs of the §7b rate verification (same batch as b3) |
| b4 | `ver_meas_1x` | bag9 window 1100–1220 s | the 1× run of the §7b rate verification (run alone, not competing for CPU in parallel) |

A separate smoke run (`smk_meas_q060`, bag9 1725–1770 s, containing three 90° injections) is used only to check the node and topic wiring and does not enter the results.

Each run uses its own `ROS_MASTER_URI` port (11450+) and its own `ROS_HOME=$P2_WORK/roshome/<tag>`,
and every process runs under `systemd-run --user --scope -q -p MemoryMax=4G --`.

## 6 Metrics (the same script definitions as M8)

Consumer `/odometry/gps`, reference the recorded `/rtk_odom`; the constant translation between the two frames is estimated away with the whole-segment median:

- **Peak deviation** = maximum `dev` within 2 s after the event; **recovery time** = time from the peak back to `max(2 × baseline median, 0.02 m)`;
- **Baseline median / p99** = the baseline of the no-injection run of the same arm;
- **Withheld outside injection**: two quantities reported separately —
  - `withheld_pos`: frames the consumer `/odometry/gps` did not publish (expected to be identically 0 for MEAS-GATE, since the gate is on the measurement side and navsat publishes as usual);
  - `withheld_meas`: heading measurement frames the measurement-side gate did not hand to the EKF (NOPUB), counted from `meas_gate.csv`;
  - `held_meas`: HOLD frames (a stale heading was handed over);
- All false triggers are counted "outside ±3 s of any injection epoch"; the 10 `hd_ok=0` epochs in `exclude.csv` are likewise removed.

## 7 Verification (filled in after the runs)

**a) Equivalence (the focus of this run, the one thing M8 could not do)**
The M8 gate is inside the node and its input is not recorded, so only interpolated `/odometry/filtered` could serve as a proxy, matching only 98.6 % frame by frame.
The M8b gate is outside the node and **its input is the recorded `/rtk_heading_imu`**; the node logs every frame as
`(t, yaw_in, gz_cum, state, published, used_yaw)`; offline the same `(t, yaw_in, gz_cum)`
is fed to `gate_series_v3` of `fig/gate.py` and compared frame by frame on `state/published/used_yaw`.
🔴 **Criterion: mismatches must be 0.** If it is not 0, the raw output is pasted as is, nothing hidden.

- `fig/gate.py` md5 (local): `9887f5cb97420f40609c21bf78f8089e`
- `$P2_WORK/gate.py` md5 at the time of the runs: `9887f5cb97420f40609c21bf78f8089e` (byte for byte
  identical to `fig/gate.py` as it then stood). For publication three comment lines of
  `fig/gate.py` were translated into English, so the file in this repository is
  `69e26f2f11a89b85571177ee6ed6ff06`; no statement, parameter or value was touched.
- Frame-by-frame mismatches: **0**. Over 24 replays and **162 249 frames**, `state` / `published` / `used_yaw` all show 0 mismatches
  (`used_yaw` criterion >1e-9 rad). Detail in `gate_equivalence.csv`. Maximum online buffer 53 frames.

**b) Fast replay does not degrade quality** (criterion follows §7 of the M8 `design.md`: **the difference introduced by fast replay ≤ the difference between repeats at the same rate**;
a 1 mm criterion is unreachable on this chain at any rate, not even 1× against itself).
This run has one more Python node in the loop, so it must be verified again:

| Comparison | Median (m) | p99 (m) | Max (m) |
|---|---|---|---|
| 6× vs 6× (baseline) | 0.000002 | 0.0144 | 0.0258 |
| 1× vs 6× (a) | 0.000001 | 0.0107 | 0.0259 |
| 1× vs 6× (b) | 0.000000 | 0.0109 | 0.2095 |
| 6× vs 6× (first 2 s of the common window dropped) | 0.000002 | 0.0144 | 0.0258 |
| 1× vs 6× (first 2 s dropped, two runs) | 0.000000–0.000001 | 0.0091 / 0.0108 | 0.0200 / 0.0259 |

The 0.2095 m in (b) falls in the first 3 frames of the comparison window: the two replays started 0.4 s apart, so the filter had converged to a different degree;
it is not a rate effect (the gate decisions on those 3 frames are identical in both runs, all ACCEPT). **The criterion is met**, 6× is fixed.

**c) Supporting evidence (filled in after the runs)**:
- 5°/10° injections (below the 17.0° budget): MEAS-GATE acts **0/18 times**, peak 0.016–0.064 m, the same as STOCK at every setting;
  for 20°/45°/90° it acts on **27/27**. These two numbers are **exactly the same** at all four q settings.
- Baseline median 0.0003 m (STOCK 0.0001 m); the baseline p99 looks like 0.122 m (STOCK 0.017 m),
  but after removing the 10 segments where the recorded heading is frozen or 0 it is **0.0168 m against STOCK 0.0162 m** (`baseline_cost_m8b.py`, Table 7 of summary.md) —
  in those segments the reference `/rtk_odom` itself rotated the lever arm with the bad heading, so an arm that rejects the bad heading is scored as deviating.
