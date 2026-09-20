# Heading-continuity gate — design note (`heading_gate.patch`, **rule v3**)

> Applies to `cra-ros-pkg/robot_localization`, branch `noetic-devel` (branch head `9ef26a5`; this note describes the patch against commit `040dd17`).
> All upstream line numbers quoted below were checked against the upstream source at the commit named above. **Not compiled, not run under ROS.**

**Version history (every revision was forced by replay, none of it was designed up front)**
- **v1**: budget `max_yaw_rate·(t − last_accepted_t) + margin`. Hole: a rejected frame does not update `last_accepted_t`, so a run of consecutive rejections keeps widening the budget, and in the end it let a **−178.9°** outlier through.
- **v2**: capped that time difference at `max_hold_frames·nominal_dt`, and added an invalid state plus a two-frame re-anchor. The hole remained: **the budget still grew with dt**, only more slowly. Replay showed it hid **event 2** — by the time of the jump, the budget had already widened enough to contain it.
- **v3 (this version)**: **the budget no longer scales with time at all**; it is fixed at `g = max_yaw_rate·nominal_dt + yaw_gate_margin`. A stale reference is handled by a **separate expiry rule** (`max_gap_frames`). Re-anchoring prefers a **gyro-propagated** reference, and that gyro reference is given a **time limit** `max_gyro_ref_time`.

🔴 **Consistency requirement**: this patch must agree with its Python twin `fig/gate.py` **statement by statement and parameter name by parameter name** — the same consistency rule adopted in the companion paper. **A line-by-line audit was completed on 2026-09-19**: all 14 items of the rule itself agree; one **real difference** was found and fixed (same-stamp de-duplication, item 1 under "Two things the patch adds beyond the specification"); after the fix, the C++ logic was re-run statement by statement inside Python over 364 799 frames x 3 variants, with **0 disagreements in publish decision and in the heading used**. What remains is a **parameter-value** mismatch (`max_gyro_ref_time`), not a rule mismatch.

---

## 1. The v3 rule (the single authoritative statement; both C++ and Python follow it)

**Parameters**: `max_yaw_rate`(rad/s), `yaw_gate_margin`(rad), `nominal_dt`(s), `max_hold_frames`(int), `max_gap_frames`(int), `use_gyro`(bool), `gyro_drift_rate`(rad/s), `max_gyro_ref_time`(s)
**State**: `last_accepted_yaw`, `last_accepted_t`, `hold_count`, `valid`, `pending(yaw, t)`, `gyro_yaw`

For every incoming position frame (`y` = the heading about to rotate the lever arm, at time `t`):

```
g = max_yaw_rate · nominal_dt + yaw_gate_margin          # fixed, does not scale with dt

# expiry rule (new in v3)
if valid and (t − last_accepted_t) > max_gap_frames · nominal_dt:
    valid = false, pending = none                        # reference too old to bound anything

if valid:
    if |wrap(y − last_accepted_yaw)| ≤ g:
        accept → use y, last_accepted = (y,t), hold_count = 0, publish
    else:
        hold → use last_accepted_yaw, hold_count += 1
        if hold_count ≥ max_hold_frames:
            valid = false, pending = none, withhold this frame
        else:
            publish this frame (with the held heading)
else:
    withhold this frame (unless a re-anchor below succeeds)
    # (a) gyro reference first, and only inside its time window
    if use_gyro and gyro available and (t − last_accepted_t) ≤ max_gyro_ref_time:
        if |wrap(y − gyro_yaw)| ≤ g + gyro_drift_rate·(t − last_accepted_t):
            re-anchor → accept y, publish
        # otherwise this frame is simply rejected; no fallback to (b)
    # (b) when no gyro reference is available: two frames must agree
    else:
        if pending is none:      pending = (y,t)
        elif |wrap(y − pending.yaw)| ≤ g:   re-anchor → accept y, publish
        else:                     pending = (y,t)
```

`gyro_yaw`: `last_accepted_yaw` propagated forward from `last_accepted_t` using the IMU ωz; every "accept" re-seeds it to the heading just accepted. It is maintained only when `use_gyro=true` and an IMU topic is present.

### Three changes from v2 to v3

1. **The budget loses its time scaling.** The `min(Δt, cap)` of v2 still let the gate widen while a fault persisted (it merely capped how far), and that was exactly the crack that hid event 2. The v3 `g` is a constant: however long the fault lasts, the gate stays the same width.
2. **Reference expiry becomes its own rule.** The budget no longer carries the job of "loosen because the reference is old"; `max_gap_frames` declares the state invalid directly, so dropped or stalled position frames can no longer prise the gate open quietly.
3. **Re-anchoring gains a gyro reference with a time limit.** A gyro is a **third party independent of the quantity under test**, which is far stronger than "two frames agree with each other". But bias drifts, so the gyro is trusted only within `max_gyro_ref_time` seconds, with the tolerance widening at `gyro_drift_rate`.

### Two things the patch adds beyond the specification (required for deployment)

1. **Same-stamp de-duplication**: `periodicUpdate()` calls `prepareGpsOdometry()` every cycle, while `gps_updated_` is cleared only after a successful publish (line 860), so **the same GPS stamp can be fed through the gate more than once**. Without de-duplication, in the invalid state the second sight of the same frame gives `|wrap(y − pending.yaw)| = 0 ≤ g`, and one frame masquerades as "two frames agree" and re-anchors immediately.

   🔴 **Correction (2026-09-19)**: this was originally written as "if `stamp <= last_gated_time_`, replay the previous decision", with the claim that it was "**a no-op on replay, since stamps increase strictly**" — **that claim was wrong**. In the 364 799 replay frames, **2 757 frames (0.756 %) carry the same timestamp as the previous frame, and all 47 bags contain such frames**; only 4 of them also repeat the heading. Among same-stamp neighbours the median heading difference is 0.283° and the maximum 91.5°, i.e. **two distinct samples sharing one stamp, not one frame seen twice**. Under the original wording those frames would have been discarded as duplicates, the reference would have stopped refreshing, and 0.4 s later the expiry rule would have declared it invalid and withheld the output — **the gate manufacturing a fault of its own**. Measured against `gate.py` over 364 799 frames, that wording disagreed in 19–20 publish decisions and in 2 757 headings used.
   **Now changed to**: a duplicate is the same sample seen twice, i.e. **same stamp and same heading**: `stamp <= last_gated_time_ && yaw == last_gated_yaw_` (new member `last_gated_yaw_`). This still closes the "one frame masquerading as two agreeing frames" hole while letting distinct samples that share a stamp pass through the gate normally. After the fix, agreement with `gate.py` is **frame-by-frame exact** over 364 799 frames x 3 variants.
2. **The gyro needs its own subscription**: the node's existing `imu_sub_` is `shutdown()` once the datum has been computed (line 214), so when `use_gyro=true` a lightweight second subscription `gyro_sub_` (also on `imu/data`) is opened, doing nothing but ωz integration.

---

## 2. What the patch does at code level

Two files change: **312 lines added, 1 removed (net +311)**, including Doxygen and comments; the logic alone is about 110 lines. After patching, `navsat_transform.cpp` is 1121 lines and `navsat_transform.h` 508 lines (upstream: 932 + 386). These counts were re-measured after the same-stamp de-duplication fix of 2026-09-19.

| File | Change |
|---|---|
| `include/robot_localization/navsat_transform.h` | three declarations `gateHeading()` / `acceptHeading()` / `gyroCallback()` + `gyro_sub_` + 21 members (8 parameters + 13 state/cache) |
| `src/navsat_transform.cpp` | constructor initialiser list, 9 `nh_priv.param()` calls, the `gyro_sub_` subscription, three function definitions, 1 default-value line + 2 call lines in `getRobotOriginWorldPose()`, and one early return in `prepareGpsOdometry()` |

### Parameters (eight; names must match `fig/gate.py`)

| Parameter | Default | Meaning |
|---|---|---|
| `max_yaw_rate` | 0.5 rad/s | physical turn-rate limit of the platform, ω_max |
| `yaw_gate_margin` | 0.05 rad (2.9°) | constant margin covering heading noise and timestamp jitter |
| `nominal_dt` | 0.2 s | nominal frame interval (`/rtk_odom` on this vehicle runs at 5 Hz) |
| `max_hold_frames` | 5 | number of consecutive rejections before the state is declared invalid |
| `max_gap_frames` | 3 | nominal intervals without a refresh before the reference expires (= 0.6 s) |
| `use_gyro` | false | use the gyro as the re-anchor reference (off by default, to keep the change to upstream behaviour minimal) |
| `gyro_drift_rate` | 0.002 rad/s | tolerance growth rate of the gyro reference |
| `max_gyro_ref_time` | 10 s | maximum lifetime of the gyro reference (**value still open; to be fixed once replay settles it**) |

With the defaults, the fixed budget is **g = 0.15 rad = 8.59° per frame**; the reference expires after **0.6 s** without a refresh; the gyro tolerance at 10 s is **9.74°**.

### Where it is inserted, why an invalid state means "withhold", and what it logs

- Inserted in `getRobotOriginWorldPose()` after tf has resolved `world→base_link` and before `tf2::quatRotate()` (upstream line 601), i.e. **ahead of the rotation**. This is the only place the heading enters the per-frame lever-arm rotation, so the gate covers the IMU source and the `use_odometry_yaw` source alike. It is the code counterpart of the claim that the guard sits in the heading domain, ahead of the rotation, independently of |L|.
- Once invalid, `prepareGpsOdometry()` returns `false` (lines 220–223 then simply do not publish). This is an **existing** "no usable data this cycle" path in the file. Inflating the covariance was rejected instead, because nowhere in the file is covariance used to express health (lines 675/829/844 are all copy-through plus a static rotation), and the published attitude is an identity quaternion anyway (lines 599–602, 846), so inflating the yaw covariance would be a no-op downstream.
- **Nothing is silent**: hold, reference expiry, transition to invalid, gyro re-anchor and two-frame re-anchor each emit one `ROS_WARN_THROTTLE(1.0, ...)`. By contrast, the upstream Mahalanobis gate writes only `FB_DEBUG` on rejection, which requires `debug` to be enabled before anything is printed.

---

## 3. Upper bound on the cost of holding (the expression required by manuscript §3.2)

Holding for h frames at frame interval dt, with the true heading changing no faster than ω_max, the heading lag is bounded by `ω_max·h·dt`, and the position error by

    e_hold ≤ 2|L| · sin( ω_max · h · dt / 2 ),   h ≤ max_hold_frames

In v3, h is hard-limited to `max_hold_frames` (beyond that the state goes invalid and output is withheld), so the cost of holding is bounded by construction. For this vehicle (|L| = 0.3655 m, 2|L| = 0.7311 m, ω_max = 0.5 rad/s, h = 5):

| dt | e_hold bound | for comparison: fault step bound 2\|L\| |
|---|---|---|
| 0.05 s (20 Hz) | 4.6 cm | 73.1 cm |
| 0.10 s (10 Hz, navsat default frequency) | 9.1 cm | 73.1 cm |
| 0.20 s (5 Hz, `/rtk_odom` on this vehicle) | **18.1 cm** | 73.1 cm |

> All three rows are **[derived]** from the expression, not measured. Beyond h frames the cost is no longer an error but a withheld output: it turns from a position error into an interruption of GPS updates, which the downstream EKF handles by de-weighting on timeout.

---

## 4. 🔴 An honest limitation: rule (b) can anchor onto a heading that is wrong but steady

This has to go into the manuscript; it must not be buried.

**Mechanism**: rule (b) only asks "do these two frames agree with each other?", never "are these two frames right?". If the fault is not a single-frame spike but a heading that **jumps to a wrong value and then holds steady** (exactly what a wrong dual-antenna integer-ambiguity fix looks like), the two frames agree perfectly, (b) re-anchors happily, and the wrong heading is promoted to the new reference. **Measured event in replay**: the gate rejected for only **0.6 s**, then re-anchored onto the wrong heading, and the published position carried a **0.395 m** offset from then on (that offset corresponds to a heading error of about 65°, back-computed from `2|L|sin(Δ/2)`).

**How much the gyro reference recovers**: rule (a) propagates the pre-fault heading with the gyro and confronts the heading under test with it — it asks "is this right?" rather than "is this steady?", so it does block the wrong-but-steady re-anchor above. **But only for seconds**: the tolerance opens up at `gyro_drift_rate·Δt`, and at `max_gyro_ref_time` (default 10 s) the reference is dropped entirely and behaviour falls back to (b). In other words, **the gyro extends the rejection window from 0.6 s to a few seconds, not indefinitely**. Recognising a wrong-but-steady heading indefinitely needs a different class of information (baseline-length residual, double-difference residual, or a second independent heading source), which is outside the scope of this patch; it belongs in the manuscript's Discussion, not its Results.

**Framing for the manuscript**: the guard addresses the case where a heading outlier is converted into the position domain while every quality field stays green. It **does not claim** to detect every heading fault. A persistent wrong heading is a known residual, and stating it plainly with the 0.395 m event as the worked example is more defensible than glossing over it.

---

## 5. Which manuscript sections this patch backs

| Manuscript section | What this patch covers |
|---|---|
| §3.2, heading-domain gate ω_max·τ | the test `\|Δψ\| > max_yaw_rate·nominal_dt + yaw_gate_margin` **is the implementation of the expression in §3.2**; 🔴 §3.2 needs one added sentence: the budget must be a fixed per-frame quantity and must not scale with time, or the persistence of the fault itself prises the gate open — this is what the two failures v1→v2→v3 bought |
| §3.3, two-domain comparison, "the guard sits ahead of the rotation" | the insertion point (after the tf lookup, before `quatRotate()`) is where that sentence lands in code, independently of \|L\| |
| §4.4, guard implementation | five parts: fixed-budget gate / hold / reference expiry / invalid-state withholding / gyro or two-frame re-anchor |
| §5.4, threshold sweep | must be re-run with v3; the v1 and v2 curves are void |
| §5.5, `navsat_transform` comparison | use the English paragraph in §6 plus the upstream line numbers listed in §7 |
| §6, Discussion | the limitation in §4 (wrong-but-steady heading; the gyro only buys seconds) goes here in one sentence, **not in Results** |
| Contribution 3 | the same mechanism holds in a third-party open-source implementation, and the guard fits into it in roughly 110 lines of logic |

---

## 6. English paragraph for §5.5 (142 words)

> The same conversion appears in the ROS `robot_localization` package. In `navsat_transform.cpp` (branch `noetic-devel`, commit `040dd17`, accessed 2026-09-19), `getRobotOriginWorldPose()` takes the `world`→`base_link` rotation from tf and rotates the `base_link`→`gps` offset with it (lines 576–603); `getRobotOriginCartesianPose()` does the same at datum time (lines 549–556). No validity, continuity or rate check is applied to that heading, and the published covariance is copied element-wise from `NavSatFix.position_covariance` and rotated by a static datum rotation (lines 675, 829, 844), so it is independent of the heading and of the step the heading introduces. The patch inserts a gate ahead of the rotation: a yaw increment exceeding one nominal interval of reachable rotation is rejected and the last accepted heading held, a reference left unrefreshed is declared stale, and after a set number of rejections the output is withheld until the heading can be re-anchored against a gyro-propagated reference.

(word count: 142, ≤ 150)

---

## 7. Verified vs. inferred

**Verified**
- Upstream line numbers and behaviour: 549–556 / 576–603 / 601–603 / 605–615 / 629–632 / 670–677 / 828–857, read from the upstream source at the commit named above
- Upstream EKF side: seven Mahalanobis thresholds default to `DBL_MAX`, and a rejection discards the whole frame silently, read from the same upstream source
- `imu_sub_` is `shutdown()` once `transform_good_` is set (lines 211–215) — this is why a separate `gyro_sub_` is required
- `gps_updated_` is cleared only after a successful publish (line 860) — this is why same-stamp de-duplication is required
- `clampRotation()` = `angles::normalize_angle` (`filter_utilities.cpp:131–134`)
- The patch applies cleanly: `patch -p1 --dry-run` is OK for both files, and so is the real application

**Inferred / not verified**
- The patch has **not been compiled and has never been run**; syntax, `-Wreorder` cleanliness and the availability of `std::fabs`/`std::min` were all inferred from the existing style of the file, and `catkin_make` is mandatory before it goes on a vehicle.
- The 0.6 s / 0.395 m event in §4 was **reported to me from the results side**; I did not run that replay myself. Back-computing 65° from 0.395 m via `2|L|sin(Δ/2)` is **[derived]**.
- The three cost figures in §3 are computed from the expression, i.e. **[derived]**.
- `max_yaw_rate = 0.5`, `nominal_dt = 0.2`, `max_gap_frames = 3`, `gyro_drift_rate = 0.002` and `max_gyro_ref_time = 10` are all **conservative defaults, not measured on this vehicle**; ω_max follows the p99 turn rate used in the manuscript, and `max_gyro_ref_time` should follow the measured bias stability of the CH110 IMU. Both are to be fixed once replay settles them.
- "Gyro available" is implemented as `has_gyro_ && gyro_yaw_time_ >= last_accepted_yaw_time_` (an IMU message has been received, and propagation has reached at least the reference epoch). The specification only said "gyro available"; this reading is mine, and `gate.py` follows the same one.
- Re-checked on 2026-09-19: the patch (including the same-stamp de-duplication fix) is OK for both upstream files under `patch -p1 --dry-run` and when actually applied, and the member declaration order matches the constructor initialisation order (`-Wreorder` clean). It is still **not compiled and not run under ROS**.
- `fig/gate.py` has been aligned with v3 and audited item by item. **The only remaining mismatch is a parameter value**: the v3a figures quoted in the manuscript body (false-gate rate 5.33 %, 19 466 withheld frames) were produced with `max_gyro_ref_time = None` (uncapped), whereas the patch default is **10 s** (which gives a false-gate rate of 0.369 % and 1 642 withheld frames). **These are not the same set of numbers**, and one of the two must be chosen before the manuscript is finalised.
