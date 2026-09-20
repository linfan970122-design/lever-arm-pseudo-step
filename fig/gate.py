#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 -- Python replay of the heading-continuity gate, v1 (as patched) and v2 (specified).

🔴 Consistency requirement (the rule adopted in the companion paper / patch/heading_gate_note.md §1): this module is the ONLY
implementation of the gate used anywhere in the P2 results.

Two rules live here:

  gate_series    -- v1, the rule as it stands in `patch/heading_gate.patch` today.
  gate_series_v2 -- v2, the first amendment: the budget stops widening after
                    max_hold_frames * nominal_dt, a hold that reaches max_hold_frames stops
                    publishing altogether, and the heading is trusted again once two
                    consecutive samples agree with each other ("re-anchor").
  gate_series_v3 -- v3, the specification that goes into the C++ patch.  The budget does
                    not scale with dt at all (that scaling was the remaining hole: after a
                    recording gap or a long hold it grew large enough to wave a 16-63 deg
                    outlier through), a stale reference is declared invalid after
                    max_gap_frames, and re-anchoring is done against the gyro-propagated
                    heading when an IMU is present (variant a) instead of against two
                    mutually consistent samples (variant b).

Parameter names are the ones the C++ patch will carry: max_yaw_rate, yaw_gate_margin,
nominal_dt, max_hold_frames, max_gap_frames, gyro_drift_rate, use_gyro.

v1 reproduces, statement for statement and parameter name for parameter name,
`NavSatTransform::gateHeading()` as added by `patch/heading_gate.patch`:

    if (last_accepted_yaw_time_.isZero()) { accept; return true; }
    dt     = max((stamp - last_accepted_yaw_time_).toSec(), 0.0)
    delta  = clampRotation(yaw - last_accepted_yaw_)          # wrap to (-pi, pi]
    budget = max_yaw_rate_ * dt + yaw_gate_margin_
    if (fabs(delta) <= budget) { accept; hold = 0; valid = true;  return true; }
    ++hold; yaw <- last_accepted_yaw_                          # hold the last accepted yaw
    if (hold > max_hold_frames_) valid = false
    return valid

Notes that matter for the replay:
  * `dt` is measured from the last ACCEPTED heading, not from the previous frame, so the
    budget widens during a hold and the gate heals itself (patch note §1).
  * a rejected frame does not update `last_accepted_yaw_`; the held yaw is what would be
    rotated into the lever arm.
  * `heading_valid` latches false only after strictly more than `max_hold_frames`
    consecutive rejections, and is cleared by the next acceptance.

Parameters carry the C++ names and units: max_yaw_rate [rad/s], yaw_gate_margin [rad],
max_hold_frames [frames].  Deployed defaults 0.5 / 0.05 / 5.

Run `python3 gate.py` for the self-test against the patch's worked numbers.
"""
import numpy as np

# patch defaults (navsat_transform.cpp constructor / nh_priv.param calls)
DEFAULT_MAX_YAW_RATE = 0.5      # rad/s
DEFAULT_YAW_GATE_MARGIN = 0.05  # rad  (2.865 deg)
DEFAULT_MAX_HOLD_FRAMES = 5     # frames
DEFAULT_MAX_GAP_FRAMES = 3      # frames, v3: a reference older than this is stale
# v3 MEMS bias allowance.  Measured on this platform: on straight, fixed-solution segments
# the |gyro-integrated minus HDT| disagreement over 60 s has a median of 0.0108 deg/s
# (1.89e-4 rad/s) and a p90 of 0.433 deg/s over 23 windows in 4 recordings
# (results_numbers.md section 7).  0.002 rad/s = 0.115 deg/s sits an order of magnitude
# above the median and well below the p90: 6.9 deg of extra allowance per minute withheld.
DEFAULT_GYRO_DRIFT_RATE = 0.002  # rad/s
TAU = 0.2                       # s, nominal /rtk_odom frame interval (5 Hz)


def clamp_rotation(a):
    """FilterUtilities::clampRotation == angles::normalize_angle, wrap to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def params_for_gate(gate_deg, tau=TAU, yaw_gate_margin=DEFAULT_YAW_GATE_MARGIN):
    """Map a per-frame budget g (deg, at the nominal interval tau) to the patch parameters.

    The gate a sweep point stands for is  g = max_yaw_rate * tau + yaw_gate_margin.  The
    margin is held at the deployed 0.05 rad and the platform rate absorbs the rest; for
    g <= 2 * 0.05 rad (5.73 deg) the margin is halved to g/2 so that max_yaw_rate stays
    strictly positive -- a gate with max_yaw_rate = 0 never widens its budget during a
    hold and is a different animal, not a tighter version of the same gate.
    Returns (max_yaw_rate [rad/s], yaw_gate_margin [rad]).
    """
    g = float(np.radians(gate_deg))
    marg = min(yaw_gate_margin, g / 2.0)
    return (g - marg) / tau, marg


def gate_series(yaw_rad, t_s, max_yaw_rate=DEFAULT_MAX_YAW_RATE,
                yaw_gate_margin=DEFAULT_YAW_GATE_MARGIN,
                max_hold_frames=DEFAULT_MAX_HOLD_FRAMES, dt_cap=None):
    """Run gateHeading() over one bag's heading sequence (time-ordered).

    yaw_rad : heading of each frame, radians (any fixed convention; only increments matter)
    t_s     : frame time stamps, seconds
    dt_cap  : None reproduces the patch exactly.  Any number caps the dt used in the budget,
              which is the one-line amendment discussed in the Results: as written, the
              budget grows without bound during a long hold, so a burst of heading garbage
              is eventually waved through.  Not part of the deployed patch.

    Returns dict of equal-length arrays:
      rejected      bool, the frame's heading was rejected and the held yaw substituted
      heading_valid bool, gateHeading()'s return value (false once hold > max_hold_frames)
      hold_count    int,  yaw_hold_count_ after this frame
      delta         float, clampRotation(yaw - last_accepted_yaw_) in rad (0 for the first)
      budget        float, max_yaw_rate*dt + yaw_gate_margin in rad (nan for the first)
      used_yaw      float, the yaw that would be rotated into the lever arm (rad)
    """
    yaw_rad = np.asarray(yaw_rad, float)
    t_s = np.asarray(t_s, float)
    n = yaw_rad.size
    rejected = np.zeros(n, bool)
    valid = np.ones(n, bool)
    hold = np.zeros(n, np.int32)
    delta = np.zeros(n, float)
    budget = np.full(n, np.nan)
    used = np.empty(n, float)

    last_yaw = np.nan
    last_t = np.nan
    hold_count = 0
    heading_valid = True
    for i in range(n):
        y = yaw_rad[i]
        if not np.isfinite(last_t):                      # last_accepted_yaw_time_.isZero()
            last_yaw, last_t = y, t_s[i]
            used[i] = y
            continue
        dt = max(t_s[i] - last_t, 0.0)
        if dt_cap is not None:
            dt = min(dt, dt_cap)
        d = float(clamp_rotation(y - last_yaw))
        b = max_yaw_rate * dt + yaw_gate_margin
        delta[i] = d
        budget[i] = b
        if abs(d) <= b:
            last_yaw, last_t = y, t_s[i]
            hold_count = 0
            heading_valid = True
            used[i] = y
        else:
            hold_count += 1
            used[i] = last_yaw                           # orientation.setRPY(..., held yaw)
            if hold_count > max_hold_frames:
                heading_valid = False
            rejected[i] = True
        hold[i] = hold_count
        valid[i] = heading_valid
    return dict(rejected=rejected, heading_valid=valid, hold_count=hold,
                delta=delta, budget=budget, used_yaw=used)


# --------------------------------------------------------------------------- v2
# States written into `state`:
ACCEPT = 0       # heading used as reported, anchor updated, output published
HOLD = 1         # heading rejected, last accepted yaw rotated into the lever arm, published
NOPUB = 2        # nothing published this frame (hold exhausted, or waiting to re-anchor)


def gate_series_v2(yaw_rad, t_s, max_yaw_rate=DEFAULT_MAX_YAW_RATE,
                   yaw_gate_margin=DEFAULT_YAW_GATE_MARGIN,
                   max_hold_frames=DEFAULT_MAX_HOLD_FRAMES, nominal_dt=TAU):
    """Heading-continuity gate v2 -- the specification that goes into the C++ patch.

    State: last_accepted_yaw, last_accepted_t, hold_count, valid, pending = (yaw, t).
    Parameters: max_yaw_rate [rad/s], yaw_gate_margin [rad], max_hold_frames [frames],
    nominal_dt [s].  For each sample (y, t):

      valid:
        budget = max_yaw_rate * min(t - last_accepted_t, max_hold_frames * nominal_dt)
                 + yaw_gate_margin
        |wrap(y - last_accepted_yaw)| <= budget
            -> ACCEPT: last_accepted = (y, t), hold_count = 0, publish with y
        else
            -> HOLD:   publish with last_accepted_yaw, hold_count += 1
               hold_count >= max_hold_frames -> valid = false, pending = None, and this
               frame is NOT published
      not valid:
        nothing is published.
        pending is None            -> pending = (y, t)
        |wrap(y - pending_yaw)| <= max_yaw_rate * (t - pending_t) + yaw_gate_margin
            -> RE-ANCHOR: ACCEPT y (valid = true, hold_count = 0, last_accepted = (y, t)),
               publish with y
        else                       -> pending = (y, t)

    Compared with v1 this changes three things: the budget stops widening once the hold has
    lasted max_hold_frames * nominal_dt (v1 let it grow without bound and eventually waved
    a burst through), an exhausted hold withholds the output instead of publishing a stale
    heading, and the heading is trusted again only when two consecutive samples are mutually
    consistent instead of on the first sample that happens to fall inside a widened budget.

    Returns dict of equal-length arrays:
      state       int,  ACCEPT / HOLD / NOPUB
      published   bool, whether an output would be published for this frame
      used_yaw    float, yaw rotated into the lever arm (nan when nothing is published)
      hold_count  int,  hold_count after this frame
      valid       bool, the validity flag after this frame
      anchor      int,  index of the frame whose yaw is the anchor used here (-1 if none);
                        for ACCEPT frames this is the frame itself
      delta       float, wrap(y - last_accepted_yaw) in rad while valid (nan otherwise)
      budget      float, the budget it was compared against (nan for the first frame)
    """
    yaw_rad = np.asarray(yaw_rad, float)
    t_s = np.asarray(t_s, float)
    n = yaw_rad.size
    state = np.zeros(n, np.int8)
    published = np.zeros(n, bool)
    used = np.full(n, np.nan)
    hold = np.zeros(n, np.int32)
    valid_a = np.ones(n, bool)
    anchor_a = np.full(n, -1, np.int64)
    delta_a = np.full(n, np.nan)
    budget_a = np.full(n, np.nan)

    last_yaw = np.nan
    last_t = np.nan
    anchor_i = -1
    hold_count = 0
    valid = True
    pending = None          # (yaw, t)
    dt_cap = max_hold_frames * nominal_dt

    for i in range(n):
        y = yaw_rad[i]
        if not np.isfinite(last_t):                   # first sample: adopt it
            last_yaw, last_t, anchor_i = y, t_s[i], i
            state[i] = ACCEPT
            published[i] = True
            used[i] = y
        elif valid:
            dt = min(max(t_s[i] - last_t, 0.0), dt_cap)
            d = float(clamp_rotation(y - last_yaw))
            b = max_yaw_rate * dt + yaw_gate_margin
            delta_a[i] = d
            budget_a[i] = b
            if abs(d) <= b:
                last_yaw, last_t, anchor_i = y, t_s[i], i
                hold_count = 0
                state[i] = ACCEPT
                published[i] = True
                used[i] = y
            else:
                hold_count += 1
                if hold_count >= max_hold_frames:
                    valid = False
                    pending = None
                    state[i] = NOPUB          # hold exhausted: nothing goes out
                else:
                    state[i] = HOLD
                    published[i] = True
                    used[i] = last_yaw        # stale heading rotates the lever arm
        else:
            if pending is None:
                pending = (y, t_s[i])
                state[i] = NOPUB
            else:
                dtp = max(t_s[i] - pending[1], 0.0)
                dp = float(clamp_rotation(y - pending[0]))
                delta_a[i] = dp
                budget_a[i] = max_yaw_rate * dtp + yaw_gate_margin
                if abs(dp) <= budget_a[i]:    # two consecutive samples agree -> re-anchor
                    last_yaw, last_t, anchor_i = y, t_s[i], i
                    hold_count = 0
                    valid = True
                    pending = None
                    state[i] = ACCEPT
                    published[i] = True
                    used[i] = y
                else:
                    pending = (y, t_s[i])
                    state[i] = NOPUB
        hold[i] = hold_count
        valid_a[i] = valid
        anchor_a[i] = anchor_i
    return dict(state=state, published=published, used_yaw=used, hold_count=hold,
                valid=valid_a, anchor=anchor_a, delta=delta_a, budget=budget_a)


# --------------------------------------------------------------------------- v3
def gate_series_v3(yaw_rad, t_s, cum_gyro_rad=None, max_yaw_rate=DEFAULT_MAX_YAW_RATE,
                   yaw_gate_margin=DEFAULT_YAW_GATE_MARGIN, nominal_dt=TAU,
                   max_hold_frames=DEFAULT_MAX_HOLD_FRAMES,
                   max_gap_frames=DEFAULT_MAX_GAP_FRAMES,
                   gyro_drift_rate=DEFAULT_GYRO_DRIFT_RATE, use_gyro=True,
                   max_gyro_ref_time=None):
    """Heading-continuity gate v3 -- the specification that goes into the C++ patch.

    State: last_accepted_yaw, last_accepted_t, hold_count, valid, pending(yaw, t),
    gyro_yaw (last_accepted_yaw propagated with the integrated omega_z since
    last_accepted_t; only when use_gyro and an IMU is present).

    Per frame (y, t):
      budget  g = max_yaw_rate * nominal_dt + yaw_gate_margin      <-- FIXED, no dt scaling
      dt = t - last_accepted_t > max_gap_frames * nominal_dt and valid
            -> valid = false, pending = None          (data gap: the reference is stale)
      valid:
            |wrap(y - last_accepted_yaw)| <= g -> ACCEPT (publish y, hold_count = 0)
            else -> HOLD (publish last_accepted_yaw, hold_count += 1)
                    hold_count >= max_hold_frames -> valid = false, pending = None,
                    frame NOT published
      not valid: not published; re-anchor rule
            (a) use_gyro, gyro available, and (t - last_accepted_t) <= max_gyro_ref_time:
                |wrap(y - gyro_yaw)| <= g + gyro_drift_rate * (t - last_accepted_t)
                    -> ACCEPT y (valid = true); else stay invalid (pending unused)
            (b) otherwise (no gyro, or use_gyro = false):
                pending is None -> pending = (y, t)
                |wrap(y - pending_yaw)| <= g -> ACCEPT y (valid = true)
                else -> pending = (y, t)

    The re-anchoring frame is itself published (it is an ACCEPT, as in the valid branch);
    `n_reanchor` in the result says how many frames that is, so a stricter reading that
    publishes only from the following frame can be derived without re-running.

    max_gyro_ref_time: None leaves rule (a) unbounded (the specification as first written).
    A number bounds how long the propagated reference may be trusted after the last accepted
    heading; past it rule (a) falls back to rule (b).  The replay shows the unbounded form
    latching for up to 1298 s, so the bounded form is the one meant for the patch.

    cum_gyro_rad: cumulative integrated omega_z per frame, in the SAME angle convention as
    yaw_rad (for this driver yaw = 90 deg - heading, so it is the negated cumulative sum of
    the heading increments).  Only differences cum_gyro[k] - cum_gyro[anchor] are used, so
    the constant offset is irrelevant.  Frames whose gyro value is not finite fall back to
    rule (b); `fallback_b` marks them.

    Returns dict of equal-length arrays: state, published, used_yaw, hold_count, valid,
    anchor, delta, budget, reanchor, fallback_b -- plus the scalars n_reanchor, n_gap.
    """
    yaw_rad = np.asarray(yaw_rad, float)
    t_s = np.asarray(t_s, float)
    n = yaw_rad.size
    gyro = None if cum_gyro_rad is None else np.asarray(cum_gyro_rad, float)
    state = np.zeros(n, np.int8)
    published = np.zeros(n, bool)
    used = np.full(n, np.nan)
    hold = np.zeros(n, np.int32)
    valid_a = np.ones(n, bool)
    anchor_a = np.full(n, -1, np.int64)
    delta_a = np.full(n, np.nan)
    budget_a = np.full(n, np.nan)
    reanchor = np.zeros(n, bool)
    fallback = np.zeros(n, bool)

    g = max_yaw_rate * nominal_dt + yaw_gate_margin
    gap_s = max_gap_frames * nominal_dt
    last_yaw = np.nan
    last_t = np.nan
    anchor_i = -1
    hold_count = 0
    valid = True
    pending = None
    n_gap = 0

    for i in range(n):
        y = yaw_rad[i]
        if not np.isfinite(last_t):                        # first sample: adopt it
            last_yaw, last_t, anchor_i = y, t_s[i], i
            state[i] = ACCEPT
            published[i] = True
            used[i] = y
            hold[i] = hold_count
            valid_a[i] = valid
            anchor_a[i] = anchor_i
            continue
        if valid and (t_s[i] - last_t) > gap_s:            # stale reference after a gap
            valid = False
            pending = None
            n_gap += 1
        if valid:
            d = float(clamp_rotation(y - last_yaw))
            delta_a[i] = d
            budget_a[i] = g
            if abs(d) <= g:
                last_yaw, last_t, anchor_i = y, t_s[i], i
                hold_count = 0
                state[i] = ACCEPT
                published[i] = True
                used[i] = y
            else:
                hold_count += 1
                if hold_count >= max_hold_frames:
                    valid = False
                    pending = None
                    state[i] = NOPUB
                else:
                    state[i] = HOLD
                    published[i] = True
                    used[i] = last_yaw
        else:
            fresh = (max_gyro_ref_time is None
                     or (t_s[i] - last_t) <= max_gyro_ref_time)
            have_gyro = (fresh and gyro is not None and anchor_i >= 0
                         and np.isfinite(gyro[i]) and np.isfinite(gyro[anchor_i]))
            if use_gyro and have_gyro:                     # (a) gyro-propagated reference
                gyro_yaw = last_yaw + (gyro[i] - gyro[anchor_i])
                allow = g + gyro_drift_rate * max(t_s[i] - last_t, 0.0)
                d = float(clamp_rotation(y - gyro_yaw))
                delta_a[i] = d
                budget_a[i] = allow
                if abs(d) <= allow:
                    last_yaw, last_t, anchor_i = y, t_s[i], i
                    hold_count = 0
                    valid = True
                    pending = None
                    state[i] = ACCEPT
                    published[i] = True
                    used[i] = y
                    reanchor[i] = True
                else:
                    state[i] = NOPUB
            else:                                          # (b) two consistent samples
                if use_gyro:
                    fallback[i] = True
                if pending is None:
                    pending = (y, t_s[i])
                    state[i] = NOPUB
                else:
                    dp = float(clamp_rotation(y - pending[0]))
                    delta_a[i] = dp
                    budget_a[i] = g
                    if abs(dp) <= g:
                        last_yaw, last_t, anchor_i = y, t_s[i], i
                        hold_count = 0
                        valid = True
                        pending = None
                        state[i] = ACCEPT
                        published[i] = True
                        used[i] = y
                        reanchor[i] = True
                    else:
                        pending = (y, t_s[i])
                        state[i] = NOPUB
        hold[i] = hold_count
        valid_a[i] = valid
        anchor_a[i] = anchor_i
    return dict(state=state, published=published, used_yaw=used, hold_count=hold,
                valid=valid_a, anchor=anchor_a, delta=delta_a, budget=budget_a,
                reanchor=reanchor, fallback_b=fallback,
                n_reanchor=int(reanchor.sum()), n_gap=n_gap)


# ------------------------------------------------------- Mahalanobis variant
DEFAULT_Q_YAW = DEFAULT_GYRO_DRIFT_RATE ** 2   # rad^2/s, process noise: (0.002 rad/s)^2 per s
DEFAULT_R_YAW = 7.61544e-05                    # rad^2, the cov[35] the driver publishes


def gate_series_mahal(yaw_rad, t_s, cum_gyro_rad=None, k_sigma=3.0, q_yaw=DEFAULT_Q_YAW,
                      r_yaw=DEFAULT_R_YAW, p0=None):
    """Reviewer 2's comparison: a normalised-innovation (Mahalanobis) test instead of a
    kinematic budget.

    🔴 This is a 1-D emulation of what `robot_localization` does to the yaw channel, not the
    full filter: one scalar state, no cross-covariance with position or velocity, no
    differential-drive process model.  It exists to answer "would a chi-square gate on the
    innovation have caught these?", nothing more.

        predict:  yaw <- yaw + (integrated omega_z over the interval)   (held if no IMU)
                  P   <- P + q_yaw * dt
        innov:    nu  = wrap(y_meas - yaw)
        test:     nu^2 / (P + r_yaw) > k_sigma^2  ->  REJECT (state and P not updated,
                  so P keeps growing and the test loosens by itself)
        accept:   K = P / (P + r_yaw); yaw += K*nu; P = (1 - K) * P

    The output is always published -- the filter substitutes its own estimate, it never
    withholds -- which is the practical difference from v2/v3.

    Returns state (ACCEPT / HOLD), published (all True), used_yaw (the filter estimate that
    would rotate the lever arm), nis, P.
    """
    yaw_rad = np.asarray(yaw_rad, float)
    t_s = np.asarray(t_s, float)
    gyro = None if cum_gyro_rad is None else np.asarray(cum_gyro_rad, float)
    n = yaw_rad.size
    state = np.zeros(n, np.int8)
    used = np.empty(n, float)
    nis = np.full(n, np.nan)
    pvar = np.full(n, np.nan)

    x = yaw_rad[0] if n else 0.0
    P = r_yaw if p0 is None else p0
    last_t = t_s[0] if n else 0.0
    for i in range(n):
        if i == 0:
            used[i] = x
            pvar[i] = P
            continue
        dt = max(t_s[i] - last_t, 0.0)
        last_t = t_s[i]
        if gyro is not None and np.isfinite(gyro[i]) and np.isfinite(gyro[i - 1]):
            x = x + (gyro[i] - gyro[i - 1])        # gyro-propagated prediction
        P = P + q_yaw * dt
        nu = float(clamp_rotation(yaw_rad[i] - x))
        s_ = P + r_yaw
        nis[i] = nu * nu / s_
        if nis[i] > k_sigma * k_sigma:
            state[i] = HOLD                        # rejected: the estimate is published
        else:
            K = P / s_
            x = float(clamp_rotation(x + K * nu))
            P = (1.0 - K) * P
            state[i] = ACCEPT
        used[i] = x
        pvar[i] = P
    return dict(state=state, published=np.ones(n, bool), used_yaw=used, nis=nis, P=pvar,
                hold_count=np.zeros(n, np.int32), valid=np.ones(n, bool),
                anchor=np.arange(n), reanchor=np.zeros(n, bool),
                fallback_b=np.zeros(n, bool), n_reanchor=0, n_gap=0)


def hold_cost_m(omega_rad_s, h_frames, tau=TAU, lever_norm=float(np.hypot(0.235, 0.280))):
    """Position error bound paid by holding the last accepted yaw for h frames:
    e_hold <= 2|L| sin(omega * h * tau / 2)   (patch note §3)."""
    return 2.0 * lever_norm * np.sin(omega_rad_s * h_frames * tau / 2.0)


if __name__ == "__main__":
    # self-test 1: constant 5 Hz, a single 60 deg outlier on an otherwise still platform.
    t = np.arange(12) * TAU
    yaw = np.zeros(12)
    yaw[5] = np.radians(60.0)
    r = gate_series(yaw, t)
    assert r["rejected"].sum() == 1 and r["rejected"][5]
    assert r["heading_valid"].all()          # one rejection is far from max_hold_frames
    assert np.isclose(r["used_yaw"][5], 0.0)
    # self-test 2: a step that never comes back -> held, then declared invalid
    yaw2 = np.where(np.arange(12) >= 5, np.radians(60.0), 0.0)
    r2 = gate_series(yaw2, t)
    # budget grows with dt since the last accepted frame: 0.5*dt+0.05 rad
    print("self-test 2 rejected frames:", np.flatnonzero(r2["rejected"]))
    print("self-test 2 first invalid frame:", np.flatnonzero(~r2["heading_valid"]))
    # self-test 3: the deployed default budget at 5 Hz
    print("default budget at dt=0.2 s: %.4f rad = %.3f deg"
          % (DEFAULT_MAX_YAW_RATE * TAU + DEFAULT_YAW_GATE_MARGIN,
             np.degrees(DEFAULT_MAX_YAW_RATE * TAU + DEFAULT_YAW_GATE_MARGIN)))
    for g in (2.0, 2.865, 5.0, 8.594, 30.0):
        rate, marg = params_for_gate(g)
        print("  g = %6.3f deg -> max_yaw_rate = %.4f rad/s, yaw_gate_margin = %.4f rad"
              % (g, rate, marg))
    print("hold cost at 0.5 rad/s, h=5, 5 Hz: %.4f m" % hold_cost_m(0.5, 5))

    # ---- v2 self-tests
    # (1) isolated outlier: held for one frame, still published, anchor unchanged
    r3 = gate_series_v2(yaw, t)
    assert r3["state"][5] == HOLD and r3["published"][5] and np.isclose(r3["used_yaw"][5], 0.0)
    assert (r3["state"][[0, 1, 2, 3, 4, 6]] == ACCEPT).all()
    # (2) a 60 deg step that stays: held 4x, then output withheld, then re-anchored once two
    #     consecutive samples agree
    r4 = gate_series_v2(yaw2, t)
    print("v2 self-test 2 states :", "".join("AHN"[s_] for s_ in r4["state"]))
    print("v2 self-test 2 publish:", "".join("1" if p else "0" for p in r4["published"]))
    assert r4["state"][5] == HOLD and r4["state"][9] == NOPUB   # 4 holds, then withheld
    assert r4["state"][10] == NOPUB                             # first sample only arms pending
    assert r4["state"][11] == ACCEPT and r4["published"][11]    # second agreeing sample re-anchors
    # (3) a burst of garbage never re-anchors while it keeps moving
    rng = np.random.default_rng(0)
    burst = np.radians(rng.uniform(-180, 180, 30))
    r5 = gate_series_v2(np.concatenate([np.zeros(5), burst]),
                        np.arange(35) * TAU)
    print("v2 self-test 3 published frames in the burst: %d of 30"
          % int(r5["published"][5:].sum()))

    # ---- v3 self-tests
    # (1) the budget does not scale with dt: the same outlier after a long gap is still
    #     rejected, where v1/v2 would have widened the budget past it
    t_gap = np.concatenate([np.arange(5) * TAU, [5 * TAU + 10.0], [5 * TAU + 10.2]])
    yaw_gap = np.concatenate([np.zeros(5), [np.radians(60.0)], [np.radians(60.0)]])
    r6 = gate_series_v3(yaw_gap, t_gap, use_gyro=False)
    print("v3 self-test 1 states :", "".join("AHN"[s_] for s_ in r6["state"]))
    assert r6["state"][5] == NOPUB and r6["n_gap"] == 1        # gap invalidates first
    assert r6["state"][6] == ACCEPT and r6["reanchor"][6]      # (b) re-anchors on frame 2
    # (2) rule (b) re-anchors onto a wrong but stable heading; rule (a) does not, because
    #     the gyro says the platform never turned
    t = np.arange(14) * TAU
    yaw_step = np.where(np.arange(14) >= 4, np.radians(60.0), 0.0)
    cum_gyro = np.zeros(14)                     # platform stationary: gyro sees nothing
    rb = gate_series_v3(yaw_step, t, use_gyro=False)
    ra = gate_series_v3(yaw_step, t, cum_gyro_rad=cum_gyro, use_gyro=True)
    print("v3 self-test 2 (b)    :", "".join("AHN"[s_] for s_ in rb["state"]))
    print("v3 self-test 2 (a)    :", "".join("AHN"[s_] for s_ in ra["state"]))
    assert ACCEPT in rb["state"][8:] and rb["n_reanchor"] >= 1   # (b) adopts the bad heading
    assert not ra["published"][8:].any() and ra["n_reanchor"] == 0
    # (3) with no gyro array, (a) falls back to (b)
    rf = gate_series_v3(yaw_step, t, cum_gyro_rad=None, use_gyro=True)
    assert rf["fallback_b"][8:].any() and (rf["state"] == rb["state"]).all()
    print("v3 fixed budget at defaults: %.4f rad = %.3f deg"
          % (DEFAULT_MAX_YAW_RATE * TAU + DEFAULT_YAW_GATE_MARGIN,
             np.degrees(DEFAULT_MAX_YAW_RATE * TAU + DEFAULT_YAW_GATE_MARGIN)))
    # (4) the bounded form hands over to rule (b) once the reference is stale
    rbd = gate_series_v3(yaw_step, t, cum_gyro_rad=cum_gyro, use_gyro=True,
                         max_gyro_ref_time=0.5)
    print("v3 self-test 4 (a) bounded:", "".join("AHN"[s_] for s_ in rbd["state"]))
    assert ACCEPT in rbd["state"][8:]          # falls back to (b) and re-anchors
    print("v3 gyro allowance growth: %.4f rad/s = %.3f deg/s (%.2f deg per minute)"
          % (DEFAULT_GYRO_DRIFT_RATE, np.degrees(DEFAULT_GYRO_DRIFT_RATE),
             np.degrees(DEFAULT_GYRO_DRIFT_RATE) * 60.0))
    # ---- Mahalanobis self-test: an isolated outlier is rejected, a real turn is not
    rm = gate_series_mahal(yaw, t, cum_gyro_rad=np.zeros(12), k_sigma=3.0)
    print("mahal self-test states:", "".join("AHN"[s_] for s_ in rm["state"]))
    assert rm["state"][5] == HOLD and rm["published"].all()
    turn = np.radians(np.arange(12) * 3.0)           # 15 deg/s, real, gyro agrees
    rt = gate_series_mahal(turn, t, cum_gyro_rad=turn, k_sigma=3.0)
    assert (rt["state"] == ACCEPT).all()
    print("mahal q = %.2e rad^2/s (drift %.3f rad/s squared), R = %.2e rad^2 (%.2f deg)"
          % (DEFAULT_Q_YAW, DEFAULT_GYRO_DRIFT_RATE, DEFAULT_R_YAW,
             np.degrees(np.sqrt(DEFAULT_R_YAW))))
    print("gate.py self-tests passed")
