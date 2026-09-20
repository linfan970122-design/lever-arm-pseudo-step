#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 -- motion classification of a /rtk_odom frame, v2.

🔴 Reviewer 3: the v1 rule called a frame "slow" when the PUBLISHED position had moved no
more than 0.5 m over the trailing 3 s window.  That is the quantity the fault corrupts: a
lever-arm pseudo-step moves the published point without the platform moving, so the class
used to split the statistics was itself contaminated by the effect being measured.

v2 rule, verbatim, for a frame at time t in a recording with an IMU:

    W = 3.0 s                                     (trailing window, as in v1)
    d_ant_win = sum of |dp_ant| over the frames in (t - W, t]
                                                  (reconstructed antenna point, the quantity
                                                   the lever-arm fault does NOT move)
    turn_win  = integral of omega_z over (t - W, t]      (IMU only, never the RTK heading)

    d_ant_win <= 0.50 m            -> "slow"
    else |turn_win| <  5.0 deg     -> "straight"
    else                           -> "turn"
    less than W of history, or no IMU in the recording -> "na"

Only the *slow* test changed; the straight/turn split is the v1 IMU-integrated one.
Imported by results_numbers.py, fig4_increments.py and fig6_gate_sweep.py so the rule has a
single definition.
"""
import numpy as np

W = 3.0            # s, trailing window
SLOW_DISP = 0.50   # m, path length of the reconstructed antenna point over the window
STRAIGHT_DPSI = 5.0  # deg, |integrated omega_z| over the window


def classify(t, dp_ant, cum_imu, t_bag_start=None):
    """Labels for one recording, time-ordered.

    t        frame times (s)
    dp_ant   per-frame |dp_ant| (m); the first frame of a recording may be NaN
    cum_imu  cumulative IMU-integrated heading (deg), NaN if the recording has no IMU
    """
    t = np.asarray(t, float)
    da = np.nan_to_num(np.asarray(dp_ant, float), nan=0.0)
    cg = np.asarray(cum_imu, float)
    n = t.size
    out = np.full(n, "na", dtype=object)
    if n == 0 or not np.isfinite(cg).any():
        return out
    t0 = t[0] if t_bag_start is None else t_bag_start
    cda = np.concatenate([[0.0], np.cumsum(da)])          # cda[i] = sum of da[:i]
    j = np.searchsorted(t, t - W, side="right")           # first frame inside the window
    d_win = cda[np.arange(n) + 1] - cda[j]
    k = np.maximum(j - 1, 0)                              # frame at or before t - W
    turn = cg - cg[k]
    ok = (t - t0 >= W) & np.isfinite(turn)
    out[ok & (d_win <= SLOW_DISP)] = "slow"
    moving = ok & (d_win > SLOW_DISP)
    out[moving & (np.abs(turn) < STRAIGHT_DPSI)] = "straight"
    out[moving & (np.abs(turn) >= STRAIGHT_DPSI)] = "turn"
    return out
