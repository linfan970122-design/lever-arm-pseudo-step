#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 10 -- what the measurement-side gate costs when it re-anchors on a frozen heading.

Double-column figure for GPS Solutions (174 mm), three stacked panels over 1520-1560 s of
bag9 (2026-09-09 orchard), all offsets against T0_REF = 1788911806.950244 s:

  (a) the recorded RTK heading, the gyro-integrated heading carried forward from the last
      clean sample, and the course over ground (independent check, plotted only while the
      reference moves faster than 0.5 m/s).  The frozen span is shaded.
  (b) the measurement-side gate's per-frame decision and, for the BOTH arm, the
      node-side gate's state reconstructed from its own (throttled) warnings.  "reject"
      means different things on the two sides: on the measurement side the frame's
      orientation is withheld from the EKF (orientation_covariance[0] = -1); on the node
      side the heading is invalid and the node keeps publishing /odometry/gps with the
      lever arm rotated by the last accepted (held) yaw.
  (c) the consumer's deviation from the recorded reference for STOCK, GATED, MEAS and
      BOTH at q = 0.06, with the geometric bound 2|L| = 0.731 m.

Inputs: data/m8b_measurement_gate/episode_1527_1550.csv (written by
analysis/m8b_post/episode_1527_1550.py) and the run CSVs it points at.

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 analysis/m8b_post/fig10_hold_in_turn.py
"""
import os
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, T0_REF, dev_series, load, source, wrap_deg

# ------------------------------------------------------------------- style
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
    "font.size": 8.0,
    "axes.labelsize": 8.5,
    "axes.titlesize": 8.5,
    "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5,
    "legend.fontsize": 7.0,
    "axes.unicode_minus": False,
    "mathtext.fontset": "dejavusans",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "svg.fonttype": "none",
    "axes.linewidth": 0.7,
    "xtick.major.width": 0.7,
    "ytick.major.width": 0.7,
    "lines.linewidth": 0.9,
})

BLUE = "#0072B2"
ORANGE = "#E69F00"
VERM = "#D55E00"
GREEN = "#009E73"
PURPLE = "#CC79A7"
GREY = "#6e6e6e"
INK = "#1a1a1a"
HALO = [pe.withStroke(linewidth=2.0, foreground="white")]

HERE = os.path.dirname(os.path.abspath(__file__))
FIGD = os.path.join(ROOT, 'fig')
EPI = os.path.join(ROOT, 'data', 'm8b_measurement_gate', 'episode_1527_1550.csv')

T_A, T_B = 1520.0, 1560.0
FROZEN = (1527.11, 1529.32)      # heading frozen, hd_ok = 1 throughout
BRANCH = (1529.52, 1530.53)      # wrong heading branch, course over ground disagrees
LEVER = np.hypot(0.235, 0.280)
BOUND = 2.0 * LEVER

df = pd.read_csv(EPI)
t = df.t_ref.values

# ---- panel (a) series ---------------------------------------------------------
# everything on one continuous (unwrapped) branch so the curves can be compared directly
yaw_rec = np.degrees(np.unwrap(np.radians(df.yaw_in_deg.values)))
i0 = int(np.argmin(np.abs(t - 1526.911)))          # last clean sample before the fault
yaw_gyro = yaw_rec[i0] + (df.gz_cum_deg.values - df.gz_cum_deg.values[i0])
course = df.course_deg.values.copy()
course = course + 360.0 * np.round((yaw_rec - course) / 360.0)
course[df.speed_ms.values < 0.5] = np.nan

# ---- panel (c) series ---------------------------------------------------------
ARMS = [('bas_stock_q060', 'STOCK', GREY, '-'),
        ('bas_gate_q060', 'GATED (node side)', ORANGE, '-'),
        ('bas_meas_q060', 'MEAS (measurement side)', BLUE, '-'),
        ('bas_both_q060', 'BOTH (stacked)', VERM, '-')]

FIG_W = 174 / 25.4
fig, (axa, axb, axc) = plt.subplots(3, 1, figsize=(FIG_W, 5.6), sharex=True,
                                    gridspec_kw=dict(height_ratios=[1.25, 0.60, 1.15]))
fig.subplots_adjust(left=0.088, right=0.988, top=0.972, bottom=0.075, hspace=0.16)

for ax in (axa, axb, axc):
    ax.axvspan(*FROZEN, color="#d9d9d9", zorder=0, lw=0)
    ax.axvspan(*BRANCH, color="#f0e2e2", zorder=0, lw=0)
    ax.set_xlim(T_A, T_B)

# ------------------------------------------------------------------ (a)
axa.plot(t, yaw_rec, color=INK, lw=1.1, label="recorded RTK heading (yaw, unwrapped)")
axa.plot(t, yaw_gyro, color=GREEN, lw=1.0, ls="--",
         label="gyro-integrated from the last clean sample (1526.91 s)")
axa.plot(t, course, color=PURPLE, lw=1.2, ls=":",
         label="course over ground while speed > 0.5 m/s (independent check)")
axa.set_ylabel("heading (deg, unwrapped)")
axa.legend(loc="upper left", frameon=False, handlelength=2.2, borderaxespad=0.4,
           labelspacing=0.3)
for x, lab in ((1527.71, "re-anchors on\nthe frozen value"),
               (1539.56, "MEAS re-anchors\n(+10.24 s)"),
               (1549.64, "node re-anchors\n(+10.00 s)")):
    for ax in (axa, axb, axc):
        ax.axvline(x, color=GREY, lw=0.6, ls=(0, (1, 2)), zorder=1)
    axa.annotate(lab, xy=(x, axa.get_ylim()[0]), xytext=(x + 0.5, axa.get_ylim()[0]),
                 fontsize=6.6, color=INK, ha="left", va="bottom", path_effects=HALO)
axa.text(0.5 * (FROZEN[0] + FROZEN[1]), 0.46, "heading frozen,\nhd_ok = 1",
         transform=axa.get_xaxis_transform(), ha="center", va="center", fontsize=6.6,
         color=INK, path_effects=HALO)
axa.annotate("+91.5$^\\circ$ back onto\nthe correct branch",
             xy=(BRANCH[1], yaw_rec[int(np.argmin(np.abs(t - BRANCH[1])))]),
             xytext=(BRANCH[1] + 2.0, 0.60), textcoords=axa.get_xaxis_transform(),
             ha="left", va="center", fontsize=6.6, color=VERM, path_effects=HALO,
             arrowprops=dict(arrowstyle="-", color=VERM, lw=0.6,
                             shrinkA=0.0, shrinkB=1.0))
axa.text(0.002, 0.03, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom",
         ha="left", weight="bold")

# ------------------------------------------------------------------ (b)
LEV = {"ACCEPT": 2, "HOLD": 1, "NOPUB": 0}
st = np.array([LEV[s] for s in df.meas_state.values])
axb.step(t, st, where="post", color=BLUE, lw=1.1, label="measurement-side gate (MEAS/BOTH)")
axb.plot(t[df.meas_reanchor.values == 1], st[df.meas_reanchor.values == 1], "o",
         mfc="none", mec=VERM, ms=5.0, mew=1.0, label="re-anchor")
node = df.node_state_both.values
nb = np.array([0 if ("stale" in s or "hold" in s) else 2 for s in node], float)
axb.step(t, nb - 0.12, where="post", color=VERM, lw=1.1, ls="--",
         label="node-side gate, BOTH arm (from its warnings)")
axb.set_yticks([0, 1, 2])
axb.set_yticklabels(["reject", "hold", "accept"])
axb.set_ylim(-0.55, 3.35)
axb.set_ylabel("gate state")
axb.legend(loc="upper left", frameon=False, handlelength=2.2, borderaxespad=0.3,
           labelspacing=0.2, ncol=3, columnspacing=1.2)
axb.text(0.002, 0.04, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom",
         ha="left", weight="bold")

# ------------------------------------------------------------------ (c)
for tag, name, col, ls in ARMS:
    tt, dd = dev_series(tag)
    tt = tt - T0_REF
    m = (tt >= T_A) & (tt <= T_B)
    axc.plot(tt[m], dd[m], color=col, ls=ls, lw=1.0, label=name)
axc.axhline(BOUND, color=INK, lw=0.7, ls=":")
axc.text(T_B - 0.4, BOUND + 0.005, "$2|L| = %.3f$ m" % BOUND, ha="right", va="bottom",
         fontsize=7.0, color=INK, path_effects=HALO)
axc.set_ylim(-0.03, 0.83)
axc.set_ylabel("consumer deviation (m)")
axc.set_xlabel("time in the recording (s)")
axc.legend(loc="upper left", frameon=False, handlelength=2.2, borderaxespad=0.4,
           labelspacing=0.25, ncol=1, bbox_to_anchor=(0.004, 0.94))
axc.text(0.002, 0.955, "(c)", transform=axc.transAxes, fontsize=8.6, va="bottom",
         ha="left", weight="bold")

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(FIGD, "Fig10_hold_in_turn." + ext), **kw)

print("checks:")
print("  frozen span %.2f - %.2f s, %d frames, hd_ok all = %d"
      % (FROZEN[0], FROZEN[1],
         int(((t >= FROZEN[0]) & (t <= FROZEN[1])).sum()),
         int(df.hd_ok[(t >= FROZEN[0]) & (t <= FROZEN[1])].min())))
mrec = (t > 1530.6) & (t < 1549.5) & (df.speed_ms.values > 0.5)
print("  |course - recorded heading| after the +91.5 deg step: median %.1f deg, max %.1f deg"
      % (np.median(np.abs(df.course_minus_hdgyaw.values[mrec])),
         np.max(np.abs(df.course_minus_hdgyaw.values[mrec]))))
for tag, name, _, _ in ARMS:
    tt, dd = dev_series(tag)
    tt = tt - T0_REF
    for a, b, lab in ((1530.73, 1539.56, 'MEAS hold'), (1539.60, 1549.70, 'node hold')):
        m = (tt >= a) & (tt <= b)
        print("  %-24s %s %6.2f-%6.2f s: n=%2d median %.4f max %.4f m"
              % (name, lab, a, b, m.sum(), np.median(dd[m]), dd[m].max()))
print("wrote Fig10_hold_in_turn.pdf / .png in %s" % FIGD)
