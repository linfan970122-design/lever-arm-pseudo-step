#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 2 -- the recorded lever-arm pseudo-step, 2026-09-06 15:57:03.

Double-column figure for GPS Solutions (174 mm), four stacked panels on a common time
axis, +/- 20 s around the event (bag run_20260906_2026-09-06-15-26-15, second greenhouse,
vehicle stationary):
  (a) heading psi recovered from the published /rtk_odom attitude;
  (b) published reference point: E and N displacement from the pre-event mean, and the norm;
  (c) the antenna phase centre reconstructed as p_ant = p_pub + R(yaw) L, same y scale;
  (d) the position quality fields the receiver reports: solution status, satellites,
      differential age, cov[0].

The point of the figure: (a) moves by -67.5 deg in one 0.2 s frame, (b) steps by 0.401 m,
(c) does not move, (d) does not change.

🔴 This is deliberately NOT the view P1 used (P1 Fig. 4 showed displacement against time for
a reflector-induced event).  Here the subject is heading + reconstructed antenna + quality
fields.

Numbers: results_numbers.py / results_numbers.csv (group event1).

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig2_event.py
"""
import datetime
import os

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

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
SKY = "#56B4E9"
GREY = "#6e6e6e"
INK = "#1a1a1a"
HALO = [pe.withStroke(linewidth=2.0, foreground="white")]

HERE = os.path.dirname(os.path.abspath(__file__))
BAG = "run_20260906_2026-09-06-15-26-15"
CSV = os.path.join(HERE, "..", "data", "csv", BAG + ".csv")
EVENT_WALL = "15:57:03.094"     # frame carrying the step, from results_numbers.py
HALF_WIN = 20.0                 # s
LNORM = float(np.hypot(0.235, 0.280))

# ------------------------------------------------------------------- data
d = pd.read_csv(CSV, usecols=["t", "x", "y", "z", "hdg_deg", "dpsi_deg", "dp_pub", "dp_ant",
                              "x_ant", "y_ant", "pred_jump", "quality", "sats", "age_s",
                              "cov00", "cov35"])
wall = np.array([datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3] for t in d.t])
i_ev = int(np.flatnonzero(wall == EVENT_WALL)[0])
t_ev = float(d.t[i_ev])
i_ev2 = int(d.dpsi_deg.abs().idxmax())          # the second event, 5.5 min earlier
w = d[(d.t >= t_ev - HALF_WIN) & (d.t <= t_ev + HALF_WIN)].copy()
w["rt"] = w.t - t_ev

pre = w[w.rt < 0]
x0, y0 = pre.x.mean(), pre.y.mean()
xa0, ya0 = pre.x_ant.mean(), pre.y_ant.mean()
w["dE"] = w.x - x0
w["dN"] = w.y - y0
w["dR"] = np.hypot(w.dE, w.dN)
w["aE"] = w.x_ant - xa0
w["aN"] = w.y_ant - ya0
w["aR"] = np.hypot(w.aE, w.aN)

ev = d.loc[i_ev]
ev2 = d.loc[i_ev2]

# ==================================================================== figure
FIG_W = 174 / 25.4
fig, axs = plt.subplots(4, 1, figsize=(FIG_W, 5.55), sharex=True,
                        gridspec_kw=dict(height_ratios=[1.0, 1.05, 1.05, 1.0]))
fig.subplots_adjust(left=0.092, right=0.905, top=0.985, bottom=0.068, hspace=0.13)
axa, axb, axc, axd = axs

for ax in axs:
    ax.axvline(0.0, color=VERM, lw=0.9, ls=(0, (4, 2)), zorder=2)
    ax.grid(True, which="major", color="#e4e4e4", lw=0.5, zorder=0)
    ax.tick_params(direction="out", length=2.6)
    ax.set_axisbelow(True)


def tag(ax, s):
    ax.text(0.004, 0.955, s, transform=ax.transAxes, fontsize=8.6, va="top", ha="left",
            weight="bold", zorder=9, path_effects=HALO)


# --------------------------------------------------------------- (a) heading
axa.plot(w.rt, w.hdg_deg, color=BLUE, lw=1.0, marker="o", ms=1.6, mew=0, zorder=4)
axa.set_ylabel("heading $\\psi$ (deg)")
axa.set_ylim(176, 278)
axa.set_yticks([180, 200, 220, 240, 260])
axa.annotate("$\\Delta\\psi = %.1f^{\\circ}$ in one 0.2 s frame" % ev.dpsi_deg,
             xy=(0.0, float(ev.hdg_deg)), xytext=(2.0, 232.0),
             fontsize=7.2, color=VERM, ha="left", va="center", zorder=8,
             path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=VERM, mutation_scale=5,
                             shrinkA=1, shrinkB=3))
axa.text(-16.8, 267.0,
         "second event ($\\Delta\\psi = %.1f^{\\circ}$, $|\\Delta\\mathbf{p}_{\\mathrm{pub}}|$ "
         "= %.3f m) %.0f s earlier, outside this window"
         % (ev2.dpsi_deg, ev2.dp_pub, t_ev - float(ev2.t)),
         fontsize=6.8, color=GREY, ha="left", va="center", zorder=8, path_effects=HALO)
tag(axa, "(a)")

# ------------------------------------------------- (b) published point
axb.plot(w.rt, w.dE, color=ORANGE, lw=0.9, label="East")
axb.plot(w.rt, w.dN, color=BLUE, lw=0.9, label="North")
axb.plot(w.rt, w.dR, color=VERM, lw=1.3, label="norm")
axb.set_ylabel("published point\ndispl. (m)", linespacing=1.25)
YL = (-0.12, 0.47)
axb.set_ylim(*YL)
axb.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4])
axb.legend(loc="upper right", ncol=3, frameon=False, handlelength=1.6, columnspacing=1.1,
           borderaxespad=0.25)
axb.annotate("$|\\Delta\\mathbf{p}_{\\mathrm{pub}}| = %.3f$ m in one frame\n"
             "$2|\\mathbf{L}|\\sin(|\\Delta\\psi|/2) = %.3f$ m" % (ev.dp_pub, ev.pred_jump),
             xy=(0.0, float(w.loc[i_ev, "dR"])), xytext=(2.2, 0.345),
             fontsize=7.2, color=VERM, ha="left", va="center", zorder=8,
             linespacing=1.3, path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=VERM, mutation_scale=5,
                             shrinkA=1, shrinkB=3))
tag(axb, "(b)")

# --------------------------------------- (c) reconstructed antenna point
axc.plot(w.rt, w.aE, color=ORANGE, lw=0.9, label="East")
axc.plot(w.rt, w.aN, color=BLUE, lw=0.9, label="North")
axc.plot(w.rt, w.aR, color=GREEN, lw=1.3, label="norm")
axc.set_ylabel("antenna point ANT1\ndispl. (m)", linespacing=1.25)
axc.set_ylim(*YL)
axc.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4])
axc.legend(loc="upper right", ncol=3, frameon=False, handlelength=1.6, columnspacing=1.1,
           borderaxespad=0.25)
axc.annotate("$\\mathbf{p}_{\\mathrm{ant}} = \\mathbf{p}_{\\mathrm{pub}} + "
             "\\mathbf{R}(\\psi)\\mathbf{L}$ does not move:\n"
             "$|\\Delta\\mathbf{p}_{\\mathrm{ant}}| = %.4f$ m over the same frame,\n"
             "%.3f m peak over the whole window"
             % (ev.dp_ant, float(w.aR.max())),
             xy=(0.0, float(w.loc[i_ev, "aR"])), xytext=(2.6, 0.30),
             fontsize=7.2, color=GREEN, ha="left", va="center", zorder=8,
             linespacing=1.3, path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=GREEN, mutation_scale=5,
                             shrinkA=1, shrinkB=3))
tag(axc, "(c)")

# ------------------------------------------------------- (d) quality fields
# Four flat lines.  cov[0] is scaled by 1e4 so that all three count-like quantities share
# the left axis without overlapping; the differential age keeps its own right axis.
axd.plot(w.rt, w.sats, color=BLUE, lw=1.0)
axd.step(w.rt, w.quality, where="mid", color=PURPLE, lw=1.2)
axd.plot(w.rt, w.cov00 * 1e4, color=ORANGE, lw=1.0)
axd.set_ylabel("satellites / status /\ncov[0] ($10^{-4}$ m$^2$)", linespacing=1.25)
axd.set_ylim(0, 34)
axd.set_yticks([0, 10, 20, 30])
axd.set_xlabel("time relative to the event (s)")

axd2 = axd.twinx()
axd2.plot(w.rt, w.age_s, color=GREEN, lw=1.0)
axd2.set_ylim(0, 2.5)
axd2.set_yticks([0, 1, 2])
axd2.set_ylabel("differential age (s)")
axd2.tick_params(direction="out", length=2.6, labelsize=7.5, colors=GREEN)
axd2.yaxis.label.set_color(GREEN)
axd2.spines["right"].set_color(GREEN)

for y, c, s_ in ((float(w.sats.iloc[0]) + 1.2, BLUE, "satellites (%d\u2013%d)"
                  % (w.sats.min(), w.sats.max())),
                 (float(w.age_s.iloc[0]) / 2.5 * 34 + 1.2, GREEN,
                  "differential age = %.1f s" % w.age_s.iloc[0]),
                 (float(w.cov00.iloc[0]) * 1e4 + 1.2, ORANGE,
                  "cov[0] = %.4f m$^2$" % w.cov00.iloc[0]),
                 (float(w.quality.iloc[0]) + 1.2, PURPLE,
                  "solution status = %d (RTK fixed)" % w.quality.iloc[0])):
    axd.text(-19.2, y, s_, fontsize=6.8, color=c, ha="left", va="bottom", zorder=9,
             path_effects=HALO)
tag(axd, "(d)")

axd.set_xlim(-HALF_WIN, HALF_WIN)
axd.set_xticks(np.arange(-20, 21, 5))

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig2_event." + ext), **kw)

print("event frame   : %s  dpsi = %.4f deg  dp_pub = %.4f m  dp_ant = %.4f m"
      % (EVENT_WALL, ev.dpsi_deg, ev.dp_pub, ev.dp_ant))
print("prediction    : 2|L| sin(|dpsi|/2) = %.4f m (residual %.4f m)"
      % (ev.pred_jump, ev.dp_pub - ev.pred_jump))
print("window        : %d frames, quality %s, sats %d-%d, age %.1f s, cov00 %.4f, cov35 %.3e"
      % (len(w), sorted(w.quality.unique()), w.sats.min(), w.sats.max(), w.age_s.iloc[0],
         w.cov00.iloc[0], w.cov35.iloc[0]))
print("antenna point : peak displacement over the window %.4f m (published %.4f m)"
      % (w.aR.max(), w.dR.max()))
print("second event  : %.1f s earlier, dpsi = %.1f deg, dp_pub = %.4f m"
      % (t_ev - float(ev2.t), ev2.dpsi_deg, ev2.dp_pub))
print("wrote Fig2_event.pdf / .png in %s" % HERE)
