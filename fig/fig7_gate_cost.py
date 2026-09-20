#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 7 -- what the gate catches, and what a hold costs.

Double-column figure for GPS Solutions (174 mm), two panels:
  (a) pseudo-step episodes of at least 0.10 m still present in each rule's own published
      output, against the gate budget g, with the ungated stream as the baseline.  This is
      the metric that decides: refusing a frame does not remove the step, it moves it to the
      edge where the gate takes a heading back;
  (b) the price of holding: the bound 2|L| sin(omega h tau / 2) for h = 1..5 frames at the
      measured gyro p99 and max yaw rates, with the error actually incurred on the genuine
      frames v3 rule (a) held at the recommended gate (Delta_held from the IMU-integrated
      yaw).  v3 holds for at most max_hold_frames - 1 frames before it stops publishing,
      so the measured points stop there.

Input: gate_sweep.csv, gate_sweep_v1.csv, hold_errors.csv, results_numbers.csv -- all
written by results_numbers.py.  Nothing is recomputed here except the closed-form bound,
which comes from gate.hold_cost_m().

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig7_gate_cost.py
"""
import csv
import os

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.lines import Line2D

import gate as G

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
    "xtick.minor.width": 0.5,
    "ytick.minor.width": 0.5,
    "lines.linewidth": 0.9,
})

BLUE = "#0072B2"
ORANGE = "#E69F00"
VERM = "#D55E00"
GREEN = "#009E73"
PURPLE = "#CC79A7"
GREY = "#6e6e6e"
HALO = [pe.withStroke(linewidth=2.0, foreground="white")]

HERE = os.path.dirname(os.path.abspath(__file__))
sw3 = pd.read_csv(os.path.join(HERE, "gate_sweep.csv"))
sw3a = sw3[sw3.variant == "v3a"].reset_index(drop=True)
sw3b = sw3[sw3.variant == "v3b"].reset_index(drop=True)
sw2 = pd.read_csv(os.path.join(HERE, "gate_sweep_v2.csv"))
sw1 = pd.read_csv(os.path.join(HERE, "gate_sweep_v1.csv"))
he = pd.read_csv(os.path.join(HERE, "hold_errors.csv"))
mah = pd.read_csv(os.path.join(HERE, "gate_mahal.csv"))
mah_all = mah[mah.subset == "all"].set_index("k_sigma")
NUM = {}
with open(os.path.join(HERE, "results_numbers.csv")) as f:
    for r in csv.DictReader(f):
        NUM[(r["group"], r["quantity"])] = r["value"]
B99 = float(NUM[("physical_bound", "bound p99")])
BMAX = float(NUM[("physical_bound", "bound max")])
G_REC = float(NUM[("gate", "recommended g")])
G_DEF = float(NUM[("gate", "patch default g")])
W99 = float(NUM[("hold_cost", "omega p99")])
WMAX = float(NUM[("hold_cost", "omega max")])
TWO_L = 2.0 * float(np.hypot(0.235, 0.280))
N_PST = int(NUM[("gate", "pseudo-step frames in the sweep")])
rec2 = sw2.loc[(sw2.gate_deg - G_REC).abs().idxmin()]
rec1 = sw1.loc[(sw1.gate_deg - G_REC).abs().idxmin()]
rec3a = sw3a.loc[(sw3a.gate_deg - G_REC).abs().idxmin()]
rec3b = sw3b.loc[(sw3b.gate_deg - G_REC).abs().idxmin()]

# ==================================================================== figure
FIG_W = 174 / 25.4
fig, (axa, axb) = plt.subplots(1, 2, figsize=(FIG_W, 3.05))
fig.subplots_adjust(left=0.072, right=0.988, top=0.90, bottom=0.145, wspace=0.215)

# ---------------------------------------------------------------- panel (a)
XLIM = (2.0, 30.0)
BASE_EP = int(NUM[("gated_residual", "no gate: episodes >=0.10 m")])
YMAX = 1.10 * max(float(t["residual_episodes_0.10"].max()) for t in (sw3a, sw3b, sw2, sw1))
axa.axvspan(XLIM[0], BMAX, color="#f2c9b8", alpha=0.30, lw=0, zorder=1)
for x, c, lab, side in ((BMAX, GREEN, "gyro max %.1f$^{\\circ}$" % BMAX, -1),
                        (G_REC, VERM, "recommended %.0f$^{\\circ}$" % G_REC, 1)):
    axa.axvline(x, color=c, lw=0.8, ls=(0, (4, 2)), zorder=4)
    axa.text(x + 0.4 * side, 0.52 * YMAX, lab, color=c, fontsize=6.6,
             ha="left" if side > 0 else "right", va="center", rotation=90, zorder=9,
             path_effects=HALO)
axa.axhline(BASE_EP, color=GREY, lw=1.0, ls=(0, (1, 2)), zorder=5)
for k_, ls_ in ((3.0, (0, (5, 1, 1, 1))), (5.0, (0, (3, 1, 1, 1, 1, 1)))):
    axa.axhline(mah_all.loc[k_, "residual_episodes_010"], color=GREEN, lw=1.0, ls=ls_,
                zorder=6, label="Mahalanobis, $k$ = %.0f" % k_)
axa.text(2.3, BASE_EP + 0.012 * YMAX, "no gate: %d episodes" % BASE_EP, color=GREY, fontsize=6.8,
         ha="left", va="bottom", zorder=9, path_effects=HALO)
for tab, c, lw, ls, mk, lab in (
        (sw3a, PURPLE, 1.4, "-", "D", "v3 (a), gyro re-anchor"),
        (sw3b, VERM, 1.4, "-", "o", "v3 (b), two-sample re-anchor"),
        (sw2, BLUE, 0.9, (0, (4, 2)), None, "v2"),
        (sw1, GREY, 0.9, (0, (1, 2)), None, "v1 (as patched)")):
    axa.plot(tab.gate_deg, tab["residual_episodes_0.10"], color=c, lw=lw, ls=ls,
             marker=mk, ms=2.2, mew=0, zorder=6, label=lab)
axa.set_xlim(*XLIM)
axa.set_xticks([2, 5, 10, 15, 20, 25, 30])
axa.set_ylim(0, YMAX)
axa.set_xlabel("gate $g$ (deg per frame)")
axa.set_ylabel("residual pseudo-step episodes $\\geq$ 0.10 m")
axa.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
axa.set_axisbelow(True)
axa.tick_params(direction="out", length=2.6)
axa.legend(loc="upper right", frameon=False, handlelength=2.2, borderaxespad=0.4,
           labelspacing=0.28, fontsize=6.6)
axa.annotate("at $g$ = %.0f$^{\\circ}$: v3(a) %d, v3(b) %d,\nv2 %d, v1 %d, no gate %d"
             % (G_REC, int(rec3a["residual_episodes_0.10"]),
                int(rec3b["residual_episodes_0.10"]), int(rec2["residual_episodes_0.10"]),
                int(rec1["residual_episodes_0.10"]), BASE_EP),
             xy=(G_REC, rec3a["residual_episodes_0.10"]), xytext=(29.6, 0.20 * YMAX),
             fontsize=6.8, color=PURPLE, ha="right", va="center", zorder=9, linespacing=1.3,
             path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=PURPLE, mutation_scale=5,
                             shrinkA=2, shrinkB=3))
axa.text(0.0, 1.02, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

# ---------------------------------------------------------------- panel (b)
hh = np.arange(1, 6)
axb.plot(hh, [G.hold_cost_m(WMAX, h) for h in hh], color=ORANGE, lw=1.3, marker="o", ms=3.2,
         mew=0, zorder=6, label="bound at $\\omega_{\\max}$ = %.2f rad/s" % WMAX)
axb.plot(hh, [G.hold_cost_m(W99, h) for h in hh], color=BLUE, lw=1.3, marker="o", ms=3.2,
         mew=0, zorder=6, label="bound at $\\omega_{p99}$ = %.2f rad/s" % W99)
rng = np.random.default_rng(7)
for cls_, c, m in (("slow", BLUE, "v"), ("straight", GREY, "o"), ("turn", VERM, "^")):
    sub = he[he.cls_imu == cls_]
    axb.scatter(sub.hold_count + rng.uniform(-0.13, 0.13, len(sub)), sub.err_m, s=9,
                facecolor="none", edgecolor=c, lw=0.7, marker=m, zorder=7,
                label="measured, %s ($n$ = %d)" % (cls_, len(sub)))
gmax = he.groupby("hold_count").err_m.max()
axb.plot(gmax.index, gmax.values, color=PURPLE, lw=1.1, ls=(0, (1, 1.6)), marker="D", ms=3.0,
         mew=0, zorder=8, label="measured worst case per $h$")
axb.axhline(TWO_L, color=GREY, lw=0.8, ls=(0, (1, 2)), zorder=4)
axb.text(5.35, TWO_L, "fault step bound\n$2|\\mathbf{L}|$ = %.3f m" % TWO_L, color=GREY,
         fontsize=6.6, ha="right", va="top", linespacing=1.25, zorder=9, path_effects=HALO)
axb.set_xlim(0.6, 5.4)
axb.set_xticks(hh)
axb.set_ylim(0, 0.80)
axb.set_xlabel("frames held, $h$")
axb.set_ylabel("published position error while holding (m)")
axb.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
axb.set_axisbelow(True)
axb.tick_params(direction="out", length=2.6)
axb.legend(loc="upper left", frameon=False, handlelength=1.9, borderaxespad=0.4,
           labelspacing=0.3)
axb.text(5.33, 0.105,
         "v3 (a): %d genuine frames held at $g$ = %.0f$^{\\circ}$,\nmax %.3f m, RMS %.4f m"
         % (len(he), G_REC, he.err_m.max(), float(np.sqrt(np.mean(he.err_m ** 2)))),
         fontsize=6.8, ha="right", va="top", linespacing=1.3, color=PURPLE,
         zorder=9, path_effects=HALO)
axb.text(0.0, 1.02, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig7_gate_cost." + ext), **kw)

print("checks:")
print("  mahal residual episodes >=0.10 m: k=3 %d, k=5 %d (false-gate %.2f / %.2f %%)"
      % (mah_all.loc[3.0, "residual_episodes_010"], mah_all.loc[5.0, "residual_episodes_010"],
         mah_all.loc[3.0, "false_gate_pct"], mah_all.loc[5.0, "false_gate_pct"]))
print("  residual episodes >=0.10 m at g=%.0f: v3a %d, v3b %d, v2 %d, v1 %d, no gate %d"
      % (rec3a.gate_deg, rec3a["residual_episodes_0.10"], rec3b["residual_episodes_0.10"],
         rec2["residual_episodes_0.10"], rec1["residual_episodes_0.10"], BASE_EP))
print("  bound at omega_p99 h=1..5 : " + ", ".join("%.3f" % G.hold_cost_m(W99, h) for h in hh))
print("  bound at omega_max h=1..5 : " + ", ".join("%.3f" % G.hold_cost_m(WMAX, h) for h in hh))
print("  measured holds            : n=%d, max %.4f m, RMS %.4f m, hold_count %d..%d"
      % (len(he), he.err_m.max(), float(np.sqrt(np.mean(he.err_m ** 2))),
         he.hold_count.min(), he.hold_count.max()))
print("wrote Fig7_gate_cost.pdf / .png in %s" % HERE)
