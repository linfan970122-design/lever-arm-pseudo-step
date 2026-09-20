#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 6 -- what the heading-continuity gate costs, swept over the gate budget.

Double-column figure for GPS Solutions (174 mm), two panels against the per-frame budget
g = max_yaw_rate*tau + yaw_gate_margin (tau = 0.2 s):
  (a) false-gate rate -- increments the gyro confirms within 2 deg that the gate does not
      accept -- overall, for v3 rules (a) and (b), v2 and v1, plus the rate that survives
      attributing every withheld frame to the outage that produced it;
  (b) the same by motion state for v3 rule (b) and v2;
  (c) the share of frames each rule publishes nothing for.

Input: gate_sweep.csv (v3, both re-anchor rules), gate_sweep_v2.csv and gate_sweep_v1.csv,
all written by results_numbers.py, which runs every rule through gate.py.  Nothing is
recomputed here.

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig6_gate_sweep.py
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
from matplotlib.patches import Patch

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
mah = pd.read_csv(os.path.join(HERE, "gate_mahal.csv"))
mah_all = mah[mah.subset == "all"].set_index("k_sigma")
sw1 = pd.read_csv(os.path.join(HERE, "gate_sweep_v1.csv"))
NUM = {}
with open(os.path.join(HERE, "results_numbers.csv")) as f:
    for r in csv.DictReader(f):
        NUM[(r["group"], r["quantity"])] = r["value"]
B99 = float(NUM[("physical_bound", "bound p99")])
BMAX = float(NUM[("physical_bound", "bound max")])
G_REC = float(NUM[("gate", "recommended g")])
G_DEF = float(NUM[("gate", "patch default g")])
N_GEN = int(NUM[("gate", "genuine frames")])
rec3a = sw3a.loc[(sw3a.gate_deg - G_REC).abs().idxmin()]
rec3b = sw3b.loc[(sw3b.gate_deg - G_REC).abs().idxmin()]
rec2 = sw2.loc[(sw2.gate_deg - G_REC).abs().idxmin()]
rec1 = sw1.loc[(sw1.gate_deg - G_REC).abs().idxmin()]

XLIM = (2.0, 30.0)


def marks(ax, y_lab):
    ax.axvspan(XLIM[0], BMAX, color="#f2c9b8", alpha=0.30, lw=0, zorder=1)
    for x, c, lab, side in ((B99, GREEN, "gyro p99 %.1f$^{\\circ}$" % B99, 1),
                            (BMAX, GREEN, "gyro max %.1f$^{\\circ}$" % BMAX, -1),
                            (G_DEF, GREY, "patch default %.1f$^{\\circ}$" % G_DEF, 1),
                            (G_REC, VERM, "recommended %.0f$^{\\circ}$" % G_REC, 1)):
        ax.axvline(x, color=c, lw=0.8, ls=(0, (4, 2)), zorder=4)
        ax.text(x + 0.35 * side, y_lab, lab, color=c, fontsize=6.6,
                ha="left" if side > 0 else "right", va="center", rotation=90, zorder=9,
                path_effects=HALO)
    ax.set_xlim(*XLIM)
    ax.set_xticks([2, 5, 10, 15, 20, 25, 30])
    ax.set_xlabel("gate $g$ (deg per frame)")
    ax.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(direction="out", length=2.6, which="major")
    ax.tick_params(length=1.5, which="minor")


# ==================================================================== figure
FIG_W = 174 / 25.4
fig, (axa, axb, axc) = plt.subplots(1, 3, figsize=(FIG_W, 4.15))
fig.subplots_adjust(left=0.075, right=0.995, top=0.930, bottom=0.375, wspace=0.42)

# legends live in the strip below each panel so that no curve runs through them
LEG = dict(loc="upper left", bbox_to_anchor=(-0.02, -0.215), frameon=False,
           borderaxespad=0.0, labelspacing=0.30, fontsize=7.0)


def marks(ax, y_lab):
    ax.axvspan(XLIM[0], BMAX, color="#f2c9b8", alpha=0.30, lw=0, zorder=1)
    for x, c, lab, side in ((BMAX, GREEN, "gyro max %.1f$^{\\circ}$" % BMAX, -1),
                            (G_REC, VERM, "recommended %.0f$^{\\circ}$" % G_REC, 1)):
        ax.axvline(x, color=c, lw=0.8, ls=(0, (4, 2)), zorder=4)
        ax.text(x + 0.45 * side, y_lab, lab, color=c, fontsize=6.4,
                ha="left" if side > 0 else "right", va="center", rotation=90, zorder=9,
                path_effects=HALO)
    ax.set_xlim(*XLIM)
    ax.set_xticks([2, 10, 20, 30])
    ax.set_xlabel("gate $g$ (deg per frame)")
    ax.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(direction="out", length=2.6, which="major")
    ax.tick_params(length=1.5, which="minor")


def tag(ax, s_):
    ax.text(0.0, 1.02, s_, transform=ax.transAxes, fontsize=8.6, va="bottom", ha="left",
            weight="bold")


# ------------------------------------------------- (a) overall, rule by rule
for tab, col, c, lw, ls, lab in (
        (sw3a, "false_gate_pct", PURPLE, 1.4, "-", "v3 (a), gyro"),
        (sw3a, "false_gate_attributed_pct", PURPLE, 1.0, (0, (1, 1.5)),
         "v3 (a), attributed"),
        (sw3b, "false_gate_pct", VERM, 1.4, "-", "v3 (b), two-sample"),
        (sw2, "false_gate_pct", BLUE, 0.9, (0, (4, 2)), "v2"),
        (sw1, "false_gate_pct", GREY, 0.9, (0, (1, 2)), "v1")):
    axa.plot(tab.gate_deg, np.maximum(tab[col], 1e-4), color=c, lw=lw, ls=ls, zorder=6,
             label=lab)
for k_, ls_ in ((3.0, (0, (5, 1, 1, 1))), (5.0, (0, (3, 1, 1, 1, 1, 1)))):
    axa.axhline(mah_all.loc[k_, "false_gate_pct"], color=GREEN, lw=1.0, ls=ls_, zorder=5,
                label="Mahalanobis, $k$ = %.0f" % k_)
axa.set_yscale("log")
axa.set_ylim(3e-3, 300.0)
axa.set_ylabel("genuine frames not accepted (%)")
marks(axa, 30.0)
h, l = axa.get_legend_handles_labels()
h += [Patch(facecolor="#f2c9b8", alpha=0.30, lw=0)]
l += ["shaded: $g$ below the gyro maximum (%.1f$^{\\circ}$)" % BMAX]
axa.legend(h, l, handlelength=2.2, **LEG)
tag(axa, "(a)")

# ------------------------------------------------- (b) by motion state
for col, c, lab in (("false_gate_pct", VERM, "all frames"),
                    ("false_gate_pct_straight", BLUE, "straight"),
                    ("false_gate_pct_turn", ORANGE, "turn")):
    axb.plot(sw3b.gate_deg, np.maximum(sw3b[col], 1e-4), color=c, lw=1.3, marker="o", ms=1.8,
             mew=0, zorder=6, label=lab)
    axb.plot(sw2.gate_deg, np.maximum(sw2[col], 1e-4), color=c, lw=0.9, ls=(0, (4, 2)),
             zorder=5)
axb.set_yscale("log")
axb.set_ylim(3e-3, 300.0)
axb.set_ylabel("genuine frames not accepted (%)")
marks(axb, 30.0)
h, l = axb.get_legend_handles_labels()
h += [Line2D([], [], color=GREY, lw=1.3), Line2D([], [], color=GREY, lw=0.9, ls=(0, (4, 2)))]
l += ["v3 (b)", "v2"]
axb.legend(h, l, handlelength=2.0, **LEG)
tag(axb, "(b)")

# ------------------------------------------------- (c) frames with no output
N_INC = float(NUM[("corpus", "increments_used")])
for tab, col, c, lw, ls, lab in (
        (sw3a, "unpublished_pct", PURPLE, 1.4, "-", "v3 (a)"),
        (sw3b, "unpublished_pct", VERM, 1.4, "-", "v3 (b)"),
        (sw2, "unpublished_pct", BLUE, 0.9, (0, (4, 2)), "v2"),
        (None, None, GREY, 0.9, (0, (1, 2)), "v1 (heading invalid)")):
    if tab is None:
        axc.plot(sw1.gate_deg, np.maximum(100.0 * sw1.heading_invalid / N_INC, 1e-4),
                 color=c, lw=lw, ls=ls, zorder=5, label=lab)
    else:
        axc.plot(tab.gate_deg, np.maximum(tab[col], 1e-4), color=c, lw=lw, ls=ls, zorder=6,
                 label=lab)
axc.set_yscale("log")
axc.set_ylim(3e-3, 300.0)
axc.set_ylabel("frames with no output (%)")
marks(axc, 30.0)
axc.legend(handlelength=2.2, **LEG)
axc.text(2.3, 230.0, "Mahalanobis: 0 %", fontsize=6.4, color=GREEN,
         ha="left", va="top", zorder=9, path_effects=HALO)
axc.annotate("%.2f %% under (a):\n%s frames, %d outages\nlonger than 60 s"
             % (rec3a.unpublished_pct, format(int(rec3a.unpublished), ","),
                int(NUM[("gate_v3a", "outages longer than 60 s")])),
             xy=(rec3a.gate_deg, rec3a.unpublished_pct), xytext=(29.4, 0.40),
             fontsize=6.4, color=PURPLE, ha="right", va="center", zorder=9, linespacing=1.3,
             path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=PURPLE, mutation_scale=5,
                             shrinkA=2, shrinkB=3))
tag(axc, "(c)")

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig6_gate_sweep." + ext), **kw)

print("checks at g = %.0f deg:" % G_REC)
for nm, r_ in (("v3a", rec3a), ("v3b", rec3b), ("v2", rec2), ("v1", rec1)):
    att = r_["false_gate_attributed_pct"] if "false_gate_attributed_pct" in r_ else float("nan")
    nop = r_["unpublished"] if "unpublished" in r_ else r_["heading_invalid"]
    print("  %-4s false-gate %8.4f %% (attributed %8.4f %%)  no output %6d  "
          "straight %7.4f %%  turn %7.4f %%"
          % (nm, r_["false_gate_pct"], att, nop, r_["false_gate_pct_straight"],
             r_["false_gate_pct_turn"]))
print("wrote Fig6_gate_sweep.pdf / .png in %s" % HERE)
print("  mahal k=3 %.2f %%, k=5 %.2f %% (all frames), 0 withheld, 0 residual episodes"
      % (mah_all.loc[3.0, "false_gate_pct"], mah_all.loc[5.0, "false_gate_pct"]))
