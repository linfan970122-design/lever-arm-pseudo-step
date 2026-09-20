#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 8 -- where the pseudo-steps happen.

Double-column figure for GPS Solutions (174 mm), two panels, both using the 0.10 m
definition (fixed solution, heading valid, reconstructed antenna point still,
|dp_pub| >= 0.10 m):
  (a) rate per hour of fixed solution, by site, operational recordings only, with the
      2026-09-06 controlled multipath-induction session shown separately -- it is not
      natural operation and is excluded from every site bar;
  (b) the same, one point per recording, so that the aggregate is not read as uniform.

Input: pstep_by_bag.csv and results_numbers.csv, written by results_numbers.py.

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig8_site_rate.py
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
bb = pd.read_csv(os.path.join(HERE, "pstep_by_bag.csv"))
NUM = {}
with open(os.path.join(HERE, "results_numbers.csv")) as f:
    for r in csv.DictReader(f):
        NUM[(r["group"], r["quantity"])] = r["value"]

INDUCED_BAGS = ["run_20260906_2026-09-06-15-26-15"]      # induction session, results §7b
ind = bb[bb.bag.isin(INDUCED_BAGS)]
IND_N, IND_H = int(ind.n_pstep.sum()), float(ind.fixed_h.sum())
IND_RATE = IND_N / IND_H
bb = bb[~bb.bag.isin(INDUCED_BAGS)].reset_index(drop=True)
SITES = ["concrete yard", "greenhouse", "orchard"]
LABELS = {"concrete yard": "concrete", "greenhouse": "greenhouse\n(natural)",
          "orchard": "orchard"}
COL = {"concrete yard": BLUE, "greenhouse": VERM, "orchard": GREEN}
agg = bb.groupby("site").agg(n=("n_pstep", "sum"), h=("fixed_h", "sum"),
                             bags=("bag", "count")).reindex(SITES)
agg["rate"] = agg.n / agg.h
tot_n, tot_h = int(agg.n.sum()), float(agg.h.sum())
print(agg)
print("operational rate %.3f per fixed hour (%d frames / %.2f h); induced session %.2f "
      "(%d / %.2f h)" % (tot_n / tot_h, tot_n, tot_h, IND_RATE, IND_N, IND_H))

# ==================================================================== figure
FIG_W = 174 / 25.4
fig, (axa, axb) = plt.subplots(1, 2, figsize=(FIG_W, 3.05),
                               gridspec_kw=dict(width_ratios=[1.12, 1.25]))
fig.subplots_adjust(left=0.070, right=0.988, top=0.90, bottom=0.135, wspace=0.235)

# ---------------------------------------------------------------- panel (a)
xs = np.arange(len(SITES))
axa.bar(xs, agg.rate.values, width=0.58, color=[COL[s] for s in SITES], alpha=0.85,
        edgecolor="white", lw=0.6, zorder=5)
axa.bar([3.0], [IND_RATE], width=0.58, color=PURPLE, alpha=0.85, edgecolor="white", lw=0.6,
        hatch="///", zorder=5)
axa.text(3.0, IND_RATE + 1.25, "%.1f h$^{-1}$" % IND_RATE, fontsize=7.4, color=INK,
         ha="center", va="bottom", zorder=9, path_effects=HALO)
axa.text(3.0, IND_RATE + 0.25, "%d in %.1f h" % (IND_N, IND_H), fontsize=6.6,
         color="#4a4a4a", ha="center", va="bottom", zorder=9, path_effects=HALO)
axa.axhline(tot_n / tot_h, color=GREY, lw=0.9, ls=(0, (4, 2)), zorder=6)
axa.text(-0.55, 22.0, "dashed: all operational, %.2f h$^{-1}$" % (tot_n / tot_h),
         fontsize=6.8, color=GREY, ha="left", va="top", zorder=9, path_effects=HALO)
axa.text(-0.55, 18.6, "hatched: induced session,\nnot natural operation", fontsize=6.6,
         color=PURPLE, ha="left", va="top", linespacing=1.3, zorder=9, path_effects=HALO)
for x, s in zip(xs, SITES):
    r = agg.loc[s]
    base = max(r.rate + 0.25, 2.20)          # keep the labels clear of the corpus line
    axa.text(x, base + 1.25, "%.2f h$^{-1}$" % r.rate, fontsize=7.4, color=INK,
             ha="center", va="bottom", zorder=9, path_effects=HALO)
    axa.text(x, base, "%d in %.1f h" % (r.n, r.h), fontsize=6.6, color="#4a4a4a",
             ha="center", va="bottom", zorder=9, path_effects=HALO)
    if r.rate < 1.0:
        axa.plot([x, x], [r.rate + 0.08, base - 0.12], color="#b0b0b0", lw=0.6, zorder=6)

axa.set_xticks(list(xs) + [3.0])
axa.set_xticklabels([LABELS[s] for s in SITES] + ["induced\nsession"], fontsize=6.8)
axa.set_xlim(-0.6, 3.6)
axa.set_ylim(0, 22.5)
axa.set_ylabel("pseudo-step frames per hour of fixed solution")
axa.grid(True, axis="y", which="major", color="#dddddd", lw=0.5, zorder=0)
axa.set_axisbelow(True)
axa.tick_params(direction="out", length=2.6)
axa.text(0.0, 1.02, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

# ---------------------------------------------------------------- panel (b)
rng = np.random.default_rng(3)
for i, s in enumerate(SITES):
    sub = bb[bb.site == s]
    jitter = rng.uniform(-0.22, 0.22, len(sub))
    sz = 6.0 + 34.0 * (sub.fixed_h / bb.fixed_h.max())
    axb.scatter(i + jitter, sub.per_fixed_h.fillna(0.0), s=sz, facecolor="none",
                edgecolor=COL[s], lw=0.8, zorder=6)
    axb.plot([i - 0.32, i + 0.32], [agg.loc[s, "rate"]] * 2, color=COL[s], lw=1.6, zorder=7)
top = bb.loc[bb.per_fixed_h.idxmax()]


def human_bag(bag, site):
    """'run_20260828_2026-08-28-10-16-47' -> '2026-08-28 10:16, greenhouse' (label only)"""
    import re as _re
    m = _re.search(r"(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})$", str(bag))
    if not m:
        return "%s, %s" % (bag, site)
    y, mo, d_, hh, mi, _ss = m.groups()
    return "%s-%s-%s %s:%s, %s" % (y, mo, d_, hh, mi, site)


axb.annotate("%s\n%d frames in %.2f fixed h"
             % (human_bag(top.bag, top.site), top.n_pstep, top.fixed_h),
             xy=(SITES.index(top.site) + 0.0, top.per_fixed_h), xytext=(2.6, 15.5),
             fontsize=6.6, color=VERM, ha="right", va="center", zorder=9, linespacing=1.3,
             path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=VERM, mutation_scale=5,
                             shrinkA=2, shrinkB=4))
n_zero = int((bb.n_pstep == 0).sum())
axb.text(2.6, 11.3, "%d of the %d operational recordings carry none;\nthe rate is set by a handful of runs"
         % (n_zero, len(bb)), fontsize=6.6, color=GREY, ha="right", va="center",
         linespacing=1.3, zorder=9, path_effects=HALO)
axb.scatter([3.0], [IND_RATE], s=42, facecolor="none", edgecolor=PURPLE, lw=1.1, marker="s",
            zorder=8)
axb.plot([3 - 0.32, 3 + 0.32], [IND_RATE] * 2, color=PURPLE, lw=1.6, zorder=7)
axb.set_xticks(list(xs) + [3.0])
axb.set_xticklabels([LABELS[s] for s in SITES] + ["induced\nsession"], fontsize=6.8)
axb.set_xlim(-0.6, 3.6)
axb.set_ylim(-0.9, 23.5)
axb.set_ylabel("per recording (frames per fixed hour)")
axb.grid(True, axis="y", which="major", color="#dddddd", lw=0.5, zorder=0)
axb.set_axisbelow(True)
axb.tick_params(direction="out", length=2.6)
axb.legend(handles=[Line2D([], [], color=GREY, lw=0, marker="o", mfc="none", mec=GREY,
                           ms=3.0, label="one recording (marker area $\\propto$ fixed hours)"),
                    Line2D([], [], color=GREY, lw=1.6, label="site rate, panel (a)")],
           loc="upper left", frameon=False, handlelength=1.8, borderaxespad=0.4,
           labelspacing=0.3, bbox_to_anchor=(0.0, 1.0))
axb.text(0.0, 1.02, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig8_site_rate." + ext), **kw)

print("checks:")
for s in SITES:
    r = agg.loc[s]
    print("  %-14s %2d frames / %5.2f fixed h = %5.2f per hour (%d bags)"
          % (s, r.n, r.h, r.rate, r.bags))
print("  operational recordings with no pseudo-step frame: %d of %d" % (n_zero, len(bb)))
print("  induced session: %d frames / %.2f fixed h = %.2f per hour" % (IND_N, IND_H, IND_RATE))
print("  worst recording: %s, %.2f per fixed hour" % (top.bag, top.per_fixed_h))
print("wrote Fig8_site_rate.pdf / .png in %s" % HERE)
