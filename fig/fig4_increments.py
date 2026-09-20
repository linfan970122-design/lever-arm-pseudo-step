#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 4 -- distribution of the frame-to-frame heading increment.

Double-column figure for GPS Solutions (174 mm), two panels, both log-log and both split by
the IMU-only motion label v2 (straight / turn / slow / unclassified, motion_class.py),
which never uses the RTK heading:
  (a) histogram of |dpsi| per 5 Hz frame;
  (b) exceedance probability P(|dpsi| > x), the quantity the text quotes.

Both panels carry the measured physical bound (gyro p99 and max |omega_z| x 0.2 s) and the
two deployed position-consistency thresholds expressed as heading steps,
Delta_min = 2 arcsin(T / 2|L|).

Numbers: results_numbers.py / results_numbers.csv (groups increments, physical_bound); the
bounds are read back from that file rather than recomputed.

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig4_increments.py
"""
import csv
import glob
import os

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

import motion_class as MC

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
GREY = "#6e6e6e"
PURPLE = "#CC79A7"
INK = "#1a1a1a"
HALO = [pe.withStroke(linewidth=2.0, foreground="white")]

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(HERE, "..", "data", "csv")

LNORM = float(np.hypot(0.235, 0.280))
TWO_L = 2.0 * LNORM
T_LAT, T_NORM = 0.30, 0.50
DT_MAX = 0.5
EXCLUDE_BAGS = ["orchard3src_20260826_2026-08-26-10-02-59"]   # replay, see results_numbers.md

NUM = {}
with open(os.path.join(HERE, "results_numbers.csv")) as f:
    for r in csv.DictReader(f):
        NUM[(r["group"], r["quantity"])] = r["value"]
B99 = float(NUM[("physical_bound", "bound p99")])
BMAX = float(NUM[("physical_bound", "bound max")])


def delta_min(T):
    return float(np.degrees(2.0 * np.arcsin(min(T / TWO_L, 1.0))))


D30, D50 = delta_min(T_LAT), delta_min(T_NORM)

# ------------------------------------------------------------------- data
cols = ["bag", "t", "dt", "dpsi_deg", "dp_ant", "dpsi_imu_deg", "hd_ok"]
d = pd.concat([pd.read_csv(f, usecols=cols)
               for f in sorted(glob.glob(os.path.join(CSV_DIR, "*.csv")))], ignore_index=True)
d["bag"] = d["bag"].str.replace(".bag", "", regex=False)
d = d[~d.bag.isin(EXCLUDE_BAGS)]
# motion class v2: "slow" from the reconstructed antenna path, never from the published
# displacement the fault corrupts (motion_class.py, results_numbers.md §9.4)
_cg = d.groupby("bag", sort=False)["dpsi_imu_deg"].transform(lambda x: x.fillna(0.0).cumsum())
_has = d.groupby("bag", sort=False)["dpsi_imu_deg"].transform(lambda x: x.notna().any())
d["cum_imu_full"] = np.where(_has, _cg, np.nan)
d["hd_prev"] = d.groupby("bag", sort=False)["hd_ok"].shift(1)
# The class is assigned on the SAME frame set the statistics are computed over -- the usable
# 5 Hz increments -- and in the same order as results_numbers.py, so the legend counts and the
# per-class numbers in results_numbers.md §2 / §9.4 are the same numbers.  Classifying before
# this filter puts frames that carry no usable increment into the trailing-window sums and
# shifts straight/slow by a few tens of frames.
d = (d[(d.hd_ok == 1) & (d.hd_prev == 1) & d.dpsi_deg.notna() & (d.dt <= DT_MAX)]
     .sort_values(["bag", "t"], kind="stable").reset_index(drop=True))
_cls = np.empty(len(d), dtype=object)
for _b, _g in d.groupby("bag", sort=False):
    _cls[_g.index.to_numpy()] = MC.classify(
        _g["t"].to_numpy(), _g["dp_ant"].to_numpy(), _g["cum_imu_full"].to_numpy(),
        t_bag_start=float(_g["t"].iloc[0]))
d["cls_imu"] = _cls
d["adpsi"] = d.dpsi_deg.abs()
# motion class v2 has four labels; all four are drawn so the legend counts add up to the
# corpus.  "na" = less than 3 s of history, or a recording without an IMU.
SETS = [("all", VERM, 1.3), ("straight", BLUE, 1.0), ("turn", ORANGE, 1.0),
        ("slow", GREY, 1.0), ("na", PURPLE, 1.0)]
NAMES = {"all": "all", "straight": "straight", "turn": "turn", "slow": "slow",
         "na": "unclassified"}
V = {"all": d.adpsi.to_numpy()}
for lab in ("straight", "turn", "slow", "na"):
    V[lab] = d.loc[d.cls_imu == lab, "adpsi"].to_numpy()
print("frames %d, straight %d, turn %d, slow %d, na %d (sum %d)"
      % (V["all"].size, V["straight"].size, V["turn"].size, V["slow"].size, V["na"].size,
         V["straight"].size + V["turn"].size + V["slow"].size + V["na"].size))

# ==================================================================== figure
FIG_W = 174 / 25.4
fig, (axa, axb) = plt.subplots(1, 2, figsize=(FIG_W, 3.15))
fig.subplots_adjust(left=0.086, right=0.988, top=0.90, bottom=0.135, wspace=0.225)

XLIM = (0.01, 200.0)
XT = [0.01, 0.1, 1, 10, 100]
XTL = ["0.01", "0.1", "1", "10", "100"]


def marks(ax, ylo, rot_y):
    ax.axvspan(BMAX, XLIM[1], color="#f2c9b8", alpha=0.35, lw=0, zorder=1)
    for x, c, lab in ((B99, GREEN, "gyro p99 = %.1f$^{\\circ}$" % B99),
                      (BMAX, GREEN, "gyro max = %.1f$^{\\circ}$" % BMAX),
                      (D30, VERM, "$\\Delta_{\\min}(0.30$ m$)$ = %.1f$^{\\circ}$" % D30),
                      (D50, VERM, "$\\Delta_{\\min}(0.50$ m$)$ = %.1f$^{\\circ}$" % D50)):
        ax.axvline(x, color=c, lw=0.8, ls=(0, (4, 2)), zorder=4)
        ax.text(x * 0.88, rot_y, lab, color=c, fontsize=6.6, ha="left", va="bottom",
                rotation=90, zorder=9, path_effects=HALO)
    ax.set_xscale("log")
    ax.set_xlim(*XLIM)
    ax.set_xticks(XT)
    ax.set_xticklabels(XTL)
    ax.set_xlabel("frame-to-frame heading increment $|\\Delta\\psi|$ (deg)")
    ax.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(direction="out", length=2.6, which="major")
    ax.tick_params(length=1.5, which="minor")


# ---------------------------------------------------------------- panel (a)
EDGES = np.logspace(-2, np.log10(180.0), 70)
for lab, c, lw in SETS:
    if lab == "all":
        continue
    h, _ = np.histogram(V[lab], bins=EDGES)
    axa.step(EDGES[:-1], np.maximum(h / V[lab].size, 1e-12), where="post", color=c, lw=1.0,
             label="%s ($n$ = %s)" % (NAMES[lab], format(V[lab].size, ",")), zorder=5)
axa.set_yscale("log")
axa.set_ylim(2e-6, 0.5)
axa.set_ylabel("fraction of frames per bin")
marks(axa, 2e-6, 3.3e-6)
axa.text(190.0, 0.30, "physically impossible\nin one 0.2 s frame", fontsize=6.6, ha="right",
         va="top", color=VERM, linespacing=1.25, zorder=9, path_effects=HALO)
axa.legend(loc="lower left", frameon=False, handlelength=1.7, borderaxespad=0.35)
axa.text(0.0, 1.02, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

# ---------------------------------------------------------------- panel (b)
xs = np.logspace(-2, np.log10(180.0), 400)
for lab, c, lw in SETS:
    v = np.sort(V[lab])
    p = 1.0 - np.searchsorted(v, xs, side="right") / v.size
    axb.plot(xs, np.maximum(p, 1e-7), color=c, lw=lw,
             label="%s ($n$ = %s)" % (NAMES[lab], format(v.size, ",")), zorder=5)
axb.set_yscale("log")
axb.set_ylim(2e-6, 1.5)
axb.set_ylabel("exceedance probability $P(|\\Delta\\psi| > x)$")
marks(axb, 2e-6, 3.3e-6)
for lev in (5, 10, 20, 45, 90):
    pa = float((V["all"] > lev).mean())
    axb.plot([lev], [pa], marker="o", ms=2.8, mfc="white", mec=VERM, mew=0.8, zorder=8)
axb.annotate("$P(|\\Delta\\psi| > %.1f^{\\circ})$ = %.3f %%\n(%s frames of %s)"
             % (BMAX, 100.0 * float((V["all"] > BMAX).mean()),
                format(int((V["all"] > BMAX).sum()), ","), format(V["all"].size, ",")),
             xy=(BMAX, float((V["all"] > BMAX).mean())), xytext=(185.0, 0.22),
             fontsize=6.8, color=VERM, ha="right", va="top", zorder=9, linespacing=1.3,
             path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=VERM, mutation_scale=5,
                             shrinkA=2, shrinkB=3))
axb.legend(loc="lower left", frameon=False, handlelength=1.7, borderaxespad=0.35)
axb.text(0.0, 1.02, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig4_increments." + ext), **kw)

print("checks:")
print("  gyro bound p99 / max     : %.2f / %.2f deg per frame" % (B99, BMAX))
print("  Delta_min(0.30) / (0.50) : %.1f / %.1f deg" % (D30, D50))
for lab, _, _ in SETS:
    v = V[lab]
    print("  %-8s RMS %.3f deg, P(>5) %.4f %%, P(>%.2f) %.4f %%, max %.2f"
          % (lab, float(np.sqrt(np.mean(v * v))), 100.0 * (v > 5).mean(), BMAX,
             100.0 * (v > BMAX).mean(), v.max()))
print("wrote Fig4_increments.pdf / .png in %s" % HERE)
