#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 5 -- the position steps a heading outlier produces.

Double-column figure for GPS Solutions (174 mm), two panels:
  (a) published position step against |dpsi| for every pseudo-step candidate frame (fixed
      solution, heading valid, reconstructed antenna point moved <= 0.02 m), with the
      predicted curve 2|L| sin(|dpsi|/2), the bound 2|L| and the two deployed
      position-consistency thresholds;
  (b) the residual dp_pub - 2|L| sin(|dpsi|/2) for the frames that stepped at least 0.05 m,
      against the +/- 0.02 m band.

Numbers: results_numbers.py -> results_numbers.csv (group pseudo_step) and
pseudo_steps.csv (the frame list).

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig5_pseudosteps.py
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
INK = "#1a1a1a"
HALO = [pe.withStroke(linewidth=2.0, foreground="white")]

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(HERE, "..", "data", "csv")

LNORM = float(np.hypot(0.235, 0.280))
TWO_L = 2.0 * LNORM
T_LAT, T_NORM = 0.30, 0.50
DP_ANT_MAX, COV_HD_BAD, DT_MAX = 0.02, 1e5, 0.5
RESID_OK = 0.02
EXCLUDE_BAGS = ["orchard3src_20260826_2026-08-26-10-02-59"]   # replay, see results_numbers.md
INDUCED_BAGS = ["run_20260906_2026-09-06-15-26-15"]           # induction session, §7b

NUM = {}
with open(os.path.join(HERE, "results_numbers.csv")) as f:
    for r in csv.DictReader(f):
        NUM[(r["group"], r["quantity"])] = r["value"]

# ------------------------------------------------------------------- data
cols = ["bag", "dt", "dpsi_deg", "dp_pub", "dp_ant", "resid", "cov35", "hd_ok", "quality"]
d = pd.concat([pd.read_csv(f, usecols=cols)
               for f in sorted(glob.glob(os.path.join(CSV_DIR, "*.csv")))], ignore_index=True)
d["bag"] = d["bag"].str.replace(".bag", "", regex=False)
d = d[~d.bag.isin(EXCLUDE_BAGS)]
d["hd_prev"] = d.groupby("bag", sort=False)["hd_ok"].shift(1)
d = d[(d.hd_ok == 1) & (d.hd_prev == 1) & d.dpsi_deg.notna() & (d.dt <= DT_MAX)]
d["adpsi"] = d.dpsi_deg.abs()
cand = d[(d.quality == 4) & (d.cov35 < COV_HD_BAD) & (d.dp_ant <= DP_ANT_MAX)]
cand_ind = cand[cand.bag.isin(INDUCED_BAGS)]
cand_ops = cand[~cand.bag.isin(INDUCED_BAGS)]
big = cand[cand.dp_pub >= 0.10]
big_ind = big[big.bag.isin(INDUCED_BAGS)]
big_ops = big[~big.bag.isin(INDUCED_BAGS)]

ps = pd.read_csv(os.path.join(HERE, "pseudo_steps.csv"))
ps["adpsi"] = ps.dpsi_deg.abs()
ps["induced"] = ps.bag.isin(INDUCED_BAGS)
ev1 = ps.loc[(ps.adpsi > 67.0) & (ps.adpsi < 68.0)].nlargest(1, "dp_pub").iloc[0]
resid_rms = float(np.sqrt(np.mean(ps.resid ** 2)))
print("candidates %d (%d induced), >=0.10 m %d (%d induced), >=0.05 m %d, "
      "residual RMS %.4f m, max |resid| %.4f m"
      % (len(cand), len(cand_ind), len(big), len(big_ind), len(ps), resid_rms,
         ps.resid.abs().max()))

# ==================================================================== figure
FIG_W = 174 / 25.4
fig, (axa, axb) = plt.subplots(1, 2, figsize=(FIG_W, 3.15))
fig.subplots_adjust(left=0.072, right=0.988, top=0.90, bottom=0.135, wspace=0.215)

XLIM = (0.01, 200.0)

# ---------------------------------------------------------------- panel (a)
axa.scatter(cand.adpsi, cand.dp_pub, s=1.2, c="#b9b9b9", alpha=0.25, lw=0, rasterized=True,
            zorder=3, label="candidate frames ($n$ = %s)" % format(len(cand), ","))
axa.scatter(big_ops.adpsi, big_ops.dp_pub, s=11, facecolors="none", edgecolors=VERM, lw=0.8,
            marker="o", zorder=6,
            label="$\\geq$ 0.10 m, operational ($n$ = %d)" % len(big_ops))
axa.scatter(big_ind.adpsi, big_ind.dp_pub, s=16, facecolors="none", edgecolors=PURPLE, lw=0.8,
            marker="s", zorder=7,
            label="$\\geq$ 0.10 m, induced session ($n$ = %d)" % len(big_ind))
xx = np.logspace(-2, np.log10(180.0), 400)
axa.plot(xx, TWO_L * np.sin(np.radians(xx) / 2.0), color=BLUE, lw=1.2, zorder=7,
         label="$2|\\mathbf{L}|\\sin(|\\Delta\\psi|/2)$")
for y, lab in ((T_LAT, "deployed cross-track threshold 0.30 m"),
               (T_NORM, "deployed norm threshold 0.50 m")):
    axa.axhline(y, color=VERM, lw=0.8, ls=(0, (4, 2)), zorder=4)
    axa.text(0.0115, y * 1.08, lab, color=VERM, fontsize=6.6, ha="left", va="bottom",
             zorder=9, path_effects=HALO)
axa.axhline(TWO_L, color=GREY, lw=0.8, ls=(0, (1, 2)), zorder=4)
axa.text(170.0, TWO_L * 1.06, "$2|\\mathbf{L}|$ = %.3f m" % TWO_L, color=GREY, fontsize=6.6,
         ha="right", va="bottom", zorder=9, path_effects=HALO)
axa.text(0.30, 0.030, "cloud above the curve = the vehicle\nactually moved during the frame",
         color="#8a8a8a", fontsize=6.6, ha="center", va="bottom", linespacing=1.25, zorder=9,
         path_effects=HALO)
axa.scatter([ev1.adpsi], [ev1.dp_pub], marker="*", s=70, facecolor=GREEN, edgecolor="white",
            lw=0.5, zorder=9)
axa.annotate("2026-09-06 15:57 event", xy=(ev1.adpsi, ev1.dp_pub), xytext=(190.0, 0.0075),
             fontsize=6.8, color=GREEN, ha="right", va="center", zorder=9, path_effects=HALO,
             arrowprops=dict(arrowstyle="-|>", lw=0.7, color=GREEN, mutation_scale=5,
                             shrinkA=1, shrinkB=4))
axa.set_xscale("log")
axa.set_yscale("log")
axa.set_xlim(*XLIM)
axa.set_ylim(1e-4, 1.6)
axa.set_xticks([0.01, 0.1, 1, 10, 100])
axa.set_xticklabels(["0.01", "0.1", "1", "10", "100"])
axa.set_xlabel("frame-to-frame heading increment $|\\Delta\\psi|$ (deg)")
axa.set_ylabel("published position step $|\\Delta\\mathbf{p}_{\\mathrm{pub}}|$ (m)")
axa.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
axa.set_axisbelow(True)
axa.tick_params(direction="out", length=2.6, which="major")
axa.tick_params(length=1.5, which="minor")
axa.legend(loc="lower right", frameon=False, handlelength=1.7, borderaxespad=0.35,
           labelspacing=0.35)
axa.text(0.0, 1.02, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

# ---------------------------------------------------------------- panel (b)
axb.axhspan(-RESID_OK, RESID_OK, color="#cfe7f5", alpha=0.55, lw=0, zorder=1)
axb.axhline(0.0, color=GREY, lw=0.7, zorder=3)
for s_ in (-1, 1):
    axb.axhline(s_ * resid_rms, color=BLUE, lw=0.8, ls=(0, (4, 2)), zorder=4)
_po, _pi = ps[~ps.induced], ps[ps.induced]
axb.scatter(_po.adpsi, _po.resid, s=13, facecolor=VERM, edgecolor="white", lw=0.4, zorder=6,
            marker="o", label="operational ($n$ = %d)" % len(_po))
axb.scatter(_pi.adpsi, _pi.resid, s=15, facecolor=PURPLE, edgecolor="white", lw=0.4, zorder=6,
            marker="s", label="induced session ($n$ = %d)" % len(_pi))
axb.scatter([ev1.adpsi], [ev1.resid], marker="*", s=70, facecolor=GREEN, edgecolor="white",
            lw=0.5, zorder=9, label="2026-09-06 15:57 event")
axb.set_xscale("log")
axb.set_xlim(3.0, 200.0)
axb.set_ylim(-0.033, 0.033)
axb.set_xticks([5, 10, 20, 50, 100, 180])
axb.set_xticklabels(["5", "10", "20", "50", "100", "180"])
axb.set_yticks([-0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03])
axb.set_xlabel("frame-to-frame heading increment $|\\Delta\\psi|$ (deg)")
axb.set_ylabel("residual $|\\Delta\\mathbf{p}_{\\mathrm{pub}}| - 2|\\mathbf{L}|"
               "\\sin(|\\Delta\\psi|/2)$ (m)")
axb.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
axb.set_axisbelow(True)
axb.tick_params(direction="out", length=2.6, which="major")
axb.tick_params(length=1.5, which="minor")
axb.text(3.4, RESID_OK * 0.93, "$\\pm$ %.2f m band: all %d frames inside\n"
         "(%d operational, %d induced)" % (RESID_OK, len(ps), len(_po), len(_pi)),
         fontsize=6.8, color="#2b6f9c", ha="left", va="top", linespacing=1.3, zorder=9,
         path_effects=HALO)
axb.text(190.0, resid_rms + 0.0015, "RMS = %.4f m" % resid_rms, fontsize=6.8, color=BLUE,
         ha="right", va="bottom", zorder=9, path_effects=HALO)
axb.legend(loc="lower right", frameon=False, handlelength=1.4, borderaxespad=0.35,
           labelspacing=0.35)
axb.text(0.0, 1.02, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom", ha="left",
         weight="bold")

for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig5_pseudosteps." + ext), **kw)

print("checks:")
print("  largest candidate step : %.4f m at %.1f deg"
      % (cand.dp_pub.max(), cand.loc[cand.dp_pub.idxmax(), "adpsi"]))
print("  event marked           : |dpsi| = %.2f deg, dp_pub = %.4f m, residual %.4f m"
      % (ev1.adpsi, ev1.dp_pub, ev1.resid))
print("  residual RMS / max     : %.4f / %.4f m, all %d frames within %.2f m"
      % (resid_rms, ps.resid.abs().max(), int((ps.resid.abs() <= RESID_OK).sum()), RESID_OK))
print("wrote Fig5_pseudosteps.pdf / .png in %s" % HERE)
