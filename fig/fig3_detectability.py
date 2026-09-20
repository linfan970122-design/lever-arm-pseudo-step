#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 3 -- detectability of a lever-arm pseudo-step.

Double-column figure for GPS Solutions (174 mm), two panels:
  (a) Delta_min(T, |L|) = 2 arcsin(T / 2|L|) over the threshold / lever-arm plane,
      with the guaranteed-miss region T >= 2|L| hatched;
  (b) the same relation for this vehicle (|L| = 0.3655 m) against the heading-domain
      gate g, whose position-domain equivalent is T_eq = 2|L| sin(g/2).

Numbers: theory_numbers.py / the derivation note (not distributed).

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig3_detectability.py
"""
import csv
import os

import numpy as np
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
    "xtick.minor.width": 0.5,
    "ytick.minor.width": 0.5,
})

# Okabe-Ito colour-blind-safe set
BLUE = "#0072B2"
ORANGE = "#E69F00"
VERM = "#D55E00"
GREEN = "#009E73"
PURPLE = "#CC79A7"
GREY = "#6e6e6e"
INK = "#1a1a1a"
HALO = [pe.withStroke(linewidth=2.0, foreground="white")]

# ---------------------------------------------------------------- constants
LNORM = float(np.hypot(0.235, 0.280))    # 0.36555 m, this vehicle
TWO_L = 2.0 * LNORM                      # 0.73110 m
T_DEPLOYED = (0.30, 0.50)                # deployed lateral / chord thresholds
# deg, recorded 2026-09-06 15:57 event, one 5 Hz frame -- from theory_numbers.csv
_TN = {}
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "theory_numbers.csv")) as _f:
    for _r in csv.DictReader(_f):
        _TN[(_r["group"], _r["quantity"])] = _r["value"]
EVENT_DELTA = float(_TN[("event", "Delta (one 5 Hz frame)")])
GATES = (5.0, 7.0, 17.0)                 # candidate heading-domain gates, deg; 17 deg is the deployed budget


def delta_min(T, lever):
    r = np.asarray(T, float) / (2.0 * np.asarray(lever, float))
    out = np.degrees(2.0 * np.arcsin(np.clip(r, 0.0, 1.0)))
    return np.where(r > 1.0, np.nan, out)


def t_eq(gate_deg, lever=LNORM):
    return 2.0 * lever * np.sin(np.radians(gate_deg) / 2.0)


T_EVENT = t_eq(EVENT_DELTA)              # 0.4183 m: threshold that would just catch it

# ==================================================================== figure
FIG_W = 174 / 25.4                       # double column, 174 mm
fig, (axa, axb) = plt.subplots(1, 2, figsize=(FIG_W, 3.05))
fig.subplots_adjust(left=0.088, right=0.975, bottom=0.145, top=0.915, wspace=0.40)

TMIN, TMAX = 0.01, 1.0

# --------------------------------------------------------------- panel (a)
Tg = np.logspace(np.log10(TMIN), np.log10(TMAX), 500)
Lg = np.linspace(0.1, 2.0, 500)
TT, LL = np.meshgrid(Tg, Lg)
Z = delta_min(TT, LL)

levels = [0, 2, 5, 10, 20, 45, 90, 180]
cf = axa.contourf(TT, LL, np.ma.masked_invalid(Z), levels=levels, cmap="cividis",
                  extend="neither")
cf.set_edgecolor("face")   # kill the white seams between filled bands
cs = axa.contour(TT, LL, np.ma.masked_invalid(Z), levels=[2, 5, 10, 20, 45, 90],
                 colors="white", linewidths=0.7)
# label each contour at a hand-picked point (T = 2|L| sin(Delta/2)), clear of the legend
clab = [(2.0, 1.30), (5.0, 1.20), (10.0, 1.10), (20.0, 0.95), (45.0, 0.80), (90.0, 0.60)]
axa.clabel(cs, fmt=lambda v: "%g$^{\\circ}$" % v, fontsize=6.6, inline=True,
           inline_spacing=3,
           manual=[(2.0 * L * np.sin(np.radians(d) / 2.0), L) for d, L in clab])

# guaranteed-miss region: T >= 2|L|  <=>  |L| <= T/2
axa.fill_between(Tg, 0.1, np.clip(Tg / 2.0, 0.1, None), facecolor="white",
                 edgecolor="#555555", hatch="///", linewidth=0.6, zorder=3)
axa.text(0.98, 0.105, "never detectable\n($T \\geq 2|\\mathbf{L}|$)",
         fontsize=6.6, color="#333333", ha="right", va="bottom", zorder=6,
         linespacing=1.30, path_effects=HALO)

# this vehicle
axa.axhline(LNORM, color=VERM, lw=1.1, ls=(0, (4, 2)), zorder=5)
axa.plot(list(T_DEPLOYED), [LNORM] * 2, "o", ms=4.6, mfc="white", mec=VERM, mew=1.2,
         zorder=7, clip_on=False)
axa.plot([T_EVENT], [LNORM], "*", ms=8.5, mfc=ORANGE, mec="#5a4200", mew=0.6,
         zorder=8, clip_on=False)

leg = axa.legend(handles=[
    Line2D([], [], color=VERM, lw=1.1, ls=(0, (4, 2)),
           label="this vehicle, $|\\mathbf{L}|$ = 0.366 m"),
    Line2D([], [], color=VERM, lw=0, marker="o", ms=4.6, mfc="white", mew=1.2,
           label="deployed $T$ = 0.30, 0.50 m"),
    Line2D([], [], color=ORANGE, lw=0, marker="*", ms=8.0, mec="#5a4200", mew=0.6,
           label="recorded event, $\\Delta$ = %.1f$^{\\circ}$" % EVENT_DELTA),
], loc="upper left", frameon=True, framealpha=0.92, edgecolor="#bbbbbb",
    borderpad=0.4, handlelength=1.8, labelspacing=0.35)
leg.get_frame().set_linewidth(0.6)

axa.set_xscale("log")
axa.set_xlim(TMIN, TMAX)
axa.set_ylim(0.1, 2.0)
axa.set_xlabel("Position-consistency threshold $T$ (m)")
axa.set_ylabel("Lever-arm length $|\\mathbf{L}|$ (m)")
axa.set_xticks([0.01, 0.03, 0.1, 0.3, 1.0])
axa.set_xticklabels(["0.01", "0.03", "0.1", "0.3", "1.0"])
axa.tick_params(direction="out", length=2.6)
axa.text(0.0, 1.02, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom",
         ha="left", weight="bold")

cb = fig.colorbar(cf, ax=axa, pad=0.025, fraction=0.055, ticks=levels)
cb.set_label("$\\Delta_{\\mathrm{min}}$ (deg)", fontsize=8.0)
cb.ax.tick_params(labelsize=7.0, length=2.0, width=0.6)
cb.outline.set_linewidth(0.6)

# --------------------------------------------------------------- panel (b)
Tb = np.logspace(np.log10(TMIN), np.log10(TWO_L), 600)
axb.plot(Tb, delta_min(Tb, LNORM), color=VERM, lw=1.6, zorder=6,
         label="$\\Delta_{\\mathrm{min}} = 2\\arcsin(T/2|\\mathbf{L}|)$")

# guaranteed-miss region
axb.axvspan(TWO_L, TMAX, facecolor="white", edgecolor="#555555", hatch="///",
            linewidth=0.6, zorder=2)
axb.text(0.855, 45.0, "never\ndetectable", fontsize=6.9, color="#333333",
         ha="center", va="center", zorder=6, linespacing=1.35,
         path_effects=HALO)

# heading-domain gates
for g in GATES:
    axb.axhline(g, color=BLUE, lw=0.9, ls=(0, (3, 2)), zorder=4)
    axb.plot([t_eq(g)], [g], "o", ms=4.2, mfc=BLUE, mec="white", mew=0.7, zorder=7)
axb.text(0.0115, 3.3,
         "heading gate $g$ = 5$^{\\circ}$ / 7$^{\\circ}$ / 17$^{\\circ}$ (used)\n"
         "$T_{\\mathrm{eq}} = 2|\\mathbf{L}|\\sin(g/2)$\n"
         "= 0.032 / 0.045 / 0.108 m",
         fontsize=7.0, color=BLUE, ha="left", va="top", linespacing=1.35,
         zorder=8, path_effects=HALO)

# deployed thresholds
for T in T_DEPLOYED:
    d = float(delta_min(T, LNORM))
    axb.plot([T], [d], "o", ms=4.6, mfc="white", mec=VERM, mew=1.2, zorder=8)
    axb.annotate("$T$ = %.2f m\n$\\Delta_{\\mathrm{min}}$ = %.0f$^{\\circ}$" % (T, d),
                 xy=(T, d), xytext=(-6, 9), textcoords="offset points",
                 fontsize=7.0, color=VERM, ha="right", va="bottom",
                 linespacing=1.35, zorder=9, path_effects=HALO)

# recorded event
axb.plot([T_EVENT], [EVENT_DELTA], "*", ms=9.0, mfc=ORANGE, mec="#5a4200", mew=0.6,
         zorder=9)
axb.annotate("recorded event, $\\Delta$ = %.1f$^{\\circ}$" % EVENT_DELTA,
             xy=(T_EVENT, EVENT_DELTA), xytext=(0.70, 19.0), textcoords="data",
             fontsize=7.0, color="#8a6400", ha="right", va="center", zorder=9,
             path_effects=HALO,
             arrowprops=dict(arrowstyle="-", lw=0.6, color="#8a6400", shrinkA=2,
                             shrinkB=5))

axb.set_xscale("log")
axb.set_yscale("log")
axb.set_xlim(TMIN, TMAX)
axb.set_ylim(1.0, 200.0)
axb.set_xlabel("Position-consistency threshold $T$ (m)")
axb.set_ylabel("Floor $\\Delta_{\\mathrm{min}}$ of the detectable heading step (deg)")
axb.set_xticks([0.01, 0.03, 0.1, 0.3, 1.0])
axb.set_xticklabels(["0.01", "0.03", "0.1", "0.3", "1.0"])
axb.set_yticks([1, 2, 5, 10, 20, 50, 100, 180])
axb.set_yticklabels(["1", "2", "5", "10", "20", "50", "100", "180"])
axb.tick_params(direction="out", length=2.6, which="major")
axb.tick_params(length=1.5, which="minor")
axb.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
axb.text(0.0, 1.02, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom",
         ha="left", weight="bold")

HERE = os.path.dirname(os.path.abspath(__file__))
for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig3_detectability." + ext), **kw)

print("checks:")
print("  2|L|                = %.4f m" % TWO_L)
print("  Delta_min(0.30)     = %.2f deg" % delta_min(0.30, LNORM))
print("  Delta_min(0.50)     = %.2f deg" % delta_min(0.50, LNORM))
print("  T for Delta = %.2f = %.4f m" % (EVENT_DELTA, T_EVENT))
print("  T_eq(5), T_eq(7)    = %.4f, %.4f m" % (t_eq(5.0), t_eq(7.0)))
print("wrote Fig3_detectability.pdf / .png in %s" % HERE)
