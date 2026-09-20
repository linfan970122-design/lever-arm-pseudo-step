#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 Fig. 1 -- lever-arm geometry and the pseudo-step produced by a heading outlier.

Single-column figure for GPS Solutions (84 mm).  Schematic, top view, two panels:
  (a) nominal: two antennas, lever arm, published reference point, heading psi;
  (b) heading outlier Delta = -67.5 deg (the recorded 2026-09-06 15:57 event, one 5 Hz
      frame; the 69.8 deg quoted in P1 is the change over a 1 s window):
      ANT1 does not move, the rotated lever arm swings, the published point steps
      by 2|L| sin(Delta/2) = 0.406 m.

Numbers: theory_numbers.py / 理论_可检出性_v0_0919.md.  Geometry is schematic but the
proportions are real: |L| = 0.3655 m, antenna baseline b = 0.72 m, body width 0.89 m.

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 fig1_geometry.py
"""
import csv
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Arc, FancyArrowPatch, Polygon

# ------------------------------------------------------------------- style
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
    "font.size": 8.0,
    "axes.unicode_minus": False,
    "mathtext.fontset": "dejavusans",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "svg.fonttype": "none",
    "lines.linewidth": 0.9,
})

# Okabe-Ito colour-blind-safe set
BLUE = "#0072B2"
ORANGE = "#E69F00"
VERM = "#D55E00"
GREEN = "#009E73"
GREY = "#7f7f7f"
INK = "#1a1a1a"
BODY_FC = "#d9d9d9"
BODY_EC = "#9a9a9a"

# ------------------------------------------------------------- geometry (m)
LX, LY = 0.235, -0.280          # lever arm base_link -> ANT1, body frame (x fwd, y left)
LNORM = float(np.hypot(LX, LY))  # 0.3655 m
BASE = 0.72                      # antenna baseline ANT1 -> ANT2, along body x
HALF_W, HALF_L = 0.445, 0.675    # body half width (0.89 m) and half length
YAW = 20.0                       # drawing yaw of the true vehicle, deg CCW from East
# recorded heading outlier, deg -- single source: theory_numbers.csv
_TN = {}
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "theory_numbers.csv")) as _f:
    for _r in csv.DictReader(_f):
        _TN[(_r["group"], _r["quantity"])] = float(_r["value"]) \
            if _r["value"].replace(".", "").replace("-", "").isdigit() else _r["value"]
DELTA = -float(_TN[("event", "Delta (one 5 Hz frame)")])
DYAW = -DELTA                    # yaw = 90 - heading  => yaw step is -Delta  (verified 0920, data/sign_check_0906.md)


def rot(a_deg):
    a = np.radians(a_deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def b2w(p, yaw=YAW, origin=(0.0, 0.0)):
    """body -> world (drawing) frame"""
    return np.asarray(origin) + rot(yaw) @ np.asarray(p, float)


L = np.array([LX, LY])
P_PUB = np.array([0.0, 0.0])                    # published reference point (base_link)
ANT1 = P_PUB + rot(YAW) @ L                     # antenna phase centre, the raw fix point
ANT2 = ANT1 - BASE * (rot(YAW) @ np.array([1.0, 0.0]))
P_BAD = ANT1 - rot(YAW + DYAW) @ L              # published point under the outlier
STEP = P_BAD - P_PUB


def vehicle(ax, yaw=YAW, origin=(0.0, 0.0), fc=BODY_FC, ec=BODY_EC, alpha=1.0,
            ls="-", lw=0.7, z=1):
    """Tracked-chassis outline: hull plus two track bands."""
    hull = [(-HALF_L, -HALF_W), (HALF_L - 0.11, -HALF_W), (HALF_L, -HALF_W + 0.10),
            (HALF_L, HALF_W - 0.10), (HALF_L - 0.11, HALF_W), (-HALF_L, HALF_W)]
    ax.add_patch(Polygon([b2w(p, yaw, origin) for p in hull], closed=True, fc=fc,
                         ec=ec, lw=lw, ls=ls, alpha=alpha, zorder=z))
    for sgn in (-1, 1):
        band = [(-HALF_L, sgn * HALF_W), (HALF_L - 0.06, sgn * HALF_W),
                (HALF_L - 0.06, sgn * (HALF_W - 0.145)), (-HALF_L, sgn * (HALF_W - 0.145))]
        ax.add_patch(Polygon([b2w(p, yaw, origin) for p in band], closed=True,
                             fc="none", ec=ec, lw=lw * 0.8, ls=ls, alpha=alpha,
                             zorder=z + 0.1))


def arrow(ax, a, b, color, lw=1.2, ls="-", z=5, shrink=0.0, head=2.6, alpha=1.0):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=head,
                                 color=color, lw=lw, ls=ls, zorder=z, alpha=alpha,
                                 shrinkA=shrink, shrinkB=shrink,
                                 joinstyle="miter", capstyle="butt"))


HALO = [pe.withStroke(linewidth=2.0, foreground="white")]


def txt(ax, xy, s, color=INK, ha="left", va="center", size=7.2, **kw):
    ax.text(xy[0], xy[1], s, color=color, ha=ha, va=va, fontsize=size,
            zorder=9, path_effects=HALO, **kw)


# ==================================================================== figure
FIG_W = 84 / 25.4          # single column, 84 mm
fig, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(FIG_W, 4.95))
fig.subplots_adjust(left=0.012, right=0.988, top=0.985, bottom=0.010, hspace=0.06)

for ax in (ax_a, ax_b):
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)

XLIM = (-0.98, 1.12)

# --------------------------------------------------------------- panel (a)
vehicle(ax_a)

# body axes from the published reference point
x_tip = P_PUB + 0.52 * (rot(YAW) @ np.array([1.0, 0.0]))
y_tip = P_PUB + 0.40 * (rot(YAW) @ np.array([0.0, 1.0]))
arrow(ax_a, P_PUB, x_tip, INK, lw=1.0, z=6)
arrow(ax_a, P_PUB, y_tip, INK, lw=1.0, z=6)
txt(ax_a, x_tip + np.array([0.045, 0.010]), "$x$ (forward)", size=7.0)
txt(ax_a, y_tip + np.array([-0.030, 0.055]), "$y$ (left)", ha="center", va="bottom",
    size=7.0)

# heading psi: from North to the body x axis
NORTH = 0.60
arrow(ax_a, P_PUB, P_PUB + np.array([0.0, NORTH]), GREY, lw=0.8, z=4)
txt(ax_a, P_PUB + np.array([0.0, NORTH + 0.045]), "N", ha="center", va="bottom",
    size=7.4, weight="bold")
ax_a.add_patch(Arc(P_PUB, 0.94, 0.94, angle=0, theta1=YAW, theta2=90.0,
                   color=BLUE, lw=1.0, zorder=6))
mid = np.radians((YAW + 90.0) / 2.0)
txt(ax_a, P_PUB + 0.565 * np.array([np.cos(mid), np.sin(mid)]), r"$\psi$",
    color=BLUE, ha="center", size=8.6)

# lever arm, drawn the way the conversion uses it: p_pub = p_ant - R(psi) L
arrow(ax_a, ANT1, P_PUB, VERM, lw=1.5, z=7, head=3.0)
lmid = 0.5 * (ANT1 + P_PUB)
u = (P_PUB - ANT1) / np.linalg.norm(P_PUB - ANT1)
nrm = np.array([-u[1], u[0]])           # left normal of ANT1 -> p_pub, points up-right
txt(ax_a, lmid + 0.045 * nrm, r"$-\mathbf{R}(\psi)\,\mathbf{L}$", color=VERM,
    ha="right", va="top", size=7.0)

# antenna baseline
ax_a.plot([ANT1[0], ANT2[0]], [ANT1[1], ANT2[1]], color=BLUE, lw=0.9, ls=(0, (3, 2)),
          zorder=5)
txt(ax_a, 0.5 * (ANT1 + ANT2) + np.array([-0.02, -0.075]), "$b$", color=BLUE,
    ha="center", va="top", size=7.4)

for p_, lab, off, ha in ((ANT1, "ANT1 (reported point)", (0.055, 0.045), "left"),
                         (ANT2, "ANT2", (-0.055, 0.045), "right")):
    ax_a.plot(*p_, marker="o", ms=4.2, mfc=BLUE, mec="white", mew=0.7, zorder=8)
    txt(ax_a, p_ + np.array(off), lab, color=BLUE, ha=ha, va="bottom", size=7.0)

ax_a.plot(*P_PUB, marker="s", ms=4.6, mfc=VERM, mec="white", mew=0.7, zorder=8)
txt(ax_a, P_PUB + np.array([-0.055, -0.045]), r"$\mathbf{p}_{\mathrm{pub}}$",
    color=VERM, ha="right", va="top", size=7.6)

ax_a.text(0.985, 0.025,
          "$|\\mathbf{L}|$ = 0.366 m\n$b$ = 0.72 m\nbody width 0.89 m",
          transform=ax_a.transAxes, fontsize=6.8, va="bottom", ha="right",
          color="#4a4a4a", linespacing=1.35, zorder=9)
ax_a.text(0.010, 0.985, "(a)", transform=ax_a.transAxes, fontsize=8.6,
          va="top", ha="left", weight="bold")

# --------------------------------------------------------------- panel (b)
vehicle(ax_b, fc="#ececec", ec="#b5b5b5")

arrow(ax_b, ANT1, P_PUB, GREY, lw=1.2, z=6, head=2.8)
txt(ax_b, 0.5 * (ANT1 + P_PUB) + np.array([0.075, 0.100]),
    r"$-\mathbf{R}(\psi)\,\mathbf{L}$", color=GREY, ha="left", va="bottom",
    size=7.0)

arrow(ax_b, ANT1, P_BAD, ORANGE, lw=1.5, z=7, head=3.0)
ub = (P_BAD - ANT1) / np.linalg.norm(P_BAD - ANT1)
txt(ax_b, 0.5 * (ANT1 + P_BAD) + 0.060 * np.array([-ub[1], ub[0]]),
    r"$-\mathbf{R}(\psi+\Delta)\,\mathbf{L}$", color=ORANGE, ha="left",
    va="top", size=7.0)

# the outlier itself: the angle between the two lever arms, at ANT1
a0 = np.degrees(np.arctan2(*(P_PUB - ANT1)[::-1])) % 360.0
a1 = np.degrees(np.arctan2(*(P_BAD - ANT1)[::-1])) % 360.0
t1, t2 = (a0, a1) if (a1 - a0) % 360.0 <= 180.0 else (a1, a0)   # short way round
ax_b.add_patch(Arc(ANT1, 0.46, 0.46, angle=0, theta1=t1, theta2=t2,
                   color=ORANGE, lw=1.0, zorder=6))

arrow(ax_b, P_PUB, P_BAD, VERM, lw=1.8, z=8, head=3.4)
txt(ax_b, 0.5 * (P_PUB + P_BAD) + np.array([-0.055, 0.0]),
    "$|\\Delta\\mathbf{p}_{\\mathrm{pub}}| = 2|\\mathbf{L}|\\sin(|\\Delta|/2)$"
    "\n$= %.3f$ m" % (2.0 * LNORM * np.sin(np.radians(abs(DELTA)) / 2.0)),
    color=VERM, ha="right", va="center", size=7.0, linespacing=1.30)

ax_b.plot(*ANT1, marker="o", ms=4.2, mfc=BLUE, mec="white", mew=0.7, zorder=9)
txt(ax_b, ANT1 + np.array([0.060, 0.105]), "ANT1 unchanged", color=BLUE,
    ha="left", va="bottom", size=7.0)
txt(ax_b, ANT1 + np.array([0.060, 0.045]),
    "$\\Delta = %.1f^{\\circ}$\n(recorded event)" % DELTA, color=ORANGE, ha="left",
    va="top", size=7.0, linespacing=1.30)

ax_b.plot(*P_PUB, marker="s", ms=4.6, mfc=GREY, mec="white", mew=0.7, zorder=9)
txt(ax_b, P_PUB + np.array([0.0, 0.055]), r"$\mathbf{p}_{\mathrm{pub}}$", color=GREY,
    ha="center", va="bottom", size=7.6)
ax_b.plot(*P_BAD, marker="s", ms=4.6, mfc=VERM, mec="white", mew=0.7, zorder=9)
txt(ax_b, P_BAD + np.array([0.055, -0.020]),
    r"$\mathbf{p}'_{\mathrm{pub}}$", color=VERM, ha="left", va="top", size=7.6)

ax_b.text(0.010, 0.985, "(b)", transform=ax_b.transAxes, fontsize=8.6,
          va="top", ha="left", weight="bold")

# ------------------------------------------------------------------ limits
ax_a.set_xlim(*XLIM)
ax_a.set_ylim(-0.74, 0.80)
ax_b.set_xlim(*XLIM)
ax_b.set_ylim(-0.80, 0.74)

HERE = os.path.dirname(os.path.abspath(__file__))
for ext, kw in (("pdf", {}), ("png", {"dpi": 300})):
    fig.savefig(os.path.join(HERE, "Fig1_geometry." + ext), **kw)
print("step check: |dp| = %.4f m (expected %.4f)"
      % (np.hypot(*STEP), 2.0 * LNORM * np.sin(np.radians(abs(DELTA)) / 2.0)))
print("arc span  : %.2f deg (expected %.2f)" % ((t2 - t1) % 360.0, abs(DELTA)))
print("wrote Fig1_geometry.pdf / .png in %s" % HERE)
