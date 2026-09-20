#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 (lever-arm pseudo-step) -- every number quoted in the theory section.

Source of the derivation: the detectability derivation note (not distributed).
Project rule: no number enters the manuscript unless it comes out of this file.

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 theory_numbers.py

Writes theory_numbers.csv next to this script and prints the same content.

--------------------------------------------------------------------------
Model
--------------------------------------------------------------------------
Body frame: x forward, y left.  R(a) = 2-D counter-clockwise rotation.
Driver conversion (rtk_odom.py L605-609):   p_pub = p_ant - R(psi) L ,
                                            u_pub = u_ant - Lz     (z is NOT rotated)
A heading outlier of size Delta (reported heading psi -> psi + Delta) therefore moves
the published point while the antenna phase centre does not move at all:

    dp_pub = -R(psi) [ R(delta) - I ] L ,     |dp_pub| = 2|L| sin(|Delta|/2)

with delta the yaw step that corresponds to the heading step Delta.
TO VERIFY (sign map): the driver builds yaw = 90 deg - heading, so a heading step Delta is
expected to give delta = -Delta.  This has NOT yet been confirmed against the recorded
2026-09-06 15:57 event (mechanism-and-data checklist, not distributed, item A.8 / step 4).  SIGN_MAP below switches it;
the magnitude |dp_pub| is invariant to the choice, only the along/cross split flips.

Body-frame components of the step, written analytically (phi = atan2(Ly, Lx)):

    step_body = -[R(delta) - I] L
              = -2|L| sin(delta/2) * ( -sin(phi + delta/2), cos(phi + delta/2) )

    with delta = -Delta:
      along = -2|L| sin(Delta/2) sin(phi - Delta/2)
      cross = +2|L| sin(Delta/2) cos(phi - Delta/2)

This disagrees with the derivation note (not distributed) §1 "direction", which prints
   along = 2|L| sin(D/2) sin(phi + D/2), cross = -2|L| sin(D/2) cos(phi + D/2).
   That version does NOT reproduce the recorded event (it gives along = +0.417 /
   cross = +0.037 m for Delta = -67.54 deg, against the measured -0.109 / -0.404).
   The (phi - Delta/2) version reproduces it exactly.  The note should be corrected.
   The special case "along = 0" is at Delta = 2*phi = -100.0 deg for this vehicle
   (the note writes Delta = -2*phi but then quotes -100 deg, i.e. the number is right
   and the expression is not).
"""
import csv
import os

import numpy as np

# ---------------------------------------------------------------- parameters
# Lever arm base_link -> ANT1, body frame, from the deployed launch file
# (start_nav.sh / rtk_odom launch L28-L30; cross-checked against rtk_odom.py L455-457):
LX = 0.235    # m, forward
LY = -0.280   # m, left-positive  => antenna sits 0.280 m to the right
LZ = -0.175   # m, up-positive; subtracted as a constant, never rotated
TAU = 0.2     # s, /rtk_odom frame interval (5 Hz)
SIGM_PSI = 0.5  # deg, heading 1-sigma written into cov[35] when hd_ok (rtk_odom.py L67)

SIGN_MAP = -1.0   # delta = SIGN_MAP * Delta.  -1 = "yaw = 90 - heading".  TO VERIFY against the 2026-09-06 event.

# Recorded natural event, 2026-09-06 15:57:02.5 -> 03.5 (mechanism-and-data checklist, not distributed, A.6)
# The recorded 2026-09-06 15:57 event.  -69.8 deg is the change over a 1 s window
# (heading 256.73 -> 186.94 between 15:57:02.5 and 15:57:03.5); the position step is carried
# by ONE 5 Hz frame, whose increment is -67.54 deg (253.21 -> 185.67, results_numbers.md
# section 6).  The per-frame value is the one that pairs with the measured 0.4014 m step,
# so it is the one used here.
EVENT_DELTA = -67.5415   # deg, heading 253.2119 -> 185.6704 in one 0.2 s frame
EVENT_MEASURED = 0.4014  # m, measured published-position step

# Deployed position-domain thresholds (P1 sentinel): chord 0.50 m, lateral 0.30 m
T_DEPLOYED = (0.50, 0.30)

LNORM = float(np.hypot(LX, LY))
TWO_L = 2.0 * LNORM
PHI = float(np.degrees(np.arctan2(LY, LX)))

rows = []  # (group, key, value, unit, note)


def add(group, key, value, unit, note=""):
    rows.append((group, key, value, unit, note))


def step_body(delta_heading_deg):
    """Body-frame published-position step for a heading outlier of Delta degrees.

    step = -[R(delta) - I] L  with delta = SIGN_MAP * Delta.  Returns (along, cross).
    """
    d = np.radians(SIGN_MAP * delta_heading_deg)
    c, s = np.cos(d), np.sin(d)
    rl = np.array([c * LX - s * LY, s * LX + c * LY])
    step = -(rl - np.array([LX, LY]))
    return float(step[0]), float(step[1])


def step_body_closed_form(delta_heading_deg):
    """Same thing from the closed form, as a cross-check of the algebra."""
    dd = np.radians(delta_heading_deg)
    ph = np.radians(PHI)
    along = -TWO_L * np.sin(dd / 2.0) * np.sin(ph - dd / 2.0)
    cross = TWO_L * np.sin(dd / 2.0) * np.cos(ph - dd / 2.0)
    return float(along), float(cross)


def step_norm(delta_deg):
    return TWO_L * abs(np.sin(np.radians(delta_deg) / 2.0))


def delta_min(T, lever=None):
    """Smallest detectable heading step for a position-consistency threshold T."""
    lever = LNORM if lever is None else lever
    r = T / (2.0 * lever)
    if r >= 1.0:
        return float("nan")   # guaranteed-miss region: T >= 2|L|
    return float(np.degrees(2.0 * np.arcsin(r)))


def t_equivalent(gate_deg, lever=None):
    """Position-domain threshold equivalent to a heading gate of g degrees."""
    lever = LNORM if lever is None else lever
    return float(2.0 * lever * np.sin(np.radians(gate_deg) / 2.0))


def hold_cost(omega, hold_s, lever=None):
    """Worst-case position error from holding the last valid heading for hold_s
    while the vehicle really turns at omega rad/s."""
    lever = LNORM if lever is None else lever
    return float(2.0 * lever * np.sin(omega * hold_s / 2.0))


# ---------------------------------------------------------------- 1 geometry
add("geometry", "L_x", LX, "m", "launch L28")
add("geometry", "L_y", LY, "m", "launch L29, left-positive")
add("geometry", "L_z", LZ, "m", "launch L30, not rotated")
add("geometry", "|L|", LNORM, "m", "horizontal lever-arm length")
add("geometry", "2|L|", TWO_L, "m", "maximum possible position step")
add("geometry", "phi", PHI, "deg", "atan2(Ly, Lx), body-frame bearing of L")
add("geometry", "tau", TAU, "s", "/rtk_odom frame interval, 5 Hz")
add("geometry", "sigma_psi", SIGM_PSI, "deg", "heading 1-sigma in cov[35]")
add("event", "Delta (one 5 Hz frame)", abs(EVENT_DELTA), "deg",
    "2026-09-06 15:57:03.094, heading 253.2119 -> 185.6704; the 1 s figure quoted in P1 is "
    "69.8 deg, the per-frame value is the one that pairs with the measured step")
add("event", "measured |dp|", EVENT_MEASURED, "m", "published position step over that frame")
add("event", "predicted |dp|", step_norm(EVENT_DELTA), "m", "2|L| sin(|Delta|/2)")

# ------------------------------------------------- 2 step magnitude vs Delta
for d in (1, 2, 5, 10, 20, 30, 45, abs(EVENT_DELTA), 90, 180):
    note = "recorded event (one 5 Hz frame)" if abs(d - abs(EVENT_DELTA)) < 1e-9 else ""
    add("step_magnitude", "|dp| at Delta=%g deg" % d, step_norm(d), "m", note)
add("step_magnitude", "event measured |dp|", EVENT_MEASURED, "m",
    "0906 15:57, vs %.4f m predicted" % step_norm(EVENT_DELTA))
add("step_magnitude", "event residual", step_norm(EVENT_DELTA) - EVENT_MEASURED, "m",
    "predicted minus measured")
add("step_magnitude", "event back-solved |L|",
    EVENT_MEASURED / (2.0 * abs(np.sin(np.radians(EVENT_DELTA) / 2.0))), "m",
    "|dp|_meas / (2 sin(|Delta|/2)), vs surveyed %.4f m" % LNORM)
add("step_magnitude", "|du| (any Delta)", 0.0, "m", "z is not rotated")

# ------------------------------------------------- 3 along / cross direction
for d in (abs(EVENT_DELTA), EVENT_DELTA, -100.0, 10.0, -10.0, 30.0):
    a, c = step_body(d)
    a2, c2 = step_body_closed_form(d)
    assert abs(a - a2) < 1e-12 and abs(c - c2) < 1e-12, "closed form mismatch"
    add("direction", "along at Delta=%g deg" % d, a, "m", "body x, forward")
    add("direction", "cross at Delta=%g deg" % d, c, "m", "body y, left")
    add("direction", "norm at Delta=%g deg" % d, float(np.hypot(a, c)), "m", "")
add("direction", "Delta with along = 0", 2.0 * PHI, "deg",
    "Delta = 2*phi; pure cross-track step, invisible to any along-track test")

# --------------------------------------------- 4 position-domain Delta_min
for T in (0.03, 0.05, 0.10, 0.20, 0.30, 0.50):
    note = "deployed" if T in T_DEPLOYED else ""
    add("delta_min", "Delta_min at T=%.2f m" % T, delta_min(T), "deg", note)
add("delta_min", "guaranteed-miss threshold", TWO_L, "m",
    "T >= 2|L| : no heading outlier is ever detectable in the position domain")
add("delta_min", "T at Delta_min = %.2f deg" % abs(EVENT_DELTA),
    t_equivalent(abs(EVENT_DELTA)), "m",
    "threshold that would just catch the recorded event")

# ---------------------------------------------- 5 heading gate equivalence
for g in (3, 5, 7, 10):
    add("gate_equivalence", "T_eq at g=%g deg" % g, t_equivalent(g), "m",
        "2|L| sin(g/2)")
for g in (3, 5, 7, 10):
    add("gate_equivalence", "omega_max at g=%g deg" % g,
        float(np.radians(g) / TAU), "rad/s",
        "turn rate a real vehicle would need to produce this frame increment")

# --------------------------------------------------------- 6 guard hold cost
for omega in (0.3, 0.5):
    for hold in (0.4, 0.6, 1.0):
        add("hold_cost", "cost at omega=%.1f rad/s, hold=%.1f s" % (omega, hold),
            hold_cost(omega, hold), "m",
            "%d frames at tau=%.1f s" % (round(hold / TAU), TAU))
for omega in (0.3, 0.5):
    add("hold_cost", "omega*tau at omega=%.1f rad/s" % omega,
        float(np.degrees(omega * TAU)), "deg",
        "physical per-frame heading increment ceiling [TO BE MEASURED]")

# ------------------------------------------------------------------- output
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "theory_numbers.csv")
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["group", "quantity", "value", "unit", "note"])
    for g, k, v, u, n in rows:
        w.writerow([g, k, ("%.6f" % v) if isinstance(v, float) else v, u, n])

width = max(len(k) for _, k, _, _, _ in rows)
last = None
for g, k, v, u, n in rows:
    if g != last:
        print("\n--- %s" % g)
        last = g
    val = ("%.4f" % v) if isinstance(v, float) else str(v)
    print("  %-*s  %10s %-6s %s" % (width, k, val, u, n))
print("\nwritten: %s  (%d rows)" % (OUT, len(rows)))
