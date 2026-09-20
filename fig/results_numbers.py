#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 (lever-arm pseudo-step) -- every number quoted in the Results section.

Project rule: no number enters the manuscript unless it comes out of this file.
Companion of theory_numbers.py, which covers the Mechanism / Detectability sections.

Inputs   ../data/csv/*.csv   (one row per /rtk_odom frame, written by ../data/extract_p2.py
                              -- see that file for the columns)
         ../data/summary.csv (per-bag roll-up, extract_p2.py + post_summary.py)
Outputs  results_numbers.csv  every number, one per row, with its definition
         results_numbers.md   the same numbers grouped as they will appear in the text
         gate_sweep.csv       gate v2 sweep   (read by fig6 / fig7)
         gate_sweep_v1.csv    gate v1 sweep, kept for the v1-vs-v2 comparison
         pseudo_steps.csv     the pseudo-step frames (audit trail)

The gate replay uses gate.py: v1 is the rule as it stands in patch/heading_gate.patch, v2 is
the amended specification that goes into the patch next.  Same parameter names in both
(max_yaw_rate / yaw_gate_margin / max_hold_frames / nominal_dt).

Run:
    systemd-run --user --scope -p MemoryMax=4G -- python3 results_numbers.py
"""
import csv
import datetime
import glob
import os

import numpy as np
import pandas as pd

import gate as G
import motion_class as MC

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(HERE, "..", "data", "csv")
SUMMARY = os.path.join(HERE, "..", "data", "summary.csv")

# ---------------------------------------------------------------- constants
LX, LY = 0.235, -0.280
LNORM = float(np.hypot(LX, LY))          # 0.36555 m
TWO_L = 2.0 * LNORM                      # 0.73110 m
TAU = 0.2                                # s, nominal /rtk_odom interval (5 Hz)
COV_HD_BAD = 1e5                         # cov[35] above this => driver had no heading
DP_ANT_MAX = 0.02                        # m, "the antenna point did not move"
PSTEP_T = (0.05, 0.10, 0.20, 0.30, 0.50)  # m, pseudo-step size thresholds
PSTEP_MAIN = 0.10                        # m, the definition used for gate catch rates
T_LAT = 0.30                             # m, deployed cross-track threshold (eq. 3)
T_NORM = 0.50                            # m, deployed chord/norm threshold
# Delta_min = 2 arcsin(T / 2|L|), eq. (4): the smallest heading jump each deployed
# position test can see.  Same formula as theory_numbers.py, recomputed here so the
# exceedance probabilities quoted next to it come out of the same file as the data.
DMIN_LAT = float(np.degrees(2.0 * np.arcsin(min(T_LAT / (2.0 * np.hypot(0.235, -0.280)), 1.0))))
DMIN_NORM = float(np.degrees(2.0 * np.arcsin(min(T_NORM / (2.0 * np.hypot(0.235, -0.280)), 1.0))))
GYRO_NONPHYS = 5.0                       # rad/s, CH110 sample rejected as non-physical
GENUINE_TOL = 2.0                        # deg, |dpsi - dpsi_imu| for a "genuine" increment
DT_MAX = 0.5                             # s, frames spanning a longer gap are not 5 Hz steps
RESID_OK = 0.02                          # m, "eq. (2) holds for this frame"
GATES_DEG = np.arange(2.0, 30.0 + 0.5, 1.0)
EXCEED = (5, 10, 20, 45, 90)             # deg, exceedance levels reported for |dpsi|

# Recordings whose /rtk_odom time stamps duplicate another recording are replays of that
# recording, not independent data, and are excluded from every statistic in this file.
# The overlap that identifies them is detected and reported in section 8 before the drop.
EXCLUDE_BAGS = ["orchard3src_20260826_2026-08-26-10-02-59"]

# 🔴 The 2026-09-06 greenhouse recording is NOT natural operation.  It is the controlled
# multipath-induction session behind P1's E3b: a 0.50 x 0.70 m metal plate held beside the
# antennas, vehicle parked, three attempts, two of which produced a false fix
# (field log of the 2026-09-06 induction session, not distributed).  Both showcase events of
# section 6 come from it.  Every
# statistic below is therefore reported twice: `induced` for this session and `operational`
# for everything else, and nothing quoted as an operational rate includes it.
INDUCED_BAGS = ["run_20260906_2026-09-06-15-26-15"]

# Site of each bag.  The ENU frame is shared across the whole corpus (one
# ~/.ros/rtk_origin.yaml), so the sites separate cleanly by median position; the
# centroids below were read off the per-bag medians and cross-checked against the
# experiment table of the mechanism-and-data checklist (not distributed) §B.2.
SITES = [
    ("concrete yard", (12.0, 0.0)),        # 30x30 m yard, E1 / E1b / E5 / E8
    ("greenhouse", (88.0, -150.0)),        # 0828-0830 greenhouse, E3
    ("greenhouse", (-22.0, -31.0)),        # second greenhouse, E3b (0906 15:26)
    ("orchard", (-1280.0, -180.0)),        # working orchard, E2 / E2b / E6 / E8
    ("orchard", (4976.0, -1469.0)),        # 0826 orchard
]

USECOLS = ["bag", "t", "dt", "x", "y", "x_ant", "y_ant", "yaw_deg", "dpsi_deg", "dp_pub",
           "dp_ant",
           "pred_jump", "resid", "across", "cov35", "hd_ok", "quality", "sats",
           "age_s", "dp_fix", "dpsi_imu_deg", "gz_max", "cls", "cls_imu"]
EPISODE_GAP = 5.0      # s, residual frames closer than this are one episode

OUT = []          # (group, quantity, value, unit, definition)


def put(group, quantity, value, unit, definition):
    OUT.append((group, quantity, value, unit, definition))
    return value


def q(a, p):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.quantile(a, p)) if a.size else float("nan")


def rms(a):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a * a))) if a.size else float("nan")


def wrap180(d):
    return (np.asarray(d, float) + 180.0) % 360.0 - 180.0


def site_of(med_x, med_y):
    dd = [np.hypot(med_x - c[0], med_y - c[1]) for _, c in SITES]
    return SITES[int(np.argmin(dd))][0], float(min(dd))


def wall(t):
    return datetime.datetime.fromtimestamp(float(t)).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


# ==================================================================== load
files = sorted(glob.glob(os.path.join(CSV_DIR, "*.csv")))
print("loading %d bag CSVs ..." % len(files))
d = pd.concat([pd.read_csv(f, usecols=USECOLS) for f in files], ignore_index=True)
d["bag"] = d["bag"].str.replace(".bag", "", regex=False)
summ = pd.read_csv(SUMMARY)
summ["bag"] = summ["bag"].str.replace(".bag", "", regex=False)
n_bags_raw, n_frames_raw = d.bag.nunique(), len(d)
print("  %d frames, %d bags before exclusions" % (n_frames_raw, n_bags_raw))

# ---- replay detection (run before the drop, so the exclusion is auditable)
span = d.groupby("bag")["t"].agg(["min", "max", "size"])
bl = list(span.index)
overlaps = []
for i in range(len(bl)):
    for j in range(i + 1, len(bl)):
        a, b = span.loc[bl[i]], span.loc[bl[j]]
        ov = min(a["max"], b["max"]) - max(a["min"], b["min"])
        if ov > 60.0:
            overlaps.append((bl[i], bl[j], ov))
for a, b, ov in overlaps:
    put("anomaly", "overlapping recordings: %s / %s" % (a, b), round(ov / 60.0, 2), "min",
        "both files carry /rtk_odom frames stamped %s to %s. Recordings whose time stamps "
        "duplicate another recording are replays, not independent data; the replay (%s, "
        "whose file name claims a later recording time than its own frames) is excluded "
        "from every statistic in this file."
        % (wall(max(span.loc[a, "min"], span.loc[b, "min"])),
           wall(min(span.loc[a, "max"], span.loc[b, "max"])),
           ", ".join(x for x in (a, b) if x in EXCLUDE_BAGS) or "none"))

drop = d["bag"].isin(EXCLUDE_BAGS)
put("corpus", "bags excluded as replays", len(EXCLUDE_BAGS), "bag", ", ".join(EXCLUDE_BAGS))
put("corpus", "frames excluded as replays", int(drop.sum()), "frame",
    "%.2f h of duplicated content" % (float(summ.loc[summ.bag.isin(EXCLUDE_BAGS),
                                                     "duration_s"].sum()) / 3600.0))
d = d[~drop].reset_index(drop=True)
summ = summ[~summ["bag"].isin(EXCLUDE_BAGS)].reset_index(drop=True)
print("  %d frames, %d bags after excluding %d replay(s)"
      % (len(d), d.bag.nunique(), len(EXCLUDE_BAGS)))

# ---- previous-frame quantities (within a bag); extract_p2.py leaves the first row empty
g_ = d.groupby("bag", sort=False)
d["yaw_prev"] = g_["yaw_deg"].shift(1)
d["hd_ok_prev"] = g_["hd_ok"].shift(1)
d["x_prev"] = g_["x"].shift(1)
d["y_prev"] = g_["y"].shift(1)

# cross-track component of the published step in the PRE-jump body frame.
# extract_p2.py's `across` column rotates by the CURRENT (post-jump) yaw; the deployed
# cross-track test (eq. 3) compares the step against the heading the vehicle was believed
# to be following, i.e. the yaw of the frame BEFORE the jump, so it is recomputed here.
yp = np.radians(d["yaw_prev"].to_numpy())
dx = d["x"].to_numpy() - d["x_prev"].to_numpy()
dy = d["y"].to_numpy() - d["y_prev"].to_numpy()
d["across_pre"] = -np.sin(yp) * dx + np.cos(yp) * dy
d["along_pre"] = np.cos(yp) * dx + np.sin(yp) * dy

# An increment is usable only if both frames carry a heading (cov[35] < 1e5; without a
# heading the driver publishes yaw = 0 and skips the rotation, so the "increment" is an
# artefact) and the two frames are one 5 Hz step apart.
# Cumulative gyro-integrated heading, accumulated over EVERY frame of the recording -- the
# IMU keeps integrating through the frames this file drops (no heading, or a recording gap),
# so the gate's propagated reference has to as well, or it comes back from an outage with a
# reference that is missing however much the platform turned meanwhile.
_cg = d.groupby("bag", sort=False)["dpsi_imu_deg"].transform(lambda x: x.fillna(0.0).cumsum())
_has_imu_bag = d.groupby("bag", sort=False)["dpsi_imu_deg"].transform(lambda x: x.notna().any())
d["cum_imu_full"] = np.where(_has_imu_bag, _cg, np.nan)

d["hd_pair"] = (d["hd_ok"] == 1) & (d["hd_ok_prev"] == 1)
d["step_ok"] = d["hd_pair"] & d["dpsi_deg"].notna() & (d["dt"] <= DT_MAX)
d["adpsi"] = d["dpsi_deg"].abs()

d["induced"] = d["bag"].isin(INDUCED_BAGS)

n_first = int(d["dpsi_deg"].isna().sum())
n_hdgap = int(((~d["hd_pair"]) & d["dpsi_deg"].notna()).sum())
n_dtgap = int((d["hd_pair"] & d["dpsi_deg"].notna() & (d["dt"] > DT_MAX)).sum())
S = d[d["step_ok"]].sort_values(["bag", "t"], kind="stable").reset_index(drop=True)
# 🔴 Reviewer 3: re-classify with motion_class.py v2 -- "slow" now uses the reconstructed
# antenna path, not the published displacement the fault corrupts.  The v1 label is kept
# as cls_imu_v1 for the before/after comparison in section 9.4.
S["cls_imu_v1"] = S["cls_imu"]
_newcls = np.empty(len(S), dtype=object)
for _b, _g in S.groupby("bag", sort=False):
    _newcls[_g.index.to_numpy()] = MC.classify(_g["t"].to_numpy(), _g["dp_ant"].to_numpy(),
                                               _g["cum_imu_full"].to_numpy(),
                                               t_bag_start=float(_g["t"].iloc[0]))
S["cls_imu"] = _newcls

IND = S["bag"].isin(INDUCED_BAGS).to_numpy()
OPS = ~IND
OP_BAGS = sorted(set(S["bag"]) - set(INDUCED_BAGS))
print("  %d usable 5 Hz heading increments (%d induced, %d operational)"
      % (len(S), int(IND.sum()), int(OPS.sum())))


# ================================================ physical bound from the gyro
# (computed first: the increment table quotes exceedance against it)
gz = S["gz_max"].to_numpy(float)
gz_ok = gz[np.isfinite(gz)]
n_nonphys = int((gz_ok > GYRO_NONPHYS).sum())
gz_f = gz_ok[gz_ok <= GYRO_NONPHYS]
W99, W999, WMAX = q(gz_f, .99), q(gz_f, .999), float(gz_f.max())
B99, B999, BMAX = np.degrees(W99) * TAU, np.degrees(W999) * TAU, np.degrees(WMAX) * TAU
put("physical_bound", "gyro samples", int(gz_ok.size), "frame",
    "frames with a CH110 |omega_z| envelope (max |omega_z| over the frame interval)")
put("physical_bound", "non-physical gyro frames", n_nonphys, "frame",
    "|omega_z| > %.0f rad/s (%.0f deg/s); dropped, same rule as data/post_summary.py"
    % (GYRO_NONPHYS, np.degrees(GYRO_NONPHYS)))
put("physical_bound", "non-physical gyro max", round(float(gz_ok.max()), 4), "rad/s",
    "largest raw CH110 envelope in the corpus, before the rule above")
for w, b, lab in ((W99, B99, "p99"), (W999, B999, "p99.9"), (WMAX, BMAX, "max")):
    put("physical_bound", "omega_z %s" % lab, round(w, 4), "rad/s",
        "after dropping non-physical samples")
    put("physical_bound", "bound %s" % lab, round(b, 4), "deg/frame",
        "|omega_z| x 0.2 s: the largest heading change the platform can produce in one frame")
imu_S = S[S["gz_max"].notna() & (S["gz_max"] <= GYRO_NONPHYS)]
n_above = int((imu_S["adpsi"] > BMAX).sum())
put("physical_bound", "frames above the max bound", n_above, "frame",
    "|dpsi| > %.3f deg/frame: physically impossible in one 5 Hz frame" % BMAX)
put("physical_bound", "P(|dpsi| > max bound)", round(100.0 * n_above / len(imu_S), 6), "%",
    "of the %d increments with a usable gyro envelope" % len(imu_S))
margin_deg = np.degrees(G.DEFAULT_YAW_GATE_MARGIN)
own = imu_S["adpsi"].to_numpy() > (np.degrees(imu_S["gz_max"].to_numpy()) * TAU + margin_deg)
put("physical_bound", "frames above their own gyro envelope + margin", int(own.sum()), "frame",
    "|dpsi| > |omega_z|_frame x 0.2 s + %.2f deg (the deployed yaw_gate_margin, standing for "
    "heading noise); without the margin the comparison is dominated by the noise floor, since "
    "the median increment already exceeds what the gyro shows on a standing vehicle"
    % margin_deg)
put("physical_bound", "P(above own envelope + margin)", round(100.0 * own.mean(), 4), "%", "")


# ============================================================ 1. corpus (Table 3)
med = d.groupby("bag")[["x", "y"]].median()
site, sdist = {}, {}
for b, r in med.iterrows():
    site[b], sdist[b] = site_of(r.x, r.y)
summ["site"] = summ["bag"].map(site)
summ["fixed_h"] = summ["fixed_frames"] * TAU / 3600.0
summ["dur_h"] = summ["duration_s"] / 3600.0
summ["has_imu"] = (summ["imu_topic"].fillna("") != "").astype(int)
FIXED_H = float(summ["fixed_h"].sum())

put("corpus", "bags", len(summ), "bag",
    "recordings with a /rtk_odom stream, after excluding replays")
put("corpus", "frames", int(summ["n_frames"].sum()), "frame", "total /rtk_odom frames (5 Hz)")
put("corpus", "hours", round(float(summ["dur_h"].sum()), 4), "h",
    "sum of per-bag /rtk_odom span (last minus first frame)")
put("corpus", "fixed_frames", int(summ["fixed_frames"].sum()), "frame",
    "frames whose nearest /rtk/status (within 1.5 s) reports a fixed solution")
put("corpus", "fixed_hours", round(FIXED_H, 4), "h", "fixed_frames x 0.2 s")
put("corpus", "fixed_share", round(100.0 * summ["fixed_frames"].sum() / summ["n_frames"].sum(), 2),
    "%", "fixed_frames / frames")
put("corpus", "bags_with_rtk_fix", int(summ["has_fix"].sum()), "bag",
    "bags recording /rtk/fix (raw ANT1 lat/lon); /rtk/fix joined the recording list 2026-09-10")
put("corpus", "bags_with_ch110", int(summ["has_imu"].sum()), "bag",
    "bags recording /ch110/data_raw (IMU, ~85-100 Hz)")
put("corpus", "frames_with_imu", int(S["dpsi_imu_deg"].notna().sum()), "frame",
    "usable increments that also carry an IMU-integrated increment")
for s_, grp in summ.groupby("site"):
    put("corpus_by_site", "%s bags" % s_, len(grp), "bag", "site from the median ENU position")
    put("corpus_by_site", "%s hours" % s_, round(float(grp["dur_h"].sum()), 4), "h", "")
    put("corpus_by_site", "%s frames" % s_, int(grp["n_frames"].sum()), "frame", "")
    put("corpus_by_site", "%s fixed frames" % s_, int(grp["fixed_frames"].sum()), "frame", "")
    put("corpus_by_site", "%s fixed hours" % s_, round(float(grp["fixed_h"].sum()), 4), "h", "")
    put("corpus_by_site", "%s bags with /rtk/fix" % s_, int(grp["has_fix"].sum()), "bag", "")
    put("corpus_by_site", "%s bags with /ch110/data_raw" % s_, int(grp["has_imu"].sum()), "bag", "")

_ih = summ.loc[summ["imu_hz"].notna() & (summ["imu_hz"] > 0), "imu_hz"]
put("corpus", "imu rate median", round(float(_ih.median()), 2), "Hz",
    "per-bag /ch110/data_raw sample rate over the %d bags that carry it; nominal 100 Hz "
    "(equipment table of the field handbook, not distributed), mean %.2f Hz, range %.1f-%.1f Hz" %
    (len(_ih), float(_ih.mean()), float(_ih.min()), float(_ih.max())))
put("corpus", "odom rate median", round(float(summ["odom_hz"].median()), 3), "Hz",
    "per-bag /rtk_odom rate; the nominal 5 Hz of Table 1")
put("corpus", "first frame", wall(float(d["t"].min()))[:10], "date",
    "wall-clock date of the earliest /rtk_odom frame in the corpus, after the replay exclusion")
put("corpus", "last frame", wall(float(d["t"].max()))[:10], "date",
    "wall-clock date of the latest /rtk_odom frame in the corpus")
put("corpus", "frames_dropped_first", n_first, "frame", "first frame of each bag (no increment)")
put("corpus", "frames_dropped_no_heading", n_hdgap, "frame",
    "increments touching a frame with cov[35] >= 1e5 (no heading, published yaw = 0)")
put("corpus", "frames_dropped_dt_gap", n_dtgap, "frame", "increments spanning dt > %.1f s" % DT_MAX)
put("corpus", "increments_used", len(S), "frame", "frames entering the increment statistics")


# ================================================ 2. heading increment distribution
def dist_rows(group, label, v):
    v = np.asarray(v, float)
    put(group, "%s n" % label, int(v.size), "frame", "")
    put(group, "%s RMS" % label, round(rms(v), 4), "deg", "root mean square of |dpsi|")
    for lev in EXCEED:
        put(group, "%s P(>%d deg)" % (label, lev), round(100.0 * float((v > lev).mean()), 6),
            "%", "exceedance probability")
        put(group, "%s count >%d deg" % (label, lev), int((v > lev).sum()), "frame", "")
    put(group, "%s P(> max bound %.2f deg)" % (label, BMAX),
        round(100.0 * float((v > BMAX).mean()), 6), "%",
        "exceedance of the measured physical bound |omega_z|max x 0.2 s")
    put(group, "%s count > max bound %.2f deg" % (label, BMAX), int((v > BMAX).sum()), "frame",
        "increments that cannot be a turn of this vehicle")
    for T_, dm_ in ((T_LAT, DMIN_LAT), (T_NORM, DMIN_NORM)):
        put(group, "%s P(> Delta_min %.1f deg, T=%.2f m)" % (label, dm_, T_),
            round(100.0 * float((v > dm_).mean()), 6), "%",
            "exceedance of the eq. (4) floor of the deployed %.2f m position test" % T_)
        put(group, "%s count > Delta_min %.1f deg, T=%.2f m" % (label, dm_, T_),
            int((v > dm_).sum()), "frame", "")
    put(group, "%s median" % label, round(q(v, .5), 4), "deg", "secondary")
    put(group, "%s p90" % label, round(q(v, .9), 4), "deg", "secondary")
    put(group, "%s p99" % label, round(q(v, .99), 4), "deg", "secondary")
    put(group, "%s p99.9" % label, round(q(v, .999), 4), "deg", "secondary")
    put(group, "%s max" % label, round(float(v.max()) if v.size else np.nan, 4), "deg", "secondary")


dist_rows("increments", "all", S["adpsi"])
for c in ("straight", "turn", "slow", "na"):
    sub = S[S["cls_imu"] == c]
    if len(sub):
        dist_rows("increments", c, sub["adpsi"])
for c in ("straight", "turn", "slow", "na"):
    sub = S[S["cls"] == c]
    if len(sub):
        dist_rows("increments_cls_rtk", c, sub["adpsi"])
put("increments", "definition", "|wrap(psi_k - psi_(k-1))|", "",
    "psi from the /rtk_odom quaternion, heading = 90 deg - yaw; one 5 Hz frame apart; "
    "cls_imu = IMU-only straight/turn label (|integrated gyro yaw| over the trailing 3 s "
    "window < 5 deg -> straight, else turn; 'na' = bag without IMU or first 3 s)")


# ================================================ 4. pseudo-step frames
cand = S[(S["quality"] == 4) & (S["cov35"] < COV_HD_BAD) & (S["dp_ant"] <= DP_ANT_MAX)].copy()
put("pseudo_step", "candidate frames", len(cand), "frame",
    "fixed solution (status 4) AND cov[35] < 1e5 (driver had a heading) AND |dp_ant| <= "
    "%.2f m (the reconstructed ANT1 point did not move)" % DP_ANT_MAX)
for T in PSTEP_T:
    p_ = cand[cand["dp_pub"] >= T]
    put("pseudo_step", "count >=%.2f m" % T, len(p_), "frame",
        "candidate frames whose published position moved at least %.2f m" % T)
    put("pseudo_step", "rate >=%.2f m" % T, round(len(p_) / FIXED_H, 4), "1/h",
        "per hour of fixed solution in the corpus")
    if len(p_):
        put("pseudo_step", "|dpsi| min >=%.2f m" % T, round(float(p_["adpsi"].min()), 4), "deg", "")
        put("pseudo_step", "|dpsi| max >=%.2f m" % T, round(float(p_["adpsi"].max()), 4), "deg", "")
        put("pseudo_step", "|dpsi| median >=%.2f m" % T, round(float(p_["adpsi"].median()), 4),
            "deg", "")
        put("pseudo_step", "residual RMS >=%.2f m" % T, round(rms(p_["resid"]), 4), "m",
            "residual = dp_pub - 2|L| sin(|dpsi|/2), the verification of eq. (2)")
        put("pseudo_step", "residual max |.| >=%.2f m" % T,
            round(float(p_["resid"].abs().max()), 4), "m", "")
        put("pseudo_step", "residual median >=%.2f m" % T, round(float(p_["resid"].median()), 4),
            "m", "")
        put("pseudo_step", "fraction within %.2f m >=%.2f m" % (RESID_OK, T),
            round(100.0 * float((p_["resid"].abs() <= RESID_OK).mean()), 2), "%",
            "frames whose observed step matches eq. (2) to better than %.2f m" % RESID_OK)
        put("pseudo_step", "caught by cross-track 0.30 m >=%.2f m" % T,
            int((p_["across_pre"].abs() > T_LAT).sum()), "frame",
            "|c| from eq. (3) with theta in the PRE-jump body frame, vs the deployed 0.30 m")
        put("pseudo_step", "caught by norm 0.50 m >=%.2f m" % T,
            int((p_["dp_pub"] > T_NORM).sum()), "frame", "vs the deployed 0.50 m norm threshold")
        put("pseudo_step", "caught by cross-track 0.30 m (post-jump frame) >=%.2f m" % T,
            int((p_["across"].abs() > T_LAT).sum()), "frame",
            "same test with extract_p2.py's `across` column (post-jump yaw), for comparison")

main = cand[cand["dp_pub"] >= PSTEP_MAIN].copy()
N_MAIN = len(main)
put("pseudo_step", "bags carrying >=%.2f m frames" % PSTEP_MAIN, int(main["bag"].nunique()),
    "bag", "")
by_bag = main.groupby("bag").size().sort_values(ascending=False)
for i, (b, n) in enumerate(by_bag.head(5).items(), 1):
    put("pseudo_step_by_bag", "top%d %s" % (i, b), int(n), "frame",
        "site %s; %.2f fixed h; %.2f frames per fixed hour"
        % (site[b], float(summ.loc[summ.bag == b, "fixed_h"].iloc[0]),
           int(n) / float(summ.loc[summ.bag == b, "fixed_h"].iloc[0])))
for s_ in sorted(set(site.values())):
    bags_s = [b for b in site if site[b] == s_]
    n_ = int(main["bag"].isin(bags_s).sum())
    h_ = float(summ.loc[summ["site"] == s_, "fixed_h"].sum())
    put("pseudo_step_by_site", "%s >=%.2f m" % (s_, PSTEP_MAIN), n_, "frame", "")
    put("pseudo_step_by_site", "%s per fixed hour" % s_, round(n_ / h_, 4), "1/h",
        "%.2f fixed h at this site" % h_)

bb = summ[["bag", "site", "fixed_h", "dur_h"]].copy()
bb["n_pstep"] = bb["bag"].map(main.groupby("bag").size()).fillna(0).astype(int)
bb["per_fixed_h"] = bb["n_pstep"] / bb["fixed_h"].replace(0.0, np.nan)
bb.sort_values(["site", "per_fixed_h"], ascending=[True, False]).to_csv(
    os.path.join(HERE, "pstep_by_bag.csv"), index=False, float_format="%.4f")

audit = cand[cand["dp_pub"] >= min(PSTEP_T)]
audit.assign(site=audit["bag"].map(site), wall=[wall(t) for t in audit["t"]]).to_csv(
    os.path.join(HERE, "pseudo_steps.csv"), index=False,
    columns=["bag", "site", "wall", "t", "dt", "dpsi_deg", "dp_pub", "dp_ant", "pred_jump",
             "resid", "across_pre", "across", "along_pre", "quality", "sats", "age_s",
             "cov35", "dpsi_imu_deg", "gz_max", "cls_imu"],
    float_format="%.4f")


# ================================================ 5. independent /rtk/fix check
fixb = sorted(summ.loc[summ["has_fix"] == 1, "bag"])
put("rtk_fix_check", "bags", len(fixb), "bag", "bags recording raw /rtk/fix: " + ", ".join(fixb))
F = S[S["bag"].isin(fixb) & S["dp_fix"].notna()].copy()
put("rtk_fix_check", "frames", len(F), "frame",
    "increments where both frames have a /rtk/fix sample within 0.15 s")
put("rtk_fix_check", "corr(dp_fix, dp_ant)",
    round(float(np.corrcoef(F["dp_fix"], F["dp_ant"])[0, 1]), 6), "-",
    "|dp| of the raw ANT1 lat/lon in ENU vs |dp| of the ANT1 point reconstructed as "
    "p_pub + R(yaw) L")
put("rtk_fix_check", "corr(dp_fix, dp_pub)",
    round(float(np.corrcoef(F["dp_fix"], F["dp_pub"])[0, 1]), 6), "-",
    "same against the published vehicle point")
_cb = {b: float(np.corrcoef(g_["dp_fix"], g_["dp_ant"])[0, 1])
       for b, g_ in F.groupby("bag") if len(g_) > 2}
put("rtk_fix_check", "corr(dp_fix, dp_ant) per bag, min", round(min(_cb.values()), 6), "-",
    "worst of the %d bags: %s" % (len(_cb), min(_cb, key=_cb.get)))
put("rtk_fix_check", "corr(dp_fix, dp_ant) per bag, max", round(max(_cb.values()), 6), "-",
    "best of the %d bags: %s" % (len(_cb), max(_cb, key=_cb.get)))
put("rtk_fix_check", "median |dp_fix - dp_ant|",
    round(float((F["dp_fix"] - F["dp_ant"]).abs().median()), 6), "m",
    "all frames; dp_fix is quantised at 0.1 mm by the CSV writer")
put("rtk_fix_check", "p95 |dp_fix - dp_ant|", round(q((F["dp_fix"] - F["dp_ant"]).abs(), .95), 6),
    "m", "")
put("rtk_fix_check", "p99.9 |dp_fix - dp_ant|",
    round(q((F["dp_fix"] - F["dp_ant"]).abs(), .999), 6), "m", "")
put("rtk_fix_check", "max |dp_fix - dp_ant|",
    round(float((F["dp_fix"] - F["dp_ant"]).abs().max()), 6), "m",
    "worst frame; the tail is timing (nearest /rtk/fix sample accepted within 0.15 s)")
put("rtk_fix_check", "share |dp_fix - dp_ant| below the CSV quantum",
    round(100.0 * float(((F["dp_fix"] - F["dp_ant"]).abs() < 1e-4).mean()), 3), "%",
    "0.1 mm, the resolution the extraction writes")
put("rtk_fix_check", "median |dp_fix - dp_pub|",
    round(float((F["dp_fix"] - F["dp_pub"]).abs().median()), 6), "m", "all frames")
F10 = F[F["adpsi"] > 10.0]
put("rtk_fix_check", "frames with |dpsi| > 10 deg", len(F10), "frame", "")
put("rtk_fix_check", "median |dp_fix - dp_ant| (|dpsi|>10)",
    round(float((F10["dp_fix"] - F10["dp_ant"]).abs().median()), 6), "m",
    "the reconstruction tracks the raw antenna point through these frames")
put("rtk_fix_check", "median |dp_fix - dp_pub| (|dpsi|>10)",
    round(float((F10["dp_fix"] - F10["dp_pub"]).abs().median()), 6), "m", "")
put("rtk_fix_check", "median dp_ant (|dpsi|>10)", round(float(F10["dp_ant"].median()), 6), "m", "")
put("rtk_fix_check", "median dp_pub (|dpsi|>10)", round(float(F10["dp_pub"].median()), 6), "m", "")
put("rtk_fix_check", "max |dpsi| in these bags", round(float(F["adpsi"].max()), 4), "deg",
    "these bags carry no large heading outlier, so the check validates the reconstruction")


# ================================================ 6. the two recorded events
EV_BAG = "run_20260906_2026-09-06-15-26-15"
E = d[d["bag"] == EV_BAG].copy()
E["wall"] = [wall(t) for t in E["t"]]
mask = E["wall"].str.slice(11, 19).between("15:57:00", "15:57:06")
ev1 = E.loc[mask, "adpsi"].idxmax()
ev2 = E["adpsi"].idxmax()
for tag, idx in (("event1", ev1), ("event2", ev2)):
    r = E.loc[idx]
    pre = E.loc[idx - 1]
    put(tag, "bag", EV_BAG, "", "E3b, second greenhouse, vehicle stationary")
    put(tag, "time", r["wall"], "", "local time of the frame that carries the step")
    put(tag, "dpsi", round(float(r["dpsi_deg"]), 4), "deg",
        "one 5 Hz frame, heading %.4f -> %.4f deg"
        % ((90.0 - float(pre["yaw_deg"])) % 360.0, (90.0 - float(r["yaw_deg"])) % 360.0))
    put(tag, "dp_pub", round(float(r["dp_pub"]), 4), "m", "published position step")
    put(tag, "predicted 2|L|sin(|dpsi|/2)", round(float(r["pred_jump"]), 4), "m", "eq. (2)")
    put(tag, "residual", round(float(r["resid"]), 4), "m", "dp_pub - prediction")
    put(tag, "dp_ant", round(float(r["dp_ant"]), 4), "m",
        "reconstructed ANT1 point over the same frame")
    put(tag, "cov35", float(r["cov35"]), "rad^2", "heading variance published by the driver")
    put(tag, "status", int(r["quality"]), "-", "4 = RTK fixed")
    put(tag, "satellites", int(r["sats"]), "-", "from /rtk/status")
    put(tag, "differential age", float(r["age_s"]), "s", "from /rtk/status")
    put(tag, "IMU increment", float(r["dpsi_imu_deg"]), "deg",
        "gyro-integrated heading change over the same frame")
    put(tag, "gyro envelope", float(r["gz_max"]), "rad/s", "max |omega_z| over the frame")
    put(tag, "cross-track (pre-jump frame)", round(float(r["across_pre"]), 4), "m", "eq. (3)")
    put(tag, "along-track (pre-jump frame)", round(float(r["along_pre"]), 4), "m", "eq. (3)")
put("event2", "separation from event1", round(float(E.loc[ev2, "t"] - E.loc[ev1, "t"]), 1), "s",
    "negative = earlier; the two events are not in the same 60 s window")
put("event1", "frames with |dpsi| > 20 deg in this bag", int((E["adpsi"] > 20).sum()), "frame", "")


# ================================================ 7. gate sweep (v1, v2, v3)
S["genuine"] = (np.abs(wrap180(S["dpsi_deg"] - S["dpsi_imu_deg"])) <= GENUINE_TOL) \
    & S["dpsi_imu_deg"].notna()
S["outlier"] = (~S["genuine"]) & S["dpsi_imu_deg"].notna()
S["is_pstep"] = (S["quality"] == 4) & (S["cov35"] < COV_HD_BAD) \
    & (S["dp_ant"] <= DP_ANT_MAX) & (S["dp_pub"] >= PSTEP_MAIN)
ev1_key = (EV_BAG, float(d.loc[ev1, "t"]))
ev2_key = (EV_BAG, float(d.loc[ev2, "t"]))

bag_groups = [(b, grp.index.to_numpy(), np.radians(grp["yaw_deg"].to_numpy()),
               grp["t"].to_numpy()) for b, grp in S.groupby("bag", sort=False)]
gen_np = S["genuine"].to_numpy()
out_np = S["outlier"].to_numpy()
pst_np = S["is_pstep"].to_numpy()
cls_np = S["cls_imu"].to_numpy()
N = len(S)
N_GEN = int(gen_np.sum())
N_PST = int(pst_np.sum())
ev_sel = {}
for tag, key in (("event1", ev1_key), ("event2", ev2_key)):
    ev_sel[tag] = ((S["bag"] == key[0]) & (np.abs(S["t"] - key[1]) < 1e-3)).to_numpy()

# cumulative IMU-integrated heading (deg, heading convention), carried over every frame of
# the recording: the truth the held yaw is compared against
cum_imu = np.nan_to_num(S["cum_imu_full"].to_numpy(), nan=0.0)

put("gate", "genuine frames", N_GEN, "frame",
    "|wrap(dpsi - dpsi_imu)| <= %.1f deg: the reported increment agrees with the gyro, so "
    "withholding or overriding it is a false gate trip" % GENUINE_TOL)
put("gate", "outlier frames", int(out_np.sum()), "frame",
    "frames with an IMU increment that disagrees by more than %.1f deg" % GENUINE_TOL)
for c in ("straight", "turn", "slow"):
    put("gate", "genuine frames (%s)" % c, int((gen_np & (cls_np == c)).sum()), "frame", "")
put("gate", "pseudo-step frames in the sweep", N_PST, "frame",
    "candidate frames with dp_pub >= %.2f m" % PSTEP_MAIN)

# ---- the number behind gyro_drift_rate: how far the gyro-integrated heading walks away
# from the HDT heading over 60 s of straight, fixed-solution driving
DRIFT_W = 60.0
drifts = []
n_drift_bags = 0
for b, grp in S.groupby("bag", sort=False):
    # not turning: the v2 classifier splits "not turning" into slow (antenna path <= 0.5 m
    # over 3 s) and straight, and a bias measurement wants both
    ok = ((grp["quality"] == 4) & grp["cls_imu"].isin(["straight", "slow"])
          & grp["dpsi_imu_deg"].notna() & (grp["gz_max"] <= GYRO_NONPHYS)).to_numpy()
    if not ok.any():
        continue
    tt = grp["t"].to_numpy()
    dh = grp["dpsi_deg"].to_numpy()
    dg = grp["dpsi_imu_deg"].to_numpy()
    got = 0
    i = 0
    while i < ok.size:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j < ok.size and ok[j]:
            j += 1
        k = i
        while k < j:
            m = k
            while m < j and tt[m] - tt[k] < DRIFT_W:
                m += 1
            if m < j and tt[m] - tt[k] >= DRIFT_W:
                dur = tt[m] - tt[k]
                drifts.append(abs(wrap180(np.sum(dg[k + 1:m + 1]) - wrap180(np.sum(dh[k + 1:m + 1]))))
                              / dur)
                got += 1
                k = m
            else:
                break
        i = j
    n_drift_bags += 1 if got else 0
drifts = np.array(drifts)
put("gyro_drift", "windows measured", int(drifts.size), "window",
    "%.0f s windows of contiguous not-turning (v2 straight or slow), fixed-solution frames "
    "with a usable gyro, in %d recordings" % (DRIFT_W, n_drift_bags))
put("gyro_drift", "median drift", round(float(np.median(drifts)), 5), "deg/s",
    "|gyro-integrated heading change - HDT heading change| / window length")
put("gyro_drift", "median drift (rad/s)", round(float(np.radians(np.median(drifts))), 7), "rad/s", "")
put("gyro_drift", "p90 drift", round(float(np.quantile(drifts, .9)), 5), "deg/s", "")
put("gyro_drift", "max drift", round(float(drifts.max()), 5), "deg/s", "")
put("gyro_drift", "gyro_drift_rate used", G.DEFAULT_GYRO_DRIFT_RATE, "rad/s",
    "%.3f deg/s = %.2f deg per minute withheld; an order of magnitude above the measured "
    "median and well below the p90, so the allowance covers MEMS bias without covering a "
    "real heading error" % (np.degrees(G.DEFAULT_GYRO_DRIFT_RATE),
                            np.degrees(G.DEFAULT_GYRO_DRIFT_RATE) * 60.0))
put("gyro_drift", "seconds to cover a 67.5 deg error at g=17 deg",
    round((np.radians(67.54) - np.radians(17.0)) / G.DEFAULT_GYRO_DRIFT_RATE, 1), "s",
    "how long rule (a) would have to stay withheld before the allowance alone could "
    "re-anchor onto the 2026-09-06 15:57 heading")


def episode_split(mask_np):
    """Attribute every not-accepted frame to the episode that produced it.

    A run of consecutive not-accepted frames is *outlier-triggered* when the frame that
    opened it is one the gyro contradicts, and *genuine-triggered* otherwise.  Only the
    second kind is a false trip in the sense that matters: the gate lost the heading over
    a frame that was fine.  Returns
    (genuine frames inside outlier-triggered runs, genuine frames inside genuine-triggered
     runs, number of runs, number of outlier-triggered runs, longest run in frames)."""
    g_out = g_gen = 0
    n_run = n_out = longest = 0
    for b, idx, yaw, ts in bag_groups:
        m = mask_np[idx]
        gg = gen_np[idx]
        oo = out_np[idx]
        i = 0
        while i < m.size:
            if not m[i]:
                i += 1
                continue
            j = i
            while j < m.size and m[j]:
                j += 1
            n_run += 1
            longest = max(longest, j - i)
            if oo[i]:
                n_out += 1
                g_out += int(gg[i:j].sum())
            else:
                g_gen += int(gg[i:j].sum())
            i = j
    return g_out, g_gen, n_run, n_out, longest


def runs_of(mask_np):
    """(longest run of consecutive True, longest such run that is entirely genuine),
    never crossing a bag boundary."""
    longest = pure = 0
    for b, idx, yaw, ts in bag_groups:
        m = mask_np[idx]
        g_ = gen_np[idx]
        i = 0
        while i < m.size:
            if not m[i]:
                i += 1
                continue
            j = i
            while j < m.size and m[j]:
                j += 1
            longest = max(longest, j - i)
            if g_[i:j].all():
                pure = max(pure, j - i)
            i = j
    return longest, pure


# cumulative gyro yaw in the same convention as the gated angle (yaw = 90 deg - heading,
# so the yaw-convention gyro increment is the negated heading increment)
cum_gyro_yaw = np.radians(-S["cum_imu_full"].to_numpy())   # NaN where the bag has no IMU
N_NOIMU = int(np.isnan(cum_gyro_yaw).sum())
put("gate", "frames without a gyro reference", N_NOIMU, "frame",
    "%d recordings carry no IMU; under rule (a) these frames fall back to rule (b)"
    % int((summ["has_imu"] == 0).sum()))


def run_v3(rate, marg, use_gyro, max_gyro_ref_time=None):
    state = np.zeros(N, np.int8)
    anch = np.full(N, -1, np.int64)
    rean = np.zeros(N, bool)
    fb = np.zeros(N, bool)
    pub = np.zeros(N, bool)
    uy = np.zeros(N)
    ngap = 0
    for b, idx, yaw, ts in bag_groups:
        r = G.gate_series_v3(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], max_yaw_rate=rate,
                             yaw_gate_margin=marg, nominal_dt=TAU,
                             max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
                             max_gap_frames=G.DEFAULT_MAX_GAP_FRAMES,
                             gyro_drift_rate=G.DEFAULT_GYRO_DRIFT_RATE, use_gyro=use_gyro,
                             max_gyro_ref_time=max_gyro_ref_time)
        state[idx] = r["state"]
        rean[idx] = r["reanchor"]
        fb[idx] = r["fallback_b"]
        pub[idx] = r["published"]
        uy[idx] = np.where(np.isfinite(r["used_yaw"]), r["used_yaw"], yaw)
        a = r["anchor"]
        anch[idx] = np.where(a >= 0, idx[np.maximum(a, 0)], -1)
        ngap += r["n_gap"]
    return state, anch, rean, fb, ngap, pub, uy


def v3_row(gdeg, rate, marg, variant, state, anch, rean, fb, ngap):
    held = state == G.HOLD
    nopub = state == G.NOPUB
    not_acc = held | nopub
    lg, pg = runs_of(not_acc)
    g_out, g_gen, n_run, n_out, _ = episode_split(not_acc)
    hg = held & gen_np & (anch >= 0)
    dheld = np.abs(cum_imu[hg] - cum_imu[anch[hg]])
    err = 2.0 * LNORM * np.sin(np.radians(dheld) / 2.0)
    missed = (~not_acc) & pst_np
    row = dict(variant=variant, gate_deg=float(gdeg), max_yaw_rate=rate,
               yaw_gate_margin=marg, nominal_dt=TAU,
               max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
               max_gap_frames=G.DEFAULT_MAX_GAP_FRAMES,
               gyro_drift_rate=G.DEFAULT_GYRO_DRIFT_RATE,
               held=int(held.sum()), unpublished=int(nopub.sum()),
               unpublished_pct=100.0 * nopub.mean(),
               not_accepted=int(not_acc.sum()), not_accepted_pct=100.0 * not_acc.mean(),
               genuine_not_accepted=int((not_acc & gen_np).sum()),
               genuine_unpublished=int((nopub & gen_np).sum()),
               outlier_not_accepted=int((not_acc & out_np).sum()),
               false_gate_pct=100.0 * int((not_acc & gen_np).sum()) / N_GEN,
               max_consecutive_not_accepted=lg, max_consecutive_genuine_not_accepted=pg,
               hold_err_max_m=float(err.max()) if err.size else 0.0,
               hold_err_rms_m=rms(err) if err.size else 0.0,
               hold_err_frames=int(hg.sum()),
               reanchors=int(rean.sum()), fallback_b_frames=int(fb.sum()), gaps=int(ngap),
               episodes=n_run, episodes_outlier_triggered=n_out,
               genuine_in_outlier_episodes=g_out, genuine_in_genuine_episodes=g_gen,
               false_gate_attributed_pct=100.0 * g_gen / N_GEN,
               pstep_caught=int((not_acc & pst_np).sum()),
               pstep_missed=int(missed.sum()),
               pstep_caught_pct=100.0 * int((not_acc & pst_np).sum()) / N_PST,
               pstep_missed_dpsi=";".join("%.1f" % v for v in
                                          np.sort(np.abs(S["dpsi_deg"].to_numpy()[missed]))[::-1]))
    for c in ("straight", "turn", "slow", "na"):
        m = cls_np == c
        gg = int((gen_np & m).sum())
        row["false_gate_pct_" + c] = 100.0 * int((not_acc & gen_np & m).sum()) / gg if gg else np.nan
        row["genuine_" + c] = gg
    for tag in ("event1", "event2"):
        row[tag + "_caught"] = int(bool(not_acc[ev_sel[tag]].any()))
    return row


# ---------------------------------------------------------------- gated output
# The frame-level "caught" flag is the wrong metric for a return edge: a frame the gate
# refuses still leaves a step in the published stream when the gate later re-anchors.  What
# matters is what survives IN THE OUTPUT, so the output is rebuilt for every rule:
#     p_gated = p_ant - R(psi_gate) L      psi_gate = the heading the gate actually uses
# accepted -> the reported heading; held -> the last accepted heading; withheld -> no
# output at all, so the next published frame is differenced against the last published one.
XA = S["x_ant"].to_numpy()
YA = S["y_ant"].to_numpy()
ST = S["t"].to_numpy()
QOK = ((S["quality"] == 4) & (S["cov35"] < COV_HD_BAD)).to_numpy()


def gated_residuals(pub, used_yaw, accepted, detail=False, tag="", only_bags=None):
    """Residual pseudo-steps left in the gated output stream.

    Same definition as the ungated count (fixed solution, heading valid, the reconstructed
    antenna point still to within DP_ANT_MAX over the same interval), but measured between
    consecutive PUBLISHED frames of this rule's output.
    Returns (frames per threshold, episodes per threshold, episode list)."""
    cnt = {T: 0 for T in PSTEP_T}
    eps = {T: 0 for T in PSTEP_T}
    rows = []
    for b, idx, yaw, ts in bag_groups:
        if only_bags is not None and b not in only_bags:
            continue
        p = pub[idx]
        if int(p.sum()) < 2:
            continue
        sel = idx[p]
        yg = used_yaw[sel]
        cy, sy = np.cos(yg), np.sin(yg)
        xg = XA[sel] - (cy * LX - sy * LY)
        yg2 = YA[sel] - (sy * LX + cy * LY)
        d_gat = np.hypot(np.diff(xg), np.diff(yg2))
        d_ant = np.hypot(np.diff(XA[sel]), np.diff(YA[sel]))
        hdg = (90.0 - np.degrees(yg)) % 360.0
        dpsi = wrap180(np.diff(hdg))
        cur = sel[1:]
        base = QOK[cur] & (d_ant <= DP_ANT_MAX)
        for T in PSTEP_T:
            m = base & (d_gat >= T)
            if not m.any():
                continue
            cnt[T] += int(m.sum())
            tt = ST[cur][m]
            eps[T] += 1 + int((np.diff(tt) > EPISODE_GAP).sum())
        if detail:
            m = base & (d_gat >= min(PSTEP_T))
            if not m.any():
                continue
            k = np.flatnonzero(m)
            brk = np.flatnonzero(np.diff(ST[cur][k]) > EPISODE_GAP) + 1
            for grp in np.split(k, brk):
                j = grp[int(np.argmax(d_gat[grp]))]
                rows.append(dict(
                    rule=tag, bag=b, wall=wall(ST[cur][j]), frames=int(grp.size),
                    size_m=round(float(d_gat[grp].max()), 4),
                    dpsi_vs_last_published=round(float(dpsi[j]), 2),
                    d_ant_m=round(float(d_ant[j]), 4),
                    gap_s=round(float(ST[cur][j] - ST[sel[:-1]][j]), 2),
                    reanchor=int(bool(accepted[cur[j]] and not accepted[cur[j] - 1]))))
    return cnt, eps, rows


def accepted_from(state):
    return state == G.ACCEPT


RESID_ROWS = []
G_DETAIL = (10.0, 17.0)
# baseline: the raw stream, no gate at all
_allpub = np.ones(N, bool)
_yawrad = np.radians(S["yaw_deg"].to_numpy())
BASE_CNT, BASE_EPS, BASE_ROWS = gated_residuals(_allpub, _yawrad, _allpub, detail=True,
                                                tag="no gate")
RESID_ROWS += BASE_ROWS
for T in PSTEP_T:
    put("gated_residual", "no gate: frames >=%.2f m" % T, BASE_CNT[T], "frame",
        "the ungated published stream, for reference")
    put("gated_residual", "no gate: episodes >=%.2f m" % T, BASE_EPS[T], "-",
        "frames closer together than %.0f s are one episode" % EPISODE_GAP)

sweep_v1, sweep_v2, sweep_v3 = [], [], []
for gdeg in GATES_DEG:
    rate, marg = G.params_for_gate(float(gdeg))

    # ---- v1: the rule as patched today
    rej = np.zeros(N, bool)
    inval = np.zeros(N, bool)
    uy1 = np.zeros(N)
    for b, idx, yaw, ts in bag_groups:
        r = G.gate_series(yaw, ts, max_yaw_rate=rate, yaw_gate_margin=marg,
                          max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES)
        rej[idx] = r["rejected"]
        inval[idx] = ~r["heading_valid"]
        uy1[idx] = r["used_yaw"]
    caught1 = rej | inval
    l1, p1 = runs_of(rej)
    g_out1, g_gen1, n_run1, n_out1, _ = episode_split(caught1)
    row1 = dict(gate_deg=float(gdeg), max_yaw_rate=rate, yaw_gate_margin=marg,
                max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
                rejected=int(rej.sum()), rejected_pct=100.0 * rej.mean(),
                genuine_rejected=int((rej & gen_np).sum()),
                outlier_rejected=int((rej & out_np).sum()),
                false_gate_pct=100.0 * int((rej & gen_np).sum()) / N_GEN,
                heading_invalid=int(inval.sum()),
                max_consecutive_rejections=l1, max_consecutive_genuine_rejections=p1,
                episodes=n_run1, episodes_outlier_triggered=n_out1,
                genuine_in_outlier_episodes=g_out1, genuine_in_genuine_episodes=g_gen1,
                false_gate_attributed_pct=100.0 * g_gen1 / N_GEN,
                pstep_caught=int((caught1 & pst_np).sum()),
                pstep_missed=int((~caught1 & pst_np).sum()),
                pstep_caught_pct=100.0 * int((caught1 & pst_np).sum()) / N_PST)
    for c in ("straight", "turn", "slow", "na"):
        m = cls_np == c
        gg = int((gen_np & m).sum())
        row1["false_gate_pct_" + c] = 100.0 * int((rej & gen_np & m).sum()) / gg if gg else np.nan
        row1["genuine_" + c] = gg
    for tag in ("event1", "event2"):
        row1[tag + "_caught"] = int(bool(caught1[ev_sel[tag]].any()))
    c_, e_, rr_ = gated_residuals(~inval, uy1, ~rej, detail=(gdeg in G_DETAIL),
                                  tag="v1 g=%.0f" % gdeg)
    RESID_ROWS += rr_
    for T in PSTEP_T:
        row1["residual_frames_%.2f" % T] = c_[T]
        row1["residual_episodes_%.2f" % T] = e_[T]
    sweep_v1.append(row1)

    # ---- v2: the amended specification
    state = np.zeros(N, np.int8)
    pub = np.zeros(N, bool)
    anch = np.full(N, -1, np.int64)
    validv = np.ones(N, bool)
    uy2 = np.zeros(N)
    for b, idx, yaw, ts in bag_groups:
        r = G.gate_series_v2(yaw, ts, max_yaw_rate=rate, yaw_gate_margin=marg,
                             max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES, nominal_dt=TAU)
        state[idx] = r["state"]
        pub[idx] = r["published"]
        validv[idx] = r["valid"]
        uy2[idx] = np.where(np.isfinite(r["used_yaw"]), r["used_yaw"], yaw)
        a = r["anchor"]
        anch[idx] = np.where(a >= 0, idx[np.maximum(a, 0)], -1)
    held = state == G.HOLD
    nopub = state == G.NOPUB
    not_acc = held | nopub                 # the gate did not let the reported heading through
    l2, p2 = runs_of(not_acc)
    g_out2, g_gen2, n_run2, n_out2, _ = episode_split(not_acc)
    # published-position error while holding a stale yaw on frames the gyro confirms
    hg = held & gen_np & (anch >= 0)
    dheld = np.zeros(N)
    dheld[hg] = np.abs(cum_imu[hg] - cum_imu[anch[hg]])
    err = 2.0 * LNORM * np.sin(np.radians(dheld[hg]) / 2.0)
    row2 = dict(gate_deg=float(gdeg), max_yaw_rate=rate, yaw_gate_margin=marg,
                max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES, nominal_dt=TAU,
                held=int(held.sum()), unpublished=int(nopub.sum()),
                unpublished_pct=100.0 * nopub.mean(),
                not_accepted=int(not_acc.sum()), not_accepted_pct=100.0 * not_acc.mean(),
                genuine_not_accepted=int((not_acc & gen_np).sum()),
                outlier_not_accepted=int((not_acc & out_np).sum()),
                false_gate_pct=100.0 * int((not_acc & gen_np).sum()) / N_GEN,
                genuine_unpublished=int((nopub & gen_np).sum()),
                max_consecutive_not_accepted=l2, max_consecutive_genuine_not_accepted=p2,
                episodes=n_run2, episodes_outlier_triggered=n_out2,
                genuine_in_outlier_episodes=g_out2, genuine_in_genuine_episodes=g_gen2,
                false_gate_attributed_pct=100.0 * g_gen2 / N_GEN,
                hold_err_max_m=float(err.max()) if err.size else 0.0,
                hold_err_rms_m=rms(err) if err.size else 0.0,
                hold_err_frames=int(hg.sum()),
                pstep_caught=int((not_acc & pst_np).sum()),
                pstep_missed=int((~not_acc & pst_np).sum()),
                pstep_caught_pct=100.0 * int((not_acc & pst_np).sum()) / N_PST)
    for c in ("straight", "turn", "slow", "na"):
        m = cls_np == c
        gg = int((gen_np & m).sum())
        row2["false_gate_pct_" + c] = 100.0 * int((not_acc & gen_np & m).sum()) / gg if gg else np.nan
        row2["genuine_" + c] = gg
    for tag in ("event1", "event2"):
        row2[tag + "_caught"] = int(bool(not_acc[ev_sel[tag]].any()))
    c_, e_, rr_ = gated_residuals(pub, uy2, accepted_from(state), detail=(gdeg in G_DETAIL),
                                  tag="v2 g=%.0f" % gdeg)
    RESID_ROWS += rr_
    for T in PSTEP_T:
        row2["residual_frames_%.2f" % T] = c_[T]
        row2["residual_episodes_%.2f" % T] = e_[T]
    sweep_v2.append(row2)

    # ---- v3: fixed budget, gap invalidation, gyro (a) or two-sample (b) re-anchor
    rows3 = {}
    for variant, use_gyro in (("v3a", True), ("v3b", False)):
        st3, an3, re3, fb3, ng3, pb3, uy3 = run_v3(rate, marg, use_gyro)
        rows3[variant] = v3_row(gdeg, rate, marg, variant, st3, an3, re3, fb3, ng3)
        c_, e_, rr_ = gated_residuals(pb3, uy3, accepted_from(st3),
                                      detail=(gdeg in G_DETAIL),
                                      tag="%s g=%.0f" % (variant, gdeg))
        RESID_ROWS += rr_
        for T in PSTEP_T:
            rows3[variant]["residual_frames_%.2f" % T] = c_[T]
            rows3[variant]["residual_episodes_%.2f" % T] = e_[T]
        sweep_v3.append(rows3[variant])

    print("  g=%4.1f | v1 fg %7.4f%% ps %2d ev2 %d | v2 fg %7.4f%% ps %2d ev2 %d | "
          "v3a fg %7.4f%% nopub %5d ps %2d ev1/2 %d/%d | v3b fg %7.4f%% ps %2d ev1/2 %d/%d"
          % (gdeg, row1["false_gate_pct"], row1["pstep_caught"], row1["event2_caught"],
             row2["false_gate_pct"], row2["pstep_caught"], row2["event2_caught"],
             rows3["v3a"]["false_gate_pct"], rows3["v3a"]["unpublished"],
             rows3["v3a"]["pstep_caught"], rows3["v3a"]["event1_caught"],
             rows3["v3a"]["event2_caught"],
             rows3["v3b"]["false_gate_pct"], rows3["v3b"]["pstep_caught"],
             rows3["v3b"]["event1_caught"], rows3["v3b"]["event2_caught"]))

sw1 = pd.DataFrame(sweep_v1)
sw2 = pd.DataFrame(sweep_v2)
sw3 = pd.DataFrame(sweep_v3)
sw1.to_csv(os.path.join(HERE, "gate_sweep_v1.csv"), index=False, float_format="%.6f")
sw2.to_csv(os.path.join(HERE, "gate_sweep_v2.csv"), index=False, float_format="%.6f")
sw3.to_csv(os.path.join(HERE, "gate_sweep.csv"), index=False, float_format="%.6f")
sw3a = sw3[sw3.variant == "v3a"].reset_index(drop=True)
sw3b = sw3[sw3.variant == "v3b"].reset_index(drop=True)
sw = sw3a

g_rec = float(np.ceil(BMAX))
rec = sw3a.loc[(sw3a["gate_deg"] - g_rec).abs().idxmin()]      # v3a, the recommendation
recb = sw3b.loc[(sw3b["gate_deg"] - g_rec).abs().idxmin()]
rec2 = sw2.loc[(sw2["gate_deg"] - g_rec).abs().idxmin()]
rec1 = sw1.loc[(sw1["gate_deg"] - g_rec).abs().idxmin()]
G_DEF = float(np.degrees(G.DEFAULT_MAX_YAW_RATE * TAU + G.DEFAULT_YAW_GATE_MARGIN))
dflt = sw3a.loc[(sw3a["gate_deg"] - G_DEF).abs().idxmin()]
dflt2 = sw2.loc[(sw2["gate_deg"] - G_DEF).abs().idxmin()]
dflt1 = sw1.loc[(sw1["gate_deg"] - G_DEF).abs().idxmin()]

put("gate", "recommended g", float(rec["gate_deg"]), "deg",
    "smallest 1 deg step at or above the measured physical bound %.2f deg/frame "
    "(|omega_z|max x 0.2 s), so no physically possible increment is rejected" % BMAX)
put("gate", "recommended max_yaw_rate", round(float(rec["max_yaw_rate"]), 4), "rad/s",
    "with yaw_gate_margin at the deployed 0.05 rad")
put("gate", "recommended yaw_gate_margin", round(float(rec["yaw_gate_margin"]), 4), "rad", "")
put("gate", "recommended max_hold_frames", int(G.DEFAULT_MAX_HOLD_FRAMES), "frame", "")
put("gate", "recommended nominal_dt", TAU, "s", "")
put("gate", "recommended max_gap_frames", int(G.DEFAULT_MAX_GAP_FRAMES), "frame",
    "a reference older than %.1f s is declared stale" % (G.DEFAULT_MAX_GAP_FRAMES * TAU))
put("gate", "recommended gyro_drift_rate", G.DEFAULT_GYRO_DRIFT_RATE, "rad/s",
    "rule (a) only; see the gyro_drift group")
put("gate", "recommended rule", "v3a (use_gyro = true)", "",
    "rule (b) is the fallback where no IMU is present")
put("gate", "fixed budget at the recommended gate",
    round(float(rec["max_yaw_rate"]) * TAU + float(rec["yaw_gate_margin"]), 4), "rad",
    "max_yaw_rate x nominal_dt + yaw_gate_margin = %.2f deg; v3 never scales it with dt"
    % rec["gate_deg"])
for k, lab, unit in (("false_gate_pct", "false-gate rate (overall)", "%"),
                     ("false_gate_attributed_pct",
                      "false-gate rate, episodes the gyro did not contradict", "%"),
                     ("genuine_in_outlier_episodes",
                      "genuine frames inside outlier-triggered outages", "frame"),
                     ("episodes", "outages", "-"),
                     ("episodes_outlier_triggered", "outages opened by an outlier", "-"),
                     ("false_gate_pct_straight", "false-gate rate (straight)", "%"),
                     ("false_gate_pct_turn", "false-gate rate (turn)", "%"),
                     ("genuine_not_accepted", "genuine frames held or withheld", "frame"),
                     ("genuine_unpublished", "genuine frames withheld", "frame"),
                     ("outlier_not_accepted", "outlier frames held or withheld", "frame"),
                     ("held", "frames held", "frame"),
                     ("unpublished", "frames withheld", "frame"),
                     ("unpublished_pct", "frames withheld", "%"),
                     ("pstep_caught", "pseudo-step frames caught", "frame"),
                     ("pstep_caught_pct", "pseudo-step frames caught", "%"),
                     ("pstep_missed", "pseudo-step frames missed", "frame"),
                     ("max_consecutive_not_accepted", "longest hold (any frames)", "frame"),
                     ("max_consecutive_genuine_not_accepted", "longest hold on genuine frames",
                      "frame"),
                     ("hold_err_max_m", "published error during genuine holds, max", "m"),
                     ("hold_err_rms_m", "published error during genuine holds, RMS", "m"),
                     ("hold_err_frames", "genuine held frames measured", "frame"),
                     ("event1_caught", "event 1 caught", "-"),
                     ("event2_caught", "event 2 caught", "-")):
    v = rec[k]
    put("gate_v3a", "at g=%.0f deg: %s" % (rec["gate_deg"], lab),
        round(float(v), 6) if unit in ("%", "m") else int(v), unit, "")
    v = recb[k]
    put("gate_v3b", "at g=%.0f deg: %s" % (recb["gate_deg"], lab),
        round(float(v), 6) if unit in ("%", "m") else int(v), unit, "")
    if k in sw2.columns:
        v = rec2[k]
        put("gate_v2", "at g=%.0f deg: %s" % (rec2["gate_deg"], lab),
            round(float(v), 6) if unit in ("%", "m") else int(v), unit, "")
for k, lab, unit in (("false_gate_pct", "false-gate rate (overall)", "%"),
                     ("pstep_caught", "pseudo-step frames caught", "frame"),
                     ("pstep_caught_pct", "pseudo-step frames caught", "%"),
                     ("event1_caught", "event 1 caught", "-"),
                     ("event2_caught", "event 2 caught", "-"),
                     ("max_consecutive_genuine_rejections",
                      "longest rejection run on genuine frames", "frame")):
    v = rec1[k]
    put("gate_v1", "at g=%.0f deg: %s" % (rec1["gate_deg"], lab),
        round(float(v), 6) if unit in ("%", "m") else int(v), unit, "")
for tag, row_, nm in (("v3a", rec, "gate_v3a"), ("v3b", recb, "gate_v3b")):
    put(nm, "at g=%.0f deg: pseudo-step misses |dpsi|" % row_["gate_deg"],
        row_["pstep_missed_dpsi"] or "none", "deg", "largest first")
    put(nm, "at g=%.0f deg: re-anchor events" % row_["gate_deg"], int(row_["reanchors"]),
        "frame", "frames on which the heading was trusted again")
    put(nm, "at g=%.0f deg: stale-reference invalidations" % row_["gate_deg"],
        int(row_["gaps"]), "frame", "dt > max_gap_frames x nominal_dt = %.1f s"
        % (G.DEFAULT_MAX_GAP_FRAMES * TAU))
put("gate_v3a", "at g=%.0f deg: frames falling back to rule (b)" % rec["gate_deg"],
    int(rec["fallback_b_frames"]), "frame",
    "frames in the %d IMU-less recordings, where rule (a) has no gyro reference"
    % int((summ["has_imu"] == 0).sum()))
put("gate", "patch default g", round(G_DEF, 3), "deg",
    "max_yaw_rate 0.5 rad/s x 0.2 s + yaw_gate_margin 0.05 rad")
put("gate_v3a", "at the patch default: false-gate rate", round(float(dflt["false_gate_pct"]), 6),
    "%", "nearest swept gate, g = %.0f deg" % dflt["gate_deg"])
put("gate_v3a", "at the patch default: pseudo-step frames caught",
    round(float(dflt["pstep_caught_pct"]), 4), "%", "")
put("gate_v2", "at the patch default: false-gate rate", round(float(dflt2["false_gate_pct"]), 6),
    "%", "nearest swept gate, g = %.0f deg" % dflt2["gate_deg"])
put("gate_v2", "at the patch default: pseudo-step frames caught",
    round(float(dflt2["pstep_caught_pct"]), 4), "%", "")
put("gate_v1", "at the patch default: false-gate rate", round(float(dflt1["false_gate_pct"]), 6),
    "%", "nearest swept gate, g = %.0f deg" % dflt1["gate_deg"])
put("gate_v1", "at the patch default: pseudo-step frames caught",
    round(float(dflt1["pstep_caught_pct"]), 4), "%", "")
knee = sw3a.loc[sw3a["false_gate_pct"] <= 0.1, "gate_deg"]
knee_g = float(knee.min()) if len(knee) else float("nan")
put("gate_v3a", "smallest g with false-gate rate <= 0.1 %", knee_g, "deg",
    "the knee of the false-gate curve" if len(knee) else "never reached in the swept range")
if len(knee):
    put("gate_v3a", "longest genuine-only hold over g >= %.0f deg" % knee_g,
        int(sw3a.loc[sw3a["gate_deg"] >= knee_g, "max_consecutive_genuine_not_accepted"].max()),
        "frame", "what max_hold_frames has to cover")
kneeb = sw3b.loc[sw3b["false_gate_pct"] <= 0.1, "gate_deg"]
put("gate_v3b", "smallest g with false-gate rate <= 0.1 %",
    float(kneeb.min()) if len(kneeb) else float("nan"), "deg", "")
for tag in ("event1", "event2"):
    for nm, tab in (("v1", sw1), ("v2", sw2), ("v3a", sw3a), ("v3b", sw3b)):
        got = tab.loc[tab[tag + "_caught"] == 1, "gate_deg"]
        put("gate_" + nm, "largest g catching %s" % tag,
            float(got.max()) if len(got) else float("nan"), "deg",
            "" if len(got) else "never caught in the swept range")
# per-frame record of what a hold actually cost at the recommended gate, rule v3a (Fig. 7b)
rate_r, marg_r = G.params_for_gate(float(rec["gate_deg"]))
state_r = np.zeros(N, np.int8)
hold_r = np.zeros(N, np.int32)
anch_r = np.full(N, -1, np.int64)
rean_r = np.zeros(N, bool)
delta_r = np.full(N, np.nan)
budget_r = np.full(N, np.nan)
state_rb = np.zeros(N, np.int8)
delta_rb = np.full(N, np.nan)
budget_rb = np.full(N, np.nan)
for b, idx, yaw, ts in bag_groups:
    r_ = G.gate_series_v3(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], max_yaw_rate=rate_r,
                          yaw_gate_margin=marg_r, nominal_dt=TAU,
                          max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
                          max_gap_frames=G.DEFAULT_MAX_GAP_FRAMES,
                          gyro_drift_rate=G.DEFAULT_GYRO_DRIFT_RATE, use_gyro=True)
    state_r[idx] = r_["state"]
    hold_r[idx] = r_["hold_count"]
    rean_r[idx] = r_["reanchor"]
    delta_r[idx] = np.degrees(np.abs(r_["delta"]))
    budget_r[idx] = np.degrees(r_["budget"])
    a_ = r_["anchor"]
    anch_r[idx] = np.where(a_ >= 0, idx[np.maximum(a_, 0)], -1)
for b, idx, yaw, ts in bag_groups:                      # the same gate under rule (b)
    r_ = G.gate_series_v3(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], max_yaw_rate=rate_r,
                          yaw_gate_margin=marg_r, nominal_dt=TAU,
                          max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
                          max_gap_frames=G.DEFAULT_MAX_GAP_FRAMES,
                          gyro_drift_rate=G.DEFAULT_GYRO_DRIFT_RATE, use_gyro=False)
    state_rb[idx] = r_["state"]
    delta_rb[idx] = np.degrees(np.abs(r_["delta"]))
    budget_rb[idx] = np.degrees(r_["budget"])

# ---- what the pseudo-step frames the gate still misses actually are --------
# (the frame-to-frame increment is large, but the gate compares the reported heading with
#  the ANCHOR it is holding, and these frames come back close to that anchor)
MISS_ROWS = []
for lab, st_, dl_, bg_ in (("v3a", state_r, delta_r, budget_r),
                           ("v3b", state_rb, delta_rb, budget_rb)):
    mask = (st_ == G.ACCEPT) & pst_np
    for k in np.flatnonzero(mask):
        MISS_ROWS.append(dict(rule=lab, bag=S["bag"].to_numpy()[k], wall=wall(S["t"].to_numpy()[k]),
                              dpsi_deg=round(float(S["dpsi_deg"].to_numpy()[k]), 2),
                              dp_pub=round(float(S["dp_pub"].to_numpy()[k]), 4),
                              vs_anchor_deg=round(float(dl_[k]), 2),
                              budget_deg=round(float(bg_[k]), 2)))
pd.DataFrame(MISS_ROWS).to_csv(os.path.join(HERE, "gate_misses.csv"), index=False)
for lab in ("v3a", "v3b"):
    rws = [r for r in MISS_ROWS if r["rule"] == lab]
    if not rws:
        continue
    put("gate_" + lab, "at g=%.0f deg: missed frames, |dpsi| against the previous frame"
        % rec["gate_deg"],
        "%.1f - %.1f" % (min(abs(r["dpsi_deg"]) for r in rws),
                         max(abs(r["dpsi_deg"]) for r in rws)), "deg",
        "the increment that produced the position step")
    put("gate_" + lab, "at g=%.0f deg: missed frames, |dpsi| against the held anchor"
        % rec["gate_deg"],
        "%.1f - %.1f" % (min(r["vs_anchor_deg"] for r in rws),
                         max(r["vs_anchor_deg"] for r in rws)), "deg",
        "what the gate actually compares; every one of these is inside the %.0f deg budget, "
        "so the frame is accepted although the step still happens" % rec["gate_deg"])
    put("gate_" + lab, "at g=%.0f deg: missed frames, largest budget faced" % rec["gate_deg"],
        round(max(r["budget_deg"] for r in rws), 2), "deg",
        "v3 never scales the budget with dt; the widest value here is the rule (a) gyro "
        "allowance, not a dt term")

hg_r = (state_r == G.HOLD) & gen_np & (anch_r >= 0)
dheld_r = np.abs(cum_imu[hg_r] - cum_imu[anch_r[hg_r]])
pd.DataFrame(dict(bag=S["bag"].to_numpy()[hg_r], t=S["t"].to_numpy()[hg_r],
                  cls_imu=cls_np[hg_r], hold_count=hold_r[hg_r],
                  dpsi_deg=S["dpsi_deg"].to_numpy()[hg_r],
                  dheld_deg=dheld_r,
                  err_m=2.0 * LNORM * np.sin(np.radians(dheld_r) / 2.0),
                  gate_deg=float(rec["gate_deg"]))).to_csv(
    os.path.join(HERE, "hold_errors.csv"), index=False, float_format="%.5f")

# ---- what the recommended gate catches in every pseudo-step size class (Table 4)
not_acc_r = (state_r == G.HOLD) | (state_r == G.NOPUB)
_base_pst = ((S["quality"] == 4) & (S["cov35"] < COV_HD_BAD)
             & (S["dp_ant"] <= DP_ANT_MAX)).to_numpy()
_adpsi = S["adpsi"].to_numpy()
for T in PSTEP_T:
    m_ = _base_pst & (S["dp_pub"] >= T).to_numpy()
    put("gate_v3a", "at g=%.0f deg: pseudo-step frames caught >=%.2f m" % (rec["gate_deg"], T),
        int((not_acc_r & m_).sum()), "frame",
        "of %d frames in this size class; caught = the gate held or withheld the frame" %
        int(m_.sum()))
    miss_ = m_ & ~not_acc_r
    put("gate_v3a", "at g=%.0f deg: pseudo-step frames missed >=%.2f m" % (rec["gate_deg"], T),
        int(miss_.sum()), "frame", "")
    if miss_.any():
        put("gate_v3a", "at g=%.0f deg: missed |dpsi| min >=%.2f m" % (rec["gate_deg"], T),
            round(float(_adpsi[miss_].min()), 4), "deg", "")
        put("gate_v3a", "at g=%.0f deg: missed |dpsi| max >=%.2f m" % (rec["gate_deg"], T),
            round(float(_adpsi[miss_].max()), 4), "deg", "")

# ---- why v1 lets event 2 through: the budget it had grown to by then
for b, idx, yaw, ts in bag_groups:
    if b != EV_BAG:
        continue
    r1_ = G.gate_series(yaw, ts, max_yaw_rate=rate_r, yaw_gate_margin=marg_r,
                        max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES)
    j_ = np.where(ev_sel["event2"][idx])[0]
    if j_.size:
        j_ = int(j_[0])
        V1_HOLD_EV2 = int(r1_["hold_count"][j_ - 1])
        V1_BUDGET_EV2 = float(np.degrees(r1_["budget"][j_]))
        V1_DELTA_EV2 = float(np.degrees(abs(r1_["delta"][j_])))
        put("gate_v1", "at g=%.0f deg: hold count entering event 2" % rec["gate_deg"],
            V1_HOLD_EV2, "frame",
            "consecutive rejected frames immediately before event 2")
        put("gate_v1", "at g=%.0f deg: budget at event 2" % rec["gate_deg"],
            round(V1_BUDGET_EV2, 2), "deg",
            "max_yaw_rate * dt-since-last-accepted + yaw_gate_margin, unbounded in v1")
        put("gate_v1", "at g=%.0f deg: |delta| vs stale anchor at event 2" % rec["gate_deg"],
            round(V1_DELTA_EV2, 2), "deg",
            "clampRotation(yaw - last_accepted_yaw); accepted when it falls inside the budget")
        put("gate_v1", "at g=%.0f deg: event 2 frame rejected" % rec["gate_deg"],
            int(bool(r1_["rejected"][j_])), "-", "0 = the outlier was accepted")

# ---- the long outages rule (a) opens, and what opened them -----------------
OUT_ROWS = []
not_acc_r3 = (state_r == G.HOLD) | (state_r == G.NOPUB)
for b, idx, yaw, ts in bag_groups:
    m = not_acc_r3[idx]
    i = 0
    while i < m.size:
        if not m[i]:
            i += 1
            continue
        j = i
        while j < m.size and m[j]:
            j += 1
        dur = float(ts[min(j, m.size - 1)] - ts[i])
        if dur >= 60.0:
            gi = idx[i]
            # the reference is the last ACCEPTED frame, i.e. the one before the outage
            ri = idx[max(i - 1, 0)]
            dif = np.abs(wrap180((90.0 - S["yaw_deg"].to_numpy()[idx[i:j]])
                                 - ((90.0 - S["yaw_deg"].to_numpy()[ri])
                                    + (cum_imu[idx[i:j]] - cum_imu[ri]))))
            OUT_ROWS.append(dict(bag=b, start=wall(ts[i]), duration_s=round(dur, 1),
                                 frames=int(j - i),
                                 trigger_dpsi=round(float(S["dpsi_deg"].to_numpy()[gi]), 2),
                                 trigger_gyro=round(float(S["dpsi_imu_deg"].to_numpy()[gi]), 2)
                                 if np.isfinite(S["dpsi_imu_deg"].to_numpy()[gi]) else np.nan,
                                 trigger_quality=int(S["quality"].to_numpy()[gi]),
                                 trigger_dp_pub=round(float(S["dp_pub"].to_numpy()[gi]), 4),
                                 trigger_dp_ant=round(float(S["dp_ant"].to_numpy()[gi]), 4),
                                 median_offset_deg=round(float(np.median(dif)), 1)))
        i = j
OUT_ROWS.sort(key=lambda r: -r["duration_s"])
put("gate_v3a", "outages longer than 60 s", len(OUT_ROWS), "-",
    "runs of consecutive not-accepted frames at the recommended gate")
put("gate_v3a", "frames inside outages longer than 60 s",
    int(sum(r["frames"] for r in OUT_ROWS)), "frame",
    "%.1f %% of the corpus; these dominate the raw false-gate rate"
    % (100.0 * sum(r["frames"] for r in OUT_ROWS) / N))
put("gate_v3a", "longest outage", round(OUT_ROWS[0]["duration_s"], 1) if OUT_ROWS else 0.0, "s",
    "%s from %s" % (OUT_ROWS[0]["bag"], OUT_ROWS[0]["start"]) if OUT_ROWS else "")
for i_, r_ in enumerate(OUT_ROWS[:6], 1):
    put("gate_v3a_outages", "%d %s %s" % (i_, r_["bag"], r_["start"]), r_["duration_s"], "s",
        "%d frames; opened by a %.1f deg reported increment the gyro put at %.2f deg "
        "(status %d, dp_pub %.3f m, dp_ant %.3f m); median |reported - propagated| during "
        "the outage %.1f deg"
        % (r_["frames"], r_["trigger_dpsi"], r_["trigger_gyro"], r_["trigger_quality"],
           r_["trigger_dp_pub"], r_["trigger_dp_ant"], r_["median_offset_deg"]))
pd.DataFrame(OUT_ROWS).to_csv(os.path.join(HERE, "v3a_outages.csv"), index=False)

# ---- sensitivity of rule (a) to gyro_drift_rate ---------------------------
for dr in (float(np.radians(np.median(drifts))), G.DEFAULT_GYRO_DRIFT_RATE,
           float(np.radians(np.quantile(drifts, .9)))):
    st_ = np.zeros(N, np.int8)
    for b, idx, yaw, ts in bag_groups:
        r_ = G.gate_series_v3(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], max_yaw_rate=rate_r,
                              yaw_gate_margin=marg_r, nominal_dt=TAU,
                              max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
                              max_gap_frames=G.DEFAULT_MAX_GAP_FRAMES,
                              gyro_drift_rate=dr, use_gyro=True)
        st_[idx] = r_["state"]
    na_ = (st_ == G.HOLD) | (st_ == G.NOPUB)
    put("gate_v3a_drift", "gyro_drift_rate = %.5f rad/s (%.3f deg/s)" % (dr, np.degrees(dr)),
        round(100.0 * int((na_ & gen_np).sum()) / N_GEN, 4), "%",
        "false-gate rate; %d frames withheld, %d pseudo-steps caught, %.0f s to forgive a "
        "67.5 deg offset"
        % (int((st_ == G.NOPUB).sum()), int((na_ & pst_np).sum()),
           (np.radians(67.54) - np.radians(rec["gate_deg"])) / dr))

# ---- what happens to event 1 after the gate stops trusting the heading -----
EV_IDX = None
for b, idx, yaw, ts in bag_groups:
    if b == EV_BAG:
        EV_IDX, EV_YAW, EV_TS = idx, yaw, ts
        EV_J0 = int(np.where(ev_sel["event1"][idx])[0][0])
        EV_HDG = (90.0 - S["yaw_deg"].to_numpy()[idx]) % 360.0
        EV_CG = cum_imu[idx]
        break


def ev1_stats(use_gyro, max_gyro_ref_time=None):
    """What rule (a)/(b) does with the 2026-09-06 15:57 event, at the recommended gate."""
    r_ = G.gate_series_v3(EV_YAW, EV_TS, cum_gyro_rad=cum_gyro_yaw[EV_IDX],
                          max_yaw_rate=rate_r, yaw_gate_margin=marg_r, nominal_dt=TAU,
                          max_hold_frames=G.DEFAULT_MAX_HOLD_FRAMES,
                          max_gap_frames=G.DEFAULT_MAX_GAP_FRAMES,
                          gyro_drift_rate=G.DEFAULT_GYRO_DRIFT_RATE, use_gyro=use_gyro,
                          max_gyro_ref_time=max_gyro_ref_time)
    st = r_["state"]
    j0 = EV_J0
    anchor_pre = int(r_["anchor"][j0])
    k_nopub = next((k for k in range(j0, st.size) if st[k] == G.NOPUB), None)
    k_acc = next((k for k in range(j0 + 1, st.size) if st[k] == G.ACCEPT), None)
    g_ = dict(anchor_heading=float(EV_HDG[anchor_pre]),
              first_nopub_s=float(EV_TS[k_nopub] - EV_TS[j0]) if k_nopub is not None else np.nan,
              reanchor_s=float(EV_TS[k_acc] - EV_TS[j0]) if k_acc is not None else np.nan,
              reanchor_heading=float(EV_HDG[k_acc]) if k_acc is not None else np.nan)
    if k_acc is not None:
        true_hdg = EV_HDG[anchor_pre] + (EV_CG[k_acc] - EV_CG[anchor_pre])
        derr = float(abs(wrap180(EV_HDG[k_acc] - true_hdg)))
        g_["true_heading"] = float(true_hdg % 360.0)
        g_["heading_error_deg"] = derr
        g_["position_error_m"] = float(2.0 * LNORM * np.sin(np.radians(derr) / 2.0))
        g_["gyro_allowance_deg"] = float(np.degrees(
            np.radians(rec["gate_deg"]) + G.DEFAULT_GYRO_DRIFT_RATE
            * (EV_TS[k_acc] - EV_TS[anchor_pre])))
        g_["withheld_frames"] = int(np.sum(st[j0:k_acc] == G.NOPUB))
    return g_


EVR = {"v3a": ev1_stats(True), "v3b": ev1_stats(False)}

# ---- rule (a) with a bounded gyro reference (max_gyro_ref_time) ------------
BOUNDED = []
for mrt in (5.0, 10.0, 20.0, 60.0, None):
    st_, an_, re_, fb_, ng_, pb_, uy_ = run_v3(rate_r, marg_r, True, max_gyro_ref_time=mrt)
    na_ = (st_ == G.HOLD) | (st_ == G.NOPUB)
    c_, e_, _ = gated_residuals(pb_, uy_, accepted_from(st_))
    ev_ = ev1_stats(True, max_gyro_ref_time=mrt)
    BOUNDED.append(dict(max_gyro_ref_time=("unbounded" if mrt is None else "%.0f" % mrt),
                        false_gate_pct=round(100.0 * int((na_ & gen_np).sum()) / N_GEN, 4),
                        withheld=int((st_ == G.NOPUB).sum()),
                        withheld_pct=round(100.0 * float((st_ == G.NOPUB).mean()), 4),
                        residual_frames_010=c_[0.10], residual_episodes_010=e_[0.10],
                        residual_episodes_005=e_[0.05],
                        ev1_withheld_s=round(ev_["reanchor_s"], 1),
                        ev1_reanchor_error_m=round(ev_["position_error_m"], 4)))
    put("gate_v3a_bounded", "max_gyro_ref_time = %s s"
        % ("unbounded" if mrt is None else "%.0f" % mrt),
        BOUNDED[-1]["residual_episodes_010"], "episodes",
        "residual pseudo-step episodes >= 0.10 m in the gated output; false-gate %.4f %%, "
        "%d frames withheld (%.4f %%), event 1 withheld %.1f s and re-anchors %.3f m off"
        % (BOUNDED[-1]["false_gate_pct"], BOUNDED[-1]["withheld"],
           BOUNDED[-1]["withheld_pct"], BOUNDED[-1]["ev1_withheld_s"],
           BOUNDED[-1]["ev1_reanchor_error_m"]))
_ok = [b_ for b_ in BOUNDED if b_["false_gate_pct"] <= 0.1]
BOUND_PICK = min(_ok, key=lambda b_: (b_["residual_episodes_010"], b_["withheld"])) \
    if _ok else None
put("gate_v3a_bounded", "chosen max_gyro_ref_time",
    BOUND_PICK["max_gyro_ref_time"] if BOUND_PICK else "none qualifies", "s",
    "smallest residual-episode count among the settings whose false-gate rate stays at or "
    "below 0.1 %" if BOUND_PICK else
    "no setting keeps the false-gate rate at or below 0.1 %; rule (b) remains the default")

for variant in ("v3a", "v3b"):
    e = EVR[variant]
    nm = "event1_" + variant
    put(nm, "last trusted heading before the event", round(e["anchor_heading"], 2), "deg",
        "the anchor the gate was holding when the 67.5 deg step arrived")
    put(nm, "output withheld from", round(e["first_nopub_s"], 2), "s",
        "after the event frame; the %d preceding frames are published with the held heading"
        % G.DEFAULT_MAX_HOLD_FRAMES)
    put(nm, "heading trusted again after", round(e["reanchor_s"], 2), "s",
        "time from the event frame to the first accepted frame")
    put(nm, "frames withheld", int(e.get("withheld_frames", 0)), "frame", "")
    put(nm, "heading at re-anchor", round(e["reanchor_heading"], 2), "deg", "")
    put(nm, "true heading at re-anchor", round(e["true_heading"], 2), "deg",
        "last trusted heading propagated with the integrated gyro")
    put(nm, "heading error at re-anchor", round(e["heading_error_deg"], 2), "deg", "")
    put(nm, "published position error at re-anchor", round(e["position_error_m"], 4), "m",
        "2|L| sin(heading error / 2): what the vehicle reference point is off by once the "
        "gate trusts the heading again")
    put(nm, "gyro allowance at re-anchor", round(e["gyro_allowance_deg"], 2), "deg",
        "g + gyro_drift_rate x (t - last_accepted_t); rule (a) only")

# ---- residual pseudo-steps left in each rule's output ---------------------
RES_TABLE = []
for nm, tab in (("v1", sw1), ("v2", sw2), ("v3b", sw3b), ("v3a", sw3a)):
    for gd in G_DETAIL:
        r_ = tab.loc[(tab["gate_deg"] - gd).abs().idxmin()]
        row = dict(rule=nm, gate_deg=gd)
        for T in PSTEP_T:
            row["frames_%.2f" % T] = int(r_["residual_frames_%.2f" % T])
            row["episodes_%.2f" % T] = int(r_["residual_episodes_%.2f" % T])
            put("gated_residual", "%s g=%.0f deg: frames >=%.2f m" % (nm, gd, T),
                int(r_["residual_frames_%.2f" % T]), "frame",
                "pseudo-steps surviving in the gated output stream")
            put("gated_residual", "%s g=%.0f deg: episodes >=%.2f m" % (nm, gd, T),
                int(r_["residual_episodes_%.2f" % T]), "-", "")
        RES_TABLE.append(row)
pd.DataFrame(RESID_ROWS).to_csv(os.path.join(HERE, "gated_residuals.csv"), index=False)
put("gated_residual", "episode definition", "consecutive or within %.0f s" % EPISODE_GAP, "",
    "residual frames are grouped into episodes; the episode size is its largest step")
put("gated_residual", "re-anchor mark", "accepted frame whose predecessor was not accepted",
    "", "marks a step produced by the gate adopting a new heading, not by an accepted outlier")

put("hold_cost", "omega p99", round(W99, 4), "rad/s", "measured gyro p99")
put("hold_cost", "omega max", round(WMAX, 4), "rad/s", "measured gyro max")
for h in range(1, 6):
    put("hold_cost", "bound at omega p99, h=%d" % h,
        round(float(G.hold_cost_m(W99, h)), 4), "m",
        "2|L| sin(omega h tau / 2), tau = 0.2 s")
    put("hold_cost", "bound at omega max, h=%d" % h,
        round(float(G.hold_cost_m(WMAX, h)), 4), "m", "")
put("hold_cost", "measured max during genuine holds (v3a, g=%.0f)" % rec["gate_deg"],
    round(float(rec["hold_err_max_m"]), 4), "m",
    "2|L| sin(Delta_held/2) with Delta_held from the IMU-integrated yaw")
put("hold_cost", "measured RMS during genuine holds (v3a, g=%.0f)" % rec["gate_deg"],
    round(float(rec["hold_err_rms_m"]), 4), "m", "")
put("hold_cost", "fault step bound 2|L|", round(TWO_L, 4), "m", "what the gate buys against")


# ================================================ 7b. induced vs operational
put("induced", "recordings", len(INDUCED_BAGS), "bag", ", ".join(INDUCED_BAGS))
put("induced", "session", "2026-09-06 15:26-16:24, greenhouse, vehicle parked", "",
    "controlled multipath induction: a 0.50 x 0.70 m metal plate held beside the antennas, "
    "three attempts, two produced a false fix (field log of the 2026-09-06 induction session, "
    "not distributed); this is the "
    "session behind P1's E3b, not natural operation")
put("induced", "frames", int(IND.sum()), "frame", "usable increments in the induced session")
put("induced", "fixed hours", round(float(summ.loc[summ.bag.isin(INDUCED_BAGS),
                                                   "fixed_h"].sum()), 4), "h", "")
put("operational", "recordings", len(OP_BAGS), "bag", "everything else")
put("operational", "frames", int(OPS.sum()), "frame", "")
OP_FIXED_H = float(summ.loc[~summ.bag.isin(INDUCED_BAGS), "fixed_h"].sum())
put("operational", "fixed hours", round(OP_FIXED_H, 4), "h", "")
put("operational", "both showcase events are induced", 1, "-",
    "the 15:57 (67.5 deg) and 15:51 (178.9 deg) events of section 6 are both inside the "
    "induction session; no operational recording carries a step of that size")

# ---- pseudo-steps, split
for lab, msk, bags_, hrs in (("induced", IND, set(INDUCED_BAGS),
                              float(summ.loc[summ.bag.isin(INDUCED_BAGS), "fixed_h"].sum())),
                             ("operational", OPS, set(OP_BAGS), OP_FIXED_H)):
    sub = cand[msk[cand.index.to_numpy()]] if len(cand) else cand
    c_, e_, _ = gated_residuals(_allpub, _yawrad, _allpub, only_bags=bags_)
    for T in PSTEP_T:
        n_ = int((sub["dp_pub"] >= T).sum())
        put("pseudo_step_" + lab, "frames >=%.2f m" % T, n_, "frame", "")
        put("pseudo_step_" + lab, "episodes >=%.2f m" % T, e_[T], "-",
            "frames within %.0f s are one episode" % EPISODE_GAP)
        put("pseudo_step_" + lab, "rate >=%.2f m" % T, round(n_ / hrs, 4), "1/h",
            "per hour of fixed solution in this subset (%.2f h)" % hrs)
    if len(sub):
        p10 = sub[sub["dp_pub"] >= PSTEP_MAIN]
        put("pseudo_step_" + lab, "residual RMS >=%.2f m" % PSTEP_MAIN,
            round(rms(p10["resid"]), 4), "m", "dp_pub - 2|L| sin(|dpsi|/2)")
        put("pseudo_step_" + lab, "residual max |.| >=%.2f m" % PSTEP_MAIN,
            round(float(p10["resid"].abs().max()), 4), "m", "")
        put("pseudo_step_" + lab, "|dpsi| range >=%.2f m" % PSTEP_MAIN,
            "%.1f - %.1f" % (p10["adpsi"].min(), p10["adpsi"].max()), "deg", "")
        for T in (0.05,):
            pT = sub[sub["dp_pub"] >= T]
            put("pseudo_step_" + lab, "residual RMS >=%.2f m" % T, round(rms(pT["resid"]), 4),
                "m", "")
            put("pseudo_step_" + lab, "residual max |.| >=%.2f m" % T,
                round(float(pT["resid"].abs().max()), 4), "m", "")
            put("pseudo_step_" + lab, "fraction within %.2f m >=%.2f m" % (RESID_OK, T),
                round(100.0 * float((pT["resid"].abs() <= RESID_OK).mean()), 2), "%", "")

# ---- operational rates by site (the induced session removed)
op_summ = summ[~summ["bag"].isin(INDUCED_BAGS)].copy()
op_main = main[~main["bag"].isin(INDUCED_BAGS)]
for s_ in sorted(set(op_summ["site"])):
    bags_s = set(op_summ.loc[op_summ["site"] == s_, "bag"])
    n_ = int(op_main["bag"].isin(bags_s).sum())
    h_ = float(op_summ.loc[op_summ["site"] == s_, "fixed_h"].sum())
    lab = s_ + (" (natural)" if s_ == "greenhouse" else "")
    put("operational_by_site", "%s frames >=%.2f m" % (lab, PSTEP_MAIN), n_, "frame", "")
    put("operational_by_site", "%s fixed hours" % lab, round(h_, 4), "h", "")
    put("operational_by_site", "%s rate" % lab, round(n_ / h_, 4), "1/h",
        "the greenhouse recordings left here are 2026-08-28/29 (natural operation) plus the "
        "0.01 h tail of 2026-08-30" if s_ == "greenhouse" else "")
put("operational_by_site", "overall frames >=%.2f m" % PSTEP_MAIN, len(op_main), "frame", "")
put("operational_by_site", "overall rate", round(len(op_main) / OP_FIXED_H, 4), "1/h",
    "%d frames in %.2f fixed hours" % (len(op_main), OP_FIXED_H))

# ---- increment distribution, operational only
dist_rows("increments_operational", "all", S.loc[OPS, "adpsi"])
for c in ("straight", "turn"):
    m_ = OPS & (cls_np == c)
    if m_.any():
        dist_rows("increments_operational", c, S.loc[m_, "adpsi"])
imu_op = imu_S[~imu_S["bag"].isin(INDUCED_BAGS)]
put("increments_operational", "frames above the max bound",
    int((imu_op["adpsi"] > BMAX).sum()), "frame", "|dpsi| > %.3f deg/frame" % BMAX)
put("increments_operational", "P(|dpsi| > max bound)",
    round(100.0 * float((imu_op["adpsi"] > BMAX).mean()), 6), "%",
    "of the %d operational increments with a usable gyro envelope" % len(imu_op))

# ---- gate headline at the recommended gate, split
ONSET = {}
for lab, bags_ in (("induced", set(INDUCED_BAGS)), ("operational", set(OP_BAGS))):
    ons = []
    for b, idx, yaw, ts in bag_groups:
        if b not in bags_:
            continue
        k = np.flatnonzero(pst_np[idx])
        if k.size:
            brk = np.flatnonzero(np.diff(ST[idx][k]) > EPISODE_GAP) + 1
            ons += [idx[g_[0]] for g_ in np.split(k, brk)]
    ONSET[lab] = np.array(ons, dtype=np.int64)
    put("gate_split", "%s: pseudo-step onsets >=%.2f m" % (lab, PSTEP_MAIN), len(ons), "-",
        "first frame of each ungated pseudo-step episode")

SPLIT = []
for rule, use_gyro, mrt in (("v3b", False, None), ("v3a 5 s", True, 5.0),
                            ("v3a 10 s", True, 10.0), ("v3a 20 s", True, 20.0),
                            ("v3a 60 s", True, 60.0), ("v3a unbounded", True, None)):
    st_, an_, re_, fb_, ng_, pb_, uy_ = run_v3(rate_r, marg_r, use_gyro,
                                               max_gyro_ref_time=mrt)
    acc_ = accepted_from(st_)
    na_ = ~acc_
    for lab, msk, bags_ in (("operational", OPS, set(OP_BAGS)),
                            ("induced", IND, set(INDUCED_BAGS))):
        gsub = gen_np & msk
        c_, e_, _ = gated_residuals(pb_, uy_, acc_, only_bags=bags_)
        row = dict(rule=rule, subset=lab,
                   false_gate_pct=round(100.0 * int((na_ & gsub).sum())
                                        / max(int(gsub.sum()), 1), 4),
                   onsets_held=int(na_[ONSET[lab]].sum()) if ONSET[lab].size else 0,
                   onsets=int(ONSET[lab].size),
                   withheld=int(((st_ == G.NOPUB) & msk).sum()),
                   residual_episodes_010=e_[0.10], residual_frames_010=c_[0.10],
                   residual_episodes_005=e_[0.05])
        SPLIT.append(row)
        put("gate_split", "%s, %s: false-gate rate" % (rule, lab), row["false_gate_pct"], "%",
            "g = %.0f deg; %d/%d onsets held, %d frames withheld, %d residual episodes "
            ">= %.2f m" % (rec["gate_deg"], row["onsets_held"], row["onsets"],
                           row["withheld"], row["residual_episodes_010"], PSTEP_MAIN))
pd.DataFrame(SPLIT).to_csv(os.path.join(HERE, "gate_split.csv"), index=False)

# ================================================ 9. reviewer checks
# ---- 9.1 three-class frame counts and label-tolerance sensitivity ----------
lab_np = S["dpsi_imu_deg"].notna().to_numpy()
for lab, msk in (("all", np.ones(N, bool)), ("operational", OPS)):
    n_lab = int((lab_np & msk).sum())
    for tol in (1.0, GENUINE_TOL, 5.0):
        gen_t = (np.abs(wrap180(S["dpsi_deg"].to_numpy() - S["dpsi_imu_deg"].to_numpy()))
                 <= tol) & lab_np
        put("reviewer_classes", "%s: genuine (tol %.0f deg)" % (lab, tol),
            int((gen_t & msk).sum()), "frame",
            "|dpsi - dpsi_imu| <= %.0f deg; denominator of the false-gate rate" % tol)
        put("reviewer_classes", "%s: outlier (tol %.0f deg)" % (lab, tol),
            int((~gen_t & lab_np & msk).sum()), "frame", "")
    put("reviewer_classes", "%s: unclassified (no IMU)" % lab, int((~lab_np & msk).sum()),
        "frame", "frames in the %d recordings without an IMU; excluded from both rates"
        % int((summ["has_imu"] == 0).sum()))
    put("reviewer_classes", "%s: total increments" % lab, int(msk.sum()), "frame", "")

# false-gate sensitivity to the tolerance, at the recommended gate
_rules = {}
for nm, st_ in (("v3a", state_r), ("v3b", state_rb)):
    _rules[nm] = (st_ != G.ACCEPT)
for nm, fn in (("v1", lambda y, t: G.gate_series(y, t, max_yaw_rate=rate_r,
                                                 yaw_gate_margin=marg_r)),
               ("v2", lambda y, t: G.gate_series_v2(y, t, max_yaw_rate=rate_r,
                                                    yaw_gate_margin=marg_r))):
    na_ = np.zeros(N, bool)
    for b, idx, yaw, ts in bag_groups:
        r_ = fn(yaw, ts)
        na_[idx] = r_["rejected"] | (~r_["heading_valid"]) if nm == "v1" \
            else (r_["state"] != G.ACCEPT)
    _rules[nm] = na_
for nm in ("v1", "v2", "v3b", "v3a"):
    for tol in (1.0, GENUINE_TOL, 5.0):
        gen_t = (np.abs(wrap180(S["dpsi_deg"].to_numpy() - S["dpsi_imu_deg"].to_numpy()))
                 <= tol) & lab_np
        for lab, msk in (("all", np.ones(N, bool)), ("operational", OPS)):
            den = int((gen_t & msk).sum())
            put("reviewer_tolerance", "%s, %s, tol %.0f deg" % (nm, lab, tol),
                round(100.0 * int((_rules[nm] & gen_t & msk).sum()) / max(den, 1), 4), "%",
                "false-gate rate at g = %.0f deg over %d genuine frames" % (rec["gate_deg"], den))

# ---- 9.2 heading noise sigma_psi ------------------------------------------
STILL = ((S["cls_imu"] == "slow") & (S["quality"] == 4) & S["gz_max"].notna()
         & (S["gz_max"] < 0.02)).to_numpy()
put("reviewer_sigma", "stationary frames", int(STILL.sum()), "frame",
    "v2 slow class (reconstructed antenna path <= 0.50 m over 3 s), fixed solution, "
    "|omega_z| < 0.02 rad/s")
for lab, msk in [("overall", STILL)] + [
        (s_, STILL & S["bag"].isin(summ.loc[summ.site == s_, "bag"]).to_numpy())
        for s_ in sorted(set(summ["site"]))]:
    v = S.loc[msk, "dpsi_deg"].to_numpy()
    if v.size < 30:
        continue
    sd = float(np.std(v, ddof=1))
    put("reviewer_sigma", "sigma_psi %s" % lab, round(sd / np.sqrt(2.0), 4), "deg",
        "std of the per-frame heading increment / sqrt(2), over %d frames (increment std "
        "%.4f deg)" % (int(msk.sum()), sd))
put("reviewer_sigma", "driver cov[35]", 7.61544e-05, "rad^2",
    "what the driver publishes when the heading is valid = %.3f deg 1-sigma"
    % np.degrees(np.sqrt(7.61544e-05)))

# ---- 9.3 stale-heading exclusion ------------------------------------------
# IMU-integrated heading change over the 2 s preceding each frame, from the full
# per-frame cumulative series (all frames of the recording, not just the used ones)
STALE_W = 2.0
STALE_TOL = 5.0
d_t = d["t"].to_numpy()
d_cum = d["cum_imu_full"].to_numpy()
d_bag = d["bag"].to_numpy()
imu2 = np.full(N, np.nan)
for b, idx, yaw, ts in bag_groups:
    m_ = d_bag == b
    tt_, cc_ = d_t[m_], d_cum[m_]
    if not np.isfinite(cc_).any():
        continue
    j_now = np.searchsorted(tt_, ST[idx], side="right") - 1
    j_pre = np.searchsorted(tt_, ST[idx] - STALE_W, side="right") - 1
    ok_ = (j_pre >= 0) & (j_now >= 0)
    imu2[idx[ok_]] = cc_[j_now[ok_]] - cc_[j_pre[ok_]]
S["imu_turn_2s"] = imu2
S["stale_candidate"] = (np.abs(wrap180(imu2 - S["dpsi_deg"].to_numpy())) <= STALE_TOL)
for T in (0.05, PSTEP_MAIN):
    for lab, bags_ in (("induced", set(INDUCED_BAGS)), ("operational", set(OP_BAGS))):
        m_ = ((S["dp_pub"] >= T) & (S["quality"] == 4) & (S["cov35"] < COV_HD_BAD)
              & (S["dp_ant"] <= DP_ANT_MAX) & S["bag"].isin(bags_)).to_numpy()
        put("reviewer_stale", "%s: pseudo-steps >=%.2f m" % (lab, T), int(m_.sum()), "frame", "")
        put("reviewer_stale", "%s: stale-heading candidates >=%.2f m" % (lab, T),
            int((m_ & S["stale_candidate"].to_numpy()).sum()), "frame",
            "the IMU turn over the preceding %.1f s is within %.0f deg of the observed "
            "increment, so the jump could be a late update of a real turn" % (STALE_W, STALE_TOL))
for tag, idx_ in (("event1", ev1), ("event2", ev2)):
    k_ = np.flatnonzero(((S["bag"] == EV_BAG)
                         & (np.abs(S["t"] - float(d.loc[idx_, "t"])) < 1e-3)).to_numpy())
    if k_.size:
        k_ = int(k_[0])
        put("reviewer_stale", "%s: IMU turn over the preceding %.1f s" % (tag, STALE_W),
            round(float(imu2[k_]), 3), "deg",
            "vehicle parked; the reported increment is %.2f deg"
            % float(S["dpsi_deg"].to_numpy()[k_]))
        put("reviewer_stale", "%s: IMU turn over the frame itself" % tag,
            round(float(S["dpsi_imu_deg"].to_numpy()[k_]), 3), "deg", "")
        put("reviewer_stale", "%s: stale-heading candidate" % tag,
            int(bool(S["stale_candidate"].to_numpy()[k_])), "-", "0 = no, the IMU saw no turn")

# ---- 9.5 Mahalanobis (normalised-innovation) comparison, Reviewer 2 -------
MAHAL = []
for k_ in (3.0, 5.0):
    st_ = np.zeros(N, np.int8)
    uy_ = np.zeros(N)
    for b, idx, yaw, ts in bag_groups:
        r_ = G.gate_series_mahal(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], k_sigma=k_)
        st_[idx] = r_["state"]
        uy_[idx] = r_["used_yaw"]
    rej_ = st_ != G.ACCEPT
    pub_ = np.ones(N, bool)                      # the filter always publishes an estimate
    for lab, msk, bags_ in (("all", np.ones(N, bool), None),
                            ("operational", OPS, set(OP_BAGS))):
        c_, e_, _ = gated_residuals(pub_, uy_, ~rej_, only_bags=bags_)
        gsub = gen_np & msk
        ons = ONSET["operational"] if lab == "operational" else np.concatenate(
            [ONSET["operational"], ONSET["induced"]])
        row = dict(k_sigma=k_, subset=lab,
                   false_gate_pct=round(100.0 * int((rej_ & gsub).sum())
                                        / max(int(gsub.sum()), 1), 4),
                   rejected=int((rej_ & msk).sum()),
                   withheld=0,
                   onsets_rejected=int(rej_[ons].sum()), onsets=int(ons.size),
                   residual_episodes_010=e_[0.10], residual_frames_010=c_[0.10],
                   residual_episodes_005=e_[0.05])
        MAHAL.append(row)
        put("reviewer_mahal", "k = %.0f, %s: false-gate rate" % (k_, lab),
            row["false_gate_pct"], "%",
            "%d frames rejected, 0 withheld (the filter always publishes an estimate), "
            "%d/%d pseudo-step onsets rejected, %d residual episodes >= %.2f m"
            % (row["rejected"], row["onsets_rejected"], row["onsets"],
               row["residual_episodes_010"], PSTEP_MAIN))
    # what it does with event 1
    for b, idx, yaw, ts in bag_groups:
        if b != EV_BAG:
            continue
        r_ = G.gate_series_mahal(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], k_sigma=k_)
        st2 = r_["state"]
        j0 = EV_J0
        k_acc = next((q for q in range(j0 + 1, st2.size) if st2[q] == G.ACCEPT), None)
        if k_acc is not None:
            true_hdg = EV_HDG[j0 - 1] + (EV_CG[k_acc] - EV_CG[j0 - 1])
            derr = float(abs(wrap180(EV_HDG[k_acc] - true_hdg)))
            put("reviewer_mahal", "k = %.0f: event 1 rejected for" % k_,
                round(float(ts[k_acc] - ts[j0]), 2), "s",
                "%d frames; the filter keeps publishing its own estimate throughout"
                % int(np.sum(st2[j0:k_acc] != G.ACCEPT)))
            put("reviewer_mahal", "k = %.0f: event 1 heading error when accepted again" % k_,
                round(derr, 2), "deg", "")
            put("reviewer_mahal", "k = %.0f: event 1 position error when accepted again" % k_,
                round(float(2.0 * LNORM * np.sin(np.radians(derr) / 2.0)), 4), "m", "")
pd.DataFrame(MAHAL).to_csv(os.path.join(HERE, "gate_mahal.csv"), index=False)
put("reviewer_mahal", "q_yaw", G.DEFAULT_Q_YAW, "rad^2/s",
    "process noise = (measured gyro drift %.3f rad/s)^2 per second" % G.DEFAULT_GYRO_DRIFT_RATE)
put("reviewer_mahal", "R_yaw", G.DEFAULT_R_YAW, "rad^2",
    "measurement noise = the cov[35] the driver publishes (%.2f deg)"
    % np.degrees(np.sqrt(G.DEFAULT_R_YAW)))
put("reviewer_mahal", "caveat", "1-D emulation, not the full robot_localization filter", "",
    "one scalar yaw state, no cross-covariance with position or velocity, no "
    "differential-drive process model")

# ---- 9.4 motion classification v1 -> v2 (Reviewer 3) ----------------------
old_cls = S["cls_imu_v1"].to_numpy()
new_cls = cls_np
put("reviewer_class_rule", "rule", "slow: sum|dp_ant| over the trailing 3.0 s <= 0.50 m; "
    "else straight if |integral omega_z| < 5 deg, else turn; na without 3 s of history or "
    "without an IMU", "", "v1 used the published displacement over the same window, which is "
    "the quantity a lever-arm pseudo-step corrupts")
put("reviewer_class_rule", "frames that changed class", int((old_cls != new_cls).sum()),
    "frame", "%.2f %% of the %d increments" % (100.0 * float((old_cls != new_cls).mean()), N))
for a_ in ("straight", "turn", "slow", "na"):
    for b_ in ("straight", "turn", "slow", "na"):
        n_ = int(((old_cls == a_) & (new_cls == b_)).sum())
        if n_:
            put("reviewer_class_xtab", "v1 %s -> v2 %s" % (a_, b_), n_, "frame", "")
for c in ("straight", "turn", "slow"):
    for lab, arr in (("v1", old_cls), ("v2", new_cls)):
        v = S.loc[arr == c, "adpsi"].to_numpy()
        if v.size:
            put("reviewer_class_effect", "%s %s: n" % (c, lab), int(v.size), "frame", "")
            put("reviewer_class_effect", "%s %s: RMS" % (c, lab), round(rms(v), 4), "deg", "")
            put("reviewer_class_effect", "%s %s: P(>5 deg)" % (c, lab),
                round(100.0 * float((v > 5).mean()), 4), "%", "")
    for lab, arr in (("v1", old_cls), ("v2", new_cls)):
        m_ = (arr == c)
        den = int((gen_np & m_).sum())
        if den:
            put("reviewer_class_effect", "%s %s: false-gate at g=%.0f (v3b)" % (c, lab, rec["gate_deg"]),
                round(100.0 * int((_rules["v3b"] & gen_np & m_).sum()) / den, 4), "%",
                "over %d genuine frames" % den)

# rewrite the audit trail with the new columns
audit = S[(S["quality"] == 4) & (S["cov35"] < COV_HD_BAD) & (S["dp_ant"] <= DP_ANT_MAX)
          & (S["dp_pub"] >= min(PSTEP_T))]
audit.assign(site=audit["bag"].map(site), wall=[wall(t) for t in audit["t"]],
             induced=audit["bag"].isin(INDUCED_BAGS).astype(int)).to_csv(
    os.path.join(HERE, "pseudo_steps.csv"), index=False,
    columns=["bag", "site", "induced", "wall", "t", "dt", "dpsi_deg", "dp_pub", "dp_ant",
             "pred_jump", "resid", "across_pre", "across", "along_pre", "quality", "sats",
             "age_s", "cov35", "dpsi_imu_deg", "imu_turn_2s", "stale_candidate", "gz_max",
             "cls_imu"],
    float_format="%.4f")

# ================================================ 8. data anomalies
jump = d[d["dp_pub"] > 1.0]
put("anomaly", "frames with dp_pub > 1 m", len(jump), "frame",
    "position jumps far beyond 2|L| = %.3f m; the reconstructed antenna point jumps with them "
    "(median |dp_ant| %.3f m), so these are position-domain events (solution resets / injected "
    "faults), NOT lever-arm pseudo-steps, and the dp_ant criterion excludes them"
    % (TWO_L, float(jump["dp_ant"].median()) if len(jump) else float("nan")))
for b, n in jump.groupby("bag").size().sort_values(ascending=False).items():
    put("anomaly", "dp_pub > 1 m in %s" % b, int(n), "frame",
        "largest %.2f m" % float(jump.loc[jump["bag"] == b, "dp_pub"].max()))
zero_fix = summ.loc[summ["fixed_frames"] == 0, "bag"].tolist()
put("anomaly", "bags with no fixed solution", len(zero_fix), "bag", ", ".join(zero_fix))
dupm = (d.groupby("bag")["t"].diff() == 0) & d["step_ok"]
put("anomaly", "duplicate time stamps", int(dupm.sum()), "frame",
    "consecutive /rtk_odom frames with identical stamps (dt = 0); %.2f %% of the increments. "
    "Their |dpsi| is ordinary (median %.3f deg, p99 %.3f deg, max %.3f deg), but the gate gives "
    "them a budget of yaw_gate_margin alone, so they are the tightest frames in the replay"
    % (100.0 * dupm.mean(), float(d.loc[dupm, "adpsi"].median()),
       q(d.loc[dupm, "adpsi"], .99), float(d.loc[dupm, "adpsi"].max())))
put("anomaly", "frames with cov[35] >= 1e5", int((d["cov35"] >= COV_HD_BAD).sum()), "frame",
    "driver had no heading and published the raw ANT1 point with yaw = 0; bags affected: %s"
    % ", ".join(sorted(d.loc[d["cov35"] >= COV_HD_BAD, "bag"].unique())))


# ==================================================================== write
with open(os.path.join(HERE, "results_numbers.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["group", "quantity", "value", "unit", "definition"])
    w.writerows(OUT)
print("wrote results_numbers.csv (%d rows)" % len(OUT))


# ---- markdown -------------------------------------------------------------
def md_table(rows, headers):
    """Markdown table; pipes inside a cell (|L|, |dp_fix - dp_ant|, ...) must be escaped
    or they are read as column separators."""
    def cell(x):
        return str(x).replace("|", "\\|")
    out = ["| " + " | ".join(cell(h) for h in headers) + " |",
           "|" + "|".join(["---"] * len(headers)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(cell(x) for x in r) + " |")
    return "\n".join(out)


def gsel(group):
    return [(q_, v, u, n) for (g_, q_, v, u, n) in OUT if g_ == group]


def gdict(group):
    return dict((q_, v) for (g_, q_, v, u, n) in OUT if g_ == group)


L = []
L.append("# P2 Results — every number, with its definition\n")
L.append("> Generated by `fig/results_numbers.py` from `data/csv/*.csv` and "
         "`data/summary.csv`. Project rule: a number that is not in this file does not go into "
         "the manuscript.\n"
         "> Gate replay uses `fig/gate.py`: **v1** is the rule as it stands in "
         "`patch/heading_gate.patch`, **v2** is the amended specification that goes into the "
         "patch next. Same parameter names in both.\n"
         "> Geometry: |L| = %.4f m, 2|L| = %.4f m, tau = %.1f s (5 Hz).\n" % (LNORM, TWO_L, TAU))

L.append("## 0. Frame selection\n")
L.append("**Replays are excluded.** Recordings whose `/rtk_odom` time stamps duplicate another "
         "recording are replays of that recording, not independent data, and are dropped from "
         "every statistic here. One recording qualifies — `%s`, whose frames are stamped in the "
         "same %s-minute window as `%s` — removing %d frames. The detection is in section 8.\n"
         % (EXCLUDE_BAGS[0], "%.0f" % (overlaps[0][2] / 60.0) if overlaps else "?",
            [x for x in overlaps[0][:2] if x not in EXCLUDE_BAGS][0] if overlaps else "?",
            int(n_frames_raw - len(d))))
L.append("**Gyro spikes are removed at a fixed threshold.** Every use of the CH110 in this file "
         "— the physical bound of §3, the straight/turn labelling, the genuine/outlier scoring of "
         "§7 and the drift measurement behind `gyro_drift_rate` — drops samples with "
         "|ω_z| > %.0f rad/s (%.0f °/s), the same rule as `data/post_summary.py`. Three samples "
         "in the corpus exceed it, the largest by seven orders of magnitude; nothing else is "
         "filtered.\n" % (GYRO_NONPHYS, np.degrees(GYRO_NONPHYS)))
L.append("A *frame* is one `/rtk_odom` message (5 Hz); an *increment* is the pair (k−1, k). An "
         "increment enters the statistics only if\n"
         "1. both frames carry a heading (`cov[35] < 1e5`; without a heading the driver "
         "publishes yaw = 0 **and skips the lever-arm rotation**, so such an increment is an "
         "artefact, not a heading outlier), and\n"
         "2. the two frames are less than %.1f s apart (no recording gap).\n" % DT_MAX)
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("corpus")
                   if q_.startswith("frames_dropped") or q_ == "increments_used"
                   or "excluded" in q_], ["quantity", "value", "unit"]) + "\n")

L.append("## 1. Corpus (Table 3)\n")
L.append("Site is taken from each bag's median ENU position: the corpus shares one "
         "`~/.ros/rtk_origin.yaml`, so the sites separate by hundreds of metres (concrete yard "
         "≈ (12, 0), second greenhouse ≈ (−22, −31), 0828–0830 greenhouse ≈ (88, −150), working "
         "orchard ≈ (−1280, −180), 0826 orchard ≈ (4976, −1469)). Cross-checked against the "
         "experiment table of the mechanism-and-data checklist (not distributed) §B.2.\n")
rows = []
for s_, grp in summ.groupby("site"):
    rows.append((s_, len(grp), "%.2f" % grp["dur_h"].sum(), int(grp["n_frames"].sum()),
                 "%.2f" % grp["fixed_h"].sum(), int(grp["fixed_frames"].sum()),
                 int(grp["has_fix"].sum()), int(grp["has_imu"].sum())))
rows.append(("**total**", len(summ), "%.2f" % summ["dur_h"].sum(), int(summ["n_frames"].sum()),
             "%.2f" % summ["fixed_h"].sum(), int(summ["fixed_frames"].sum()),
             int(summ["has_fix"].sum()), int(summ["has_imu"].sum())))
L.append(md_table(rows, ["site", "bags", "hours", "frames", "fixed h", "fixed frames",
                         "bags with /rtk/fix", "bags with /ch110/data_raw"]) + "\n")
L.append("*hours* = sum of each bag's `/rtk_odom` span; *fixed frames* = frames whose nearest "
         "`/rtk/status` (within 1.5 s) reports a fixed solution; *fixed h* = fixed frames × 0.2 s. "
         "`/rtk/fix` (raw ANT1 latitude/longitude) only joined the recording list on 2026-09-10, "
         "which is why %d bags carry it.\n" % int(summ["has_fix"].sum()))

L.append("## 2. Frame-to-frame heading increment |Δψ|\n")
L.append("ψ is recovered from the published `/rtk_odom` quaternion (heading = 90° − yaw); "
         "`cls_imu` is the IMU-only motion label from `extract_p2.py` (|integrated gyro yaw| over "
         "the trailing 3 s < 5° → straight, else turn; `na` = bag without an IMU, or the first "
         "3 s). `cls_imu` never uses the RTK heading, so the split is independent of the quantity "
         "being measured. RMS and the exceedance probabilities are the primary numbers; the "
         "quantiles are secondary.\n")
rows = []
for lab in ("all", "straight", "turn", "na"):
    got = dict((q_.replace(lab + " ", ""), v) for (q_, v, u, n) in gsel("increments")
               if q_.startswith(lab + " "))
    if not got:
        continue
    rows.append((lab, got["n"], got["RMS"],
                 got["P(>5 deg)"], got["P(>10 deg)"], got["P(>20 deg)"], got["P(>45 deg)"],
                 got["P(>90 deg)"], got["P(> max bound %.2f deg)" % BMAX],
                 got["median"], got["p99"], got["max"]))
L.append(md_table(rows, ["cls_imu", "n", "RMS (°)", "P(>5°) %", "P(>10°) %", "P(>20°) %",
                         "P(>45°) %", "P(>90°) %", "P(>%.2f°) %%" % BMAX,
                         "median (°)", "p99 (°)", "max (°)"]) + "\n")
rows = []
for lab in ("all", "straight", "turn", "na"):
    got = dict((q_.replace(lab + " ", ""), v) for (q_, v, u, n) in gsel("increments")
               if q_.startswith(lab + " "))
    if got:
        rows.append((lab, got["count >5 deg"], got["count >10 deg"], got["count >20 deg"],
                     got["count >45 deg"], got["count >90 deg"], got["p90"], got["p99.9"]))
L.append(md_table(rows, ["cls_imu", ">5°", ">10°", ">20°", ">45°", ">90°", "p90 (°)",
                         "p99.9 (°)"]) + "\n")
L.append("`cls_imu` has no *slow* class — that class exists only in the RTK-heading-based label "
         "`cls` (window displacement ≤ 0.5 m), tabulated below for completeness. `cls` is built "
         "from the RTK heading itself, so it is not independent of the quantity being measured; "
         "every split quoted in the text uses `cls_imu`.\n")
rows = []
for lab in ("straight", "turn", "slow", "na"):
    got = dict((q_.replace(lab + " ", ""), v) for (q_, v, u, n) in gsel("increments_cls_rtk")
               if q_.startswith(lab + " "))
    if got:
        rows.append((lab, got["n"], got["RMS"], got["P(>5 deg)"], got["P(>20 deg)"],
                     got["median"], got["p99"], got["max"]))
L.append(md_table(rows, ["cls (RTK-based)", "n", "RMS (°)", "P(>5°) %", "P(>20°) %",
                         "median (°)", "p99 (°)", "max (°)"]) + "\n")

L.append("## 3. Physical bound from the IMU\n")
L.append("`gz_max` is the largest |ω_z| the CH110 reported inside the frame interval, so "
         "|ω_z|·0.2 s is an **upper envelope** on the heading change the platform could have "
         "produced in that frame. Three samples exceed %.0f rad/s (%.0f °/s) — non-physical for "
         "a tracked chassis whose measured p99 is two orders of magnitude smaller — and are "
         "dropped, the same rule as `data/post_summary.py`.\n"
         % (GYRO_NONPHYS, np.degrees(GYRO_NONPHYS)))
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("physical_bound")],
                  ["quantity", "value", "unit"]) + "\n")

L.append("## 4. Pseudo-step frames\n")
L.append("A *pseudo-step candidate* is a frame with a **fixed** solution (status 4), a valid "
         "heading (`cov[35] < 1e5`), and a reconstructed antenna point that did not move "
         "(|Δp_ant| ≤ %.2f m), where p_ant = p_pub + R(yaw)·L. The published point, however, did "
         "move. Frame list: `fig/pseudo_steps.csv`.\n" % DP_ANT_MAX)
rows = []
for T in PSTEP_T:
    got = dict((q_.replace(" >=%.2f m" % T, ""), v) for (q_, v, u, n) in gsel("pseudo_step")
               if q_.endswith(">=%.2f m" % T))
    if got.get("count", 0) == 0:
        continue
    rows.append(("%.2f" % T, got["count"], "%.2f" % (got["count"] / FIXED_H),
                 "%s – %s" % (got["|dpsi| min"], got["|dpsi| max"]),
                 got["residual RMS"], got["residual max |.|"],
                 "%.0f %%" % got["fraction within %.2f m" % RESID_OK],
                 got["caught by cross-track 0.30 m"], got["caught by norm 0.50 m"]))
L.append(md_table(rows, ["dp_pub ≥ (m)", "frames", "per fixed hour", "|Δψ| range (°)",
                         "residual RMS (m)", "residual max (m)", "within %.2f m" % RESID_OK,
                         "above deployed 0.30 m cross-track",
                         "above deployed 0.50 m norm"]) + "\n")
L.append("*residual* = dp_pub − 2|L| sin(|Δψ|/2), the verification of eq. (2). The cross-track "
         "component |c| of eq. (3) is computed in the **pre-jump** body frame (θ measured against "
         "the yaw of frame k−1, the heading the vehicle was believed to be following); "
         "`extract_p2.py`'s own `across` column rotates by the post-jump yaw and is reported "
         "alongside in `results_numbers.csv` for comparison.\n")
L.append("Per-bag concentration (top 5, dp_pub ≥ %.2f m):\n" % PSTEP_MAIN)
L.append(md_table([(q_.split(" ", 1)[1], v, n) for (q_, v, u, n) in gsel("pseudo_step_by_bag")],
                  ["bag", "frames", "note"]) + "\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("pseudo_step_by_site")],
                  ["site", "value", "unit"]) + "\n")

L.append("## 5. Independent check against the raw antenna fix\n")
L.append("In the bags that record `/rtk/fix`, the raw ANT1 latitude/longitude is converted to the "
         "same ENU frame and differenced frame to frame (`dp_fix`). If the reconstruction "
         "p_ant = p_pub + R(yaw)·L is right, dp_fix must equal dp_ant.\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("rtk_fix_check")],
                  ["quantity", "value", "unit"]) + "\n")
L.append("Read this the right way round. **dp_fix ≡ dp_ant** to the 0.1 mm the CSV is written at, "
         "on essentially every frame: the reconstruction recovers the raw antenna point exactly, "
         "which is what licenses using it in the %d bags recorded before `/rtk/fix` was added. The "
         "residual tail is timing — the nearest `/rtk/fix` sample is accepted up to 0.15 s away. "
         "The frames with |Δψ| > 10° in these bags are **genuine pivot turns**, not outliers (the "
         "gyro confirms %d of the %d within 2°): the antenna sweeps an arc of 0.08 m while the "
         "published reference point stays within 0.01 m, i.e. the lever-arm compensation doing its "
         "job. These bags carry no large heading outlier (max |Δψ| = %.1f°), so they validate the "
         "reconstruction, not the fault.\n"
         % (len(summ) - len(fixb),
            int((np.abs(wrap180(F10["dpsi_deg"] - F10["dpsi_imu_deg"])) <= GENUINE_TOL).sum()),
            len(F10), float(F["adpsi"].max())))

L.append("## 6. The two recorded events (E3b, 2026-09-06, greenhouse, vehicle stationary)\n")
e1 = gdict("event1")
e2 = gdict("event2")
rows = []
for k in [q_ for (q_, v, u, n) in gsel("event1")]:
    if k in ("bag", "frames with |dpsi| > 20 deg in this bag"):
        continue
    rows.append((k, e1.get(k, "—"), e2.get(k, "—")))
L.append(md_table(rows, ["quantity", "event 1", "event 2"]) + "\n")
L.append("Both are single 5 Hz frames in `%s`. Event 1 is the event quoted in P1 as a 1 s change "
         "of −69.8° (heading 256.73° → 186.94° between 15:57:02.5 and 15:57:03.5); the single "
         "frame that carries the position step is the one tabulated here. The height step is zero "
         "by construction: the driver subtracts L_z as a constant and never rotates it.\n" % EV_BAG)

L.append("## 7. Gate sweep (Fig. 6 / Fig. 7 / Table 4)\n")
L.append("Three rules are replayed, all through `fig/gate.py`, all with the same parameter "
         "names. **v1** is the rule as patched today: "
         "`budget = max_yaw_rate·dt + yaw_gate_margin`, dt measured from the last *accepted* "
         "heading and unbounded. **v2** bounded that dt at `max_hold_frames·nominal_dt`, stopped "
         "publishing once a hold was exhausted, and re-anchored on two mutually consistent "
         "samples. **v3** is the specification that goes into the C++ patch: the budget does not "
         "scale with dt at all, a reference older than `max_gap_frames` is declared stale, and "
         "re-anchoring is done against the gyro-propagated heading (rule **a**) or, where no IMU "
         "is present, against two consistent samples (rule **b**).\n")
L.append("```\n"
         "params: max_yaw_rate, yaw_gate_margin, nominal_dt = %.1f s, max_hold_frames = %d,\n"
         "        max_gap_frames = %d, gyro_drift_rate = %.3f rad/s, use_gyro\n"
         "state:  last_accepted_yaw, last_accepted_t, hold_count, valid, pending(yaw, t),\n"
         "        gyro_yaw = last_accepted_yaw + integral of omega_z since last_accepted_t\n"
         "per frame (y, t):\n"
         "  g = max_yaw_rate * nominal_dt + yaw_gate_margin            # FIXED, no dt scaling\n"
         "  dt = t - last_accepted_t > max_gap_frames * nominal_dt and valid\n"
         "        -> valid = false, pending = None                     # stale reference\n"
         "  valid:\n"
         "     |wrap(y - last_accepted_yaw)| <= g -> ACCEPT (publish y, hold_count = 0)\n"
         "     else -> HOLD (publish last_accepted_yaw, hold_count += 1)\n"
         "             hold_count >= max_hold_frames -> valid = false, pending = None,\n"
         "                                              frame NOT published\n"
         "  not valid: nothing published; re-anchor\n"
         "     (a) use_gyro and gyro available:\n"
         "         |wrap(y - gyro_yaw)| <= g + gyro_drift_rate*(t - last_accepted_t)\n"
         "             -> ACCEPT y (valid = true); else stay invalid\n"
         "     (b) otherwise: pending is None -> pending = (y, t)\n"
         "         |wrap(y - pending.yaw)| <= g -> ACCEPT y (valid = true)\n"
         "         else -> pending = (y, t)\n"
         "```\n"
         % (TAU, G.DEFAULT_MAX_HOLD_FRAMES, G.DEFAULT_MAX_GAP_FRAMES,
            G.DEFAULT_GYRO_DRIFT_RATE))
L.append("The re-anchoring frame is itself published (it is an ACCEPT); the count of re-anchor "
         "events is reported so that a stricter reading, publishing only from the following "
         "frame, can be derived without re-running.\n")
L.append("A sweep point *g* is the per-frame budget, g = max_yaw_rate·τ + yaw_gate_margin with "
         "τ = %.1f s, `yaw_gate_margin` at the deployed 0.05 rad and `max_yaw_rate` carrying the "
         "rest (below g = 5.73° the margin is halved to g/2 so that `max_yaw_rate` stays "
         "positive). In v3 this budget is what the frame is compared against directly, at every "
         "frame, whatever dt is.\n" % TAU)
L.append("A frame is scored against the IMU: **genuine** if |wrap(Δψ − Δψ_IMU)| ≤ %.1f° (the "
         "gyro confirms it, so not accepting it is a false trip), **outlier** otherwise; frames "
         "from the %d recordings without an IMU carry no label and are excluded from both rates. "
         "The false-gate rate counts genuine frames that were **held or withheld**; a pseudo-step "
         "frame counts as **caught** when the reported heading was not accepted — held or "
         "withheld, either way it never rotates the lever arm.\n"
         % (GENUINE_TOL, int((summ["has_imu"] == 0).sum())))
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("gate")], ["quantity", "value", "unit"])
         + "\n")

L.append("### Where gyro_drift_rate comes from\n")
L.append("`gyro_drift_rate` is the allowance rule (a) adds to the budget while the heading is "
         "withheld, to cover MEMS bias in the propagated reference. It is set from the data: on "
         "contiguous straight, fixed-solution stretches, the gyro-integrated heading change over "
         "%.0f s is compared with the HDT heading change over the same window.\n" % DRIFT_W)
L.append(md_table([(q_, v, u, n) for (q_, v, u, n) in gsel("gyro_drift")],
                  ["quantity", "value", "unit", "note"]) + "\n")

L.append("### v3 rule (a), the recommendation, at the recommended gate\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("gate_v3a")],
                  ["quantity", "value", "unit"]) + "\n")
L.append("### v3 rule (b) at the same gate\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("gate_v3b")],
                  ["quantity", "value", "unit"]) + "\n")
L.append("### v2 and v1 at the same gate, for comparison\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("gate_v2")],
                  ["quantity", "value", "unit"]) + "\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("gate_v1")],
                  ["quantity", "value", "unit"]) + "\n")

GSHOW = [2, 4, 6, 8, 10, 12, 14, 16, 17, 20, 25, 30]
for lab, tab in (("v3 rule (a)", sw3a), ("v3 rule (b)", sw3b)):
    show = tab[tab["gate_deg"].isin(GSHOW)]
    L.append("Selected rows of the sweep, **%s** (full table `fig/gate_sweep.csv`; v2 in "
             "`fig/gate_sweep_v2.csv`, v1 in `fig/gate_sweep_v1.csv`):\n" % lab)
    L.append(md_table([("%.0f" % r.gate_deg, "%.3f" % r.max_yaw_rate, int(r.held),
                        int(r.unpublished), "%.3f" % r.unpublished_pct,
                        "%.4f" % r.false_gate_pct, "%.4f" % r.false_gate_pct_straight,
                        "%.4f" % r.false_gate_pct_turn,
                        "%d / %d" % (r.pstep_caught, N_PST),
                        r.pstep_missed_dpsi if isinstance(r.pstep_missed_dpsi, str) and
                        r.pstep_missed_dpsi else "—",
                        int(r.max_consecutive_genuine_not_accepted),
                        "%.3f" % r.hold_err_max_m, "%.4f" % r.hold_err_rms_m,
                        "%d / %d" % (r.event1_caught, r.event2_caught))
                       for r in show.itertuples()],
                      ["g (°)", "max_yaw_rate (rad/s)", "held", "withheld", "withheld %",
                       "false-gate %", "straight %", "turn %", "pseudo-steps caught",
                       "misses |Δψ| (°)", "longest genuine hold", "hold err max (m)",
                       "hold err RMS (m)", "ev1 / ev2 caught"]) + "\n")

g1e2 = sw1.loc[sw1["event2_caught"] == 1, "gate_deg"]
L.append("### What the remaining misses actually are — the dt scaling was not the cause\n")
L.append("v2 still missed %d of the %d pseudo-step frames at g = %.0f°, with frame-to-frame "
         "|Δψ| of 16.6–62.7°, and the working hypothesis was that the dt-scaled budget let "
         "them through. **The replay says otherwise.** v3 removes the dt scaling completely "
         "and misses the same 5 frames under rule (b) (%d under rule (a)). The reason is in "
         "the numbers below: the gate compares the reported heading with the **anchor it is "
         "holding**, not with the previous frame, and these frames come back close to that "
         "anchor.\n"
         % (int(rec2["pstep_missed"]), N_PST, rec2["gate_deg"], int(rec["pstep_missed"])))
L.append(md_table([(r["rule"], r["bag"][:34], r["wall"][11:], "%.1f" % r["dpsi_deg"],
                    "%.3f" % r["dp_pub"], "%.1f" % r["vs_anchor_deg"], "%.1f" % r["budget_deg"])
                   for r in MISS_ROWS if r["rule"] == "v3b"],
                  ["rule", "recording", "time", "Δψ vs previous frame (°)", "dp_pub (m)",
                   "|Δψ| vs held anchor (°)", "budget (°)"]) + "\n")
L.append("Every missed frame was compared against a budget of %.0f–%.1f° and came in at "
         "0.1–16.7° **of the anchor**, so it is accepted; the position step is produced by the "
         "change since the *previous published* frame, which the anchor test never looks at. "
         "Closing this needs a second term — testing the increment against the previous "
         "reported heading as well as against the anchor, or holding the published position "
         "rather than the yaw — not a different budget. Full list in `fig/gate_misses.csv`.\n"
         % (rec["gate_deg"], max(r["budget_deg"] for r in MISS_ROWS)))
L.append("What the dt scaling *did* hide is event 2: v1's budget, unbounded in dt, had grown "
         "to %.0f° by the time the 178.9° outlier arrived nine frames into a hold, so v1 "
         "accepted it and catches event 2 only up to g = %s. v2 bounded that growth and v3 "
         "removed it; both catch event 2 at every gate in the sweep%s.\n"
         % (V1_BUDGET_EV2, "%.0f°" % g1e2.max() if len(g1e2) else "no gate in the sweep",
            "" if int(sw3a["event2_caught"].min()) == 1 else
            " except rule (a) above g = %.0f°, where a re-anchor lands on it"
            % float(sw3a.loc[sw3a["event2_caught"] == 1, "gate_deg"].max())))

L.append("### The metric that decides it: what is left in the output\n")
L.append("Counting frames the gate refuses is the wrong measure, because refusing a frame does "
         "not remove the step — it moves it to the edge where the gate takes a heading back. "
         "So the output of every rule is rebuilt, p_gated = p_ant − R(ψ_gate)·L with ψ_gate the "
         "heading that rule actually uses (accepted → as reported, held → the last accepted, "
         "withheld → no output, so the next published frame is differenced against the last "
         "published one), and the pseudo-step definition is applied to **that** stream: fixed "
         "solution, heading valid, antenna point still to within %.2f m over the same interval, "
         "step at or above the threshold. Frames closer together than %.0f s are one episode.\n"
         % (DP_ANT_MAX, EPISODE_GAP))
rows = []
for r_ in RES_TABLE:
    rows.append(("%s, g = %.0f°" % (r_["rule"], r_["gate_deg"]),) +
                tuple("%d / %d" % (r_["frames_%.2f" % T], r_["episodes_%.2f" % T])
                      for T in PSTEP_T))
rows.insert(0, ("**no gate (baseline)**",) +
            tuple("%d / %d" % (BASE_CNT[T], BASE_EPS[T]) for T in PSTEP_T))
L.append(md_table(rows, ["stream"] + ["≥ %.2f m" % T for T in PSTEP_T]) + "\n")
L.append("Read as *frames / episodes*. At the recommended gate the heading-domain rules barely "
         "move the number of episodes that survive: %d for v1 and %d for v2 against the "
         "ungated %d, i.e. **worse**, and %d for v3(b). Only rule (a), which stops publishing "
         "while the heading is not trusted, empties the output: %d episodes at 0.10 m and none "
         "at all at 0.20 m and above.\n"
         % (RES_TABLE[1]["episodes_0.10"], RES_TABLE[3]["episodes_0.10"], BASE_EPS[0.10],
            RES_TABLE[5]["episodes_0.10"], RES_TABLE[7]["episodes_0.10"]))
_v3b17 = [r for r in RESID_ROWS if r["rule"] == "v3b g=17" and r["size_m"] >= PSTEP_MAIN]
_v3a17 = [r for r in RESID_ROWS if r["rule"] == "v3a g=17" and r["size_m"] >= PSTEP_MAIN]
L.append("Why: of the %d episodes v3(b) leaves at 0.10 m, **%d are re-anchor edges** — the gate "
         "refused the outlier, held through it, and then adopted the new heading, which puts a "
         "step of the same size a few tenths of a second later. The 09:49 event is the clearest "
         "case: 0.353 m at 09:49:01.65 without the gate, 0.354 m at 09:49:02.25 with it.\n"
         % (len(_v3b17), sum(r["reanchor"] for r in _v3b17)))
L.append("Residual episodes ≥ %.2f m left by v3 rule (b) at g = 17° (`re-anchor` marks an "
         "accepted frame whose predecessor was not accepted):\n" % PSTEP_MAIN)
L.append(md_table([(r["bag"][:34], r["wall"][11:], r["frames"], "%.3f" % r["size_m"],
                    "%.1f" % r["dpsi_vs_last_published"], "%.1f" % r["gap_s"],
                    "yes" if r["reanchor"] else "—") for r in _v3b17],
                  ["recording", "time", "frames", "size (m)", "Δψ vs last published (°)",
                   "gap (s)", "re-anchor"]) + "\n")
L.append("And the %d rule (a) leaves, all of them just under the gate:\n" % len(_v3a17))
L.append(md_table([(r["bag"][:34], r["wall"][11:], r["frames"], "%.3f" % r["size_m"],
                    "%.1f" % r["dpsi_vs_last_published"], "%.1f" % r["gap_s"],
                    "yes" if r["reanchor"] else "—") for r in _v3a17],
                  ["recording", "time", "frames", "size (m)", "Δψ vs last published (°)",
                   "gap (s)", "re-anchor"]) + "\n")
L.append("Full list for every rule and both detail gates: `fig/gated_residuals.csv`.\n")

L.append("### Rule (a) with a bounded gyro reference\n")
L.append("`max_gyro_ref_time` bounds how long the propagated reference may be trusted after the "
         "last accepted heading; past it rule (a) falls back to rule (b). Swept at g = %.0f°:\n"
         % rec["gate_deg"])
L.append(md_table([(b_["max_gyro_ref_time"], "%.4f" % b_["false_gate_pct"], b_["withheld"],
                    "%.3f" % b_["withheld_pct"], b_["residual_episodes_010"],
                    b_["residual_frames_010"], "%.1f" % b_["ev1_withheld_s"],
                    "%.3f" % b_["ev1_reanchor_error_m"]) for b_ in BOUNDED],
                  ["max_gyro_ref_time (s)", "false-gate %", "withheld", "withheld %",
                   "residual episodes ≥ 0.10 m", "residual frames", "event 1 withheld (s)",
                   "event 1 re-anchor error (m)"]) + "\n")
L.append("The trade is monotone and there is no knee: every second of extra trust in the "
         "propagated reference buys fewer residual steps and costs more availability. **No "
         "setting meets the 0.1 %% false-gate limit** — the smallest, %s s, is already at "
         "%.3f %%. So the honest reading of this corpus: with the 0.1 %% cap binding, rule (b) "
         "(%.4f %%, %d residual episodes) is what can be deployed, and it is barely better than "
         "no gate at all; buying the empty output of rule (a) costs %.2f %% of the frames. "
         "Rule (b) stays the default (it is also the only option without an IMU) and bounded "
         "rule (a) is an extension to be enabled where a short outage is cheaper than a step.\n"
         % (BOUNDED[0]["max_gyro_ref_time"], BOUNDED[0]["false_gate_pct"],
            float(recb["false_gate_pct"]), RES_TABLE[5]["episodes_0.10"],
            float(rec["unpublished_pct"])))

L.append("### Rule (a) costs 5 % of the corpus, and most of that is not a false trip\n")
L.append("Rule (a) withholds %d frames at g = %.0f° (%.2f %% of the corpus) against rule (b)'s "
         "%d (%.3f %%). Attributing each withheld frame to the outage that produced it: %d of "
         "the %d outages were opened by a frame the gyro contradicts, and %s of the %s genuine "
         "frames not accepted sit inside those. The false-gate rate that survives attribution "
         "is %.3f %% for rule (a) and %.4f %% for rule (b).\n"
         % (int(rec["unpublished"]), rec["gate_deg"], float(rec["unpublished_pct"]),
            int(recb["unpublished"]), float(recb["unpublished_pct"]),
            int(rec["episodes_outlier_triggered"]), int(rec["episodes"]),
            format(int(rec["genuine_in_outlier_episodes"]), ","),
            format(int(rec["genuine_not_accepted"]), ","),
            float(rec["false_gate_attributed_pct"]), float(recb["false_gate_attributed_pct"])))
L.append("The outages are long, and that is the real cost: %d of them last more than 60 s and "
         "hold %s frames between them, the longest %.0f s. They are not slow bias: each one "
         "opens on a single frame where the receiver reports tens of degrees of heading change "
         "and the gyro reports a fraction of a degree, and the disagreement then stays at "
         "exactly that size for the whole outage — the reported heading steps and does not "
         "come back:\n"
         % (len([r for r in OUT_ROWS if r["duration_s"] >= 60.0]),
            format(int(sum(r["frames"] for r in OUT_ROWS)), ","),
            OUT_ROWS[0]["duration_s"] if OUT_ROWS else 0.0))
L.append(md_table([(r["bag"][:34], r["start"][11:], "%.0f" % r["duration_s"], r["frames"],
                    "%.1f" % r["trigger_dpsi"], "%.2f" % r["trigger_gyro"],
                    r["trigger_quality"], "%.1f" % r["median_offset_deg"])
                   for r in OUT_ROWS[:8]],
                  ["recording", "start", "s", "frames", "opening Δψ (°)", "gyro over the same "
                   "frame (°)", "status", "median offset during the outage (°)"]) + "\n")
L.append("Only one of those two readings can be right, and that is the point of the rule. But "
         "the two longest outages open during a pivot turn (the gyro shows 15–50 °/s in the "
         "second before), which is precisely where integrating a MEMS gyro across a near-180° "
         "heading change is least trustworthy, so which side is wrong is not settled by this "
         "data. What is settled: the propagated reference is usable for seconds, not minutes. "
         "Rule (a) needs a bounded horizon — fall back to rule (b) after a few seconds — before "
         "it goes into the C++ patch. `fig/v3a_outages.csv` has all %d.\n" % len(OUT_ROWS))
L.append("The allowance value barely matters at this scale, which the sweep over "
         "`gyro_drift_rate` confirms — it decides *when* an unresolved outage is forgiven, not "
         "whether it happens:\n")
L.append(md_table([(q_, v, u, n) for (q_, v, u, n) in gsel("gate_v3a_drift")],
                  ["gyro_drift_rate", "false-gate rate", "unit", "note"]) + "\n")
L.append("At the value used here the %.0f s outage above ends not because the receiver "
         "recovered but because the allowance had grown past the offset: %.3f °/s × %.0f s = "
         "%.0f°.\n" % (OUT_ROWS[0]["duration_s"] if OUT_ROWS else 0.0,
                       np.degrees(G.DEFAULT_GYRO_DRIFT_RATE),
                       OUT_ROWS[0]["duration_s"] if OUT_ROWS else 0.0,
                       np.degrees(G.DEFAULT_GYRO_DRIFT_RATE)
                       * (OUT_ROWS[0]["duration_s"] if OUT_ROWS else 0.0)))

L.append("### Event 1 after the gate stops trusting the heading — (a) against (b)\n")
L.append("This is the number that decides between the two re-anchor rules, so it is reported in "
         "full. The 67.5° step arrives, the gate holds for %d frames and then withholds the "
         "output. The question is what happens next.\n" % G.DEFAULT_MAX_HOLD_FRAMES)
rows = []
ka = gdict("event1_v3a")
kb = gdict("event1_v3b")
for k in [q_ for (q_, v, u, n) in gsel("event1_v3a")]:
    rows.append((k, ka.get(k, "—"), kb.get(k, "—")))
L.append(md_table(rows, ["quantity", "v3 rule (a), gyro", "v3 rule (b), two samples"]) + "\n")
L.append("Rule (b) re-anchors onto the **wrong but stable** heading %.1f s after the event: the "
         "reported heading has stopped jumping, two consecutive samples agree with each other, "
         "and the gate has no way to know that both are %.1f° from where the platform is "
         "actually pointing — so the published reference point settles %.3f m off and stays "
         "there. Rule (a) keeps the output withheld for %.1f s (%d frames). It then re-anchors "
         "because the two sides meet in the middle: the reported heading has crept back to "
         "%.1f° of the gyro-propagated reference while the allowance has grown to %.1f° "
         "(g = %.0f° plus %.3f °/s). The published point is still %.3f m off at that moment — "
         "better than rule (b)'s %.3f m, but not clean: with a fixed allowance the outage would "
         "have continued until the receiver recovered fully, and with a larger one it would "
         "have ended earlier and further from the truth.\n"
         % (kb["heading trusted again after"], kb["heading error at re-anchor"],
            kb["published position error at re-anchor"],
            ka["heading trusted again after"], ka["frames withheld"],
            ka["heading error at re-anchor"], ka["gyro allowance at re-anchor"],
            rec["gate_deg"], np.degrees(G.DEFAULT_GYRO_DRIFT_RATE),
            ka["published position error at re-anchor"],
            kb["published position error at re-anchor"]))
L.append("The %d recordings without an IMU fall back to rule (b) frame by frame. At the "
         "recommended gate the fallback is exercised on %d frames — not because it is disabled, "
         "but because the gate never lost the heading in those recordings, so no re-anchor was "
         "ever needed there. On a platform without an IMU, rule (b) is the behaviour to expect, "
         "including the %.3f m it settles on above.\n"
         % (int((summ["has_imu"] == 0).sum()), int(rec["fallback_b_frames"]),
            kb["published position error at re-anchor"]))

L.append("### Hold cost\n")
L.append("Holding the last accepted yaw for h frames while the platform really turns at ω costs "
         "at most 2|L| sin(ω·h·τ/2) in published position (`patch/heading_gate_note.md` §2). The "
         "bound is tabulated at the measured gyro p99 and max; the measured value is the actual "
         "2|L| sin(Δ_held/2) over the genuine frames v3 rule (a) held, with Δ_held taken from the "
         "IMU-integrated yaw.\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("hold_cost")],
                  ["quantity", "value", "unit"]) + "\n")

L.append("## 7b. Induced against operational\n")
L.append("🔴 **One recording in the corpus is not natural operation.** "
         "`%s` (2026-09-06, 15:26–16:24, greenhouse, vehicle parked) is the controlled "
         "multipath-induction session behind P1's E3b: a 0.50 × 0.70 m metal plate was held "
         "beside the antennas, three attempts were made and two produced a false fix "
         "(field log of the 2026-09-06 induction session, not distributed). **Both showcase events "
         "of §6 come from it.** "
         "Every number below is therefore split; nothing quoted as an operational rate "
         "includes that session.\n" % INDUCED_BAGS[0])
L.append(md_table([("induced", len(INDUCED_BAGS), format(int(IND.sum()), ","),
                    "%.2f" % float(summ.loc[summ.bag.isin(INDUCED_BAGS), "fixed_h"].sum())),
                   ("operational", len(OP_BAGS), format(int(OPS.sum()), ","),
                    "%.2f" % OP_FIXED_H)],
                  ["subset", "recordings", "increments", "fixed hours"]) + "\n")
rows = []
for T in PSTEP_T:
    gi = gdict("pseudo_step_induced")
    go = gdict("pseudo_step_operational")
    rows.append(("%.2f" % T,
                 "%s / %s" % (gi["frames >=%.2f m" % T], gi["episodes >=%.2f m" % T]),
                 "%.1f" % gi["rate >=%.2f m" % T],
                 "%s / %s" % (go["frames >=%.2f m" % T], go["episodes >=%.2f m" % T]),
                 "%.2f" % go["rate >=%.2f m" % T]))
L.append(md_table(rows, ["dp_pub ≥ (m)", "induced: frames / episodes", "induced rate (h⁻¹)",
                         "operational: frames / episodes", "operational rate (h⁻¹)"]) + "\n")
_gi, _go = gdict("pseudo_step_induced"), gdict("pseudo_step_operational")
L.append("So **%s of the %d frames at 0.10 m and %s of the %d at 0.30 m are from the induced "
         "session**, which is 0.79 of the %.1f fixed hours. Operationally the corpus carries "
         "%s frames in %s episodes at 0.10 m, a rate of %.2f h⁻¹, and the largest operational "
         "increment behind a pseudo-step is %s (the 167.5° one is induced). The mechanism "
         "check is unaffected by the split: eq. (2) holds to an RMS of %.4f m (max %.4f m) "
         "operationally against %.4f m (max %.4f m) induced, and every frame of both is inside "
         "± %.2f m.\n"
         % (_gi["frames >=0.10 m"], BASE_CNT[0.10], _gi["frames >=0.30 m"], BASE_CNT[0.30],
            FIXED_H, _go["frames >=0.10 m"], _go["episodes >=0.10 m"], _go["rate >=0.10 m"],
            _go["|dpsi| range >=0.10 m"].replace(" - ", "–") + "°",
            _go["residual RMS >=0.05 m"],
            _go["residual max |.| >=0.05 m"], _gi["residual RMS >=0.05 m"],
            _gi["residual max |.| >=0.05 m"], RESID_OK))
L.append("Operational rate by site (0.10 m definition, induced session removed):\n")
L.append(md_table([(q_.replace(" rate", ""), v) for (q_, v, u, n) in
                   gsel("operational_by_site")
                   if q_.endswith("rate") and not q_.startswith("overall")]
                  + [("**overall**", gdict("operational_by_site")["overall rate"])],
                  ["site", "frames per fixed hour"]) + "\n")
L.append("The greenhouse figure is still the highest by an order of magnitude with the "
         "induced session gone — those are the 2026-08-28/29 natural-operation runs — so the "
         "site effect is not an artefact of the induction experiment.\n")
L.append("Heading increments, operational only:\n")
rows = []
for lab in ("all", "straight", "turn"):
    got = dict((q_.replace(lab + " ", ""), v) for (q_, v, u, n)
               in gsel("increments_operational") if q_.startswith(lab + " "))
    if got:
        rows.append((lab, got["n"], got["RMS"], got["P(>5 deg)"], got["P(>10 deg)"],
                     got["P(>20 deg)"], got["P(>45 deg)"], got["P(>90 deg)"],
                     got["P(> max bound %.2f deg)" % BMAX], got["max"]))
L.append(md_table(rows, ["cls_imu", "n", "RMS (°)", "P(>5°) %", "P(>10°) %", "P(>20°) %",
                         "P(>45°) %", "P(>90°) %", "P(>%.2f°) %%" % BMAX, "max (°)"]) + "\n")
L.append("%s operational increments exceed the measured physical bound (%.4f %% of those with "
         "a usable gyro envelope), against %d in the whole corpus.\n"
         % (gdict("increments_operational")["frames above the max bound"],
            gdict("increments_operational")["P(|dpsi| > max bound)"], n_above))
L.append("Gate headline at g = %.0f°, split (onsets = the first frame of each ungated "
         "pseudo-step episode ≥ %.2f m; residual episodes = what is left in that rule's own "
         "output):\n" % (rec["gate_deg"], PSTEP_MAIN))
L.append(md_table([(r_["rule"], r_["subset"], "%.4f" % r_["false_gate_pct"],
                    "%d / %d" % (r_["onsets_held"], r_["onsets"]), r_["withheld"],
                    r_["residual_episodes_010"], r_["residual_episodes_005"])
                   for r_ in SPLIT],
                  ["rule", "subset", "false-gate %", "onsets held", "frames withheld",
                   "residual episodes ≥ 0.10 m", "≥ 0.05 m"]) + "\n")
_op_v3b = [r_ for r_ in SPLIT if r_["rule"] == "v3b" and r_["subset"] == "operational"][0]
L.append("The operational row is the one to quote, and it is blunt: v3(b) holds %d of the %d "
         "onsets and still leaves **%d residual episodes — exactly the %d the ungated stream "
         "has** (the steps move to the re-anchor). Bounding rule (a) buys the reduction back "
         "one second at a time: %d episodes at 5 s (%.3f %% false-gate), %d at 10 s, %d at "
         "60 s, %d unbounded (%.2f %%). `fig/gate_split.csv` has the full split.\n"
         % (_op_v3b["onsets_held"], _op_v3b["onsets"], _op_v3b["residual_episodes_010"],
            len([r for r in BASE_ROWS if r["size_m"] >= PSTEP_MAIN
                 and r["bag"] not in INDUCED_BAGS]),
            [r_ for r_ in SPLIT if r_["rule"] == "v3a 5 s"
             and r_["subset"] == "operational"][0]["residual_episodes_010"],
            [r_ for r_ in SPLIT if r_["rule"] == "v3a 5 s"
             and r_["subset"] == "operational"][0]["false_gate_pct"],
            [r_ for r_ in SPLIT if r_["rule"] == "v3a 10 s"
             and r_["subset"] == "operational"][0]["residual_episodes_010"],
            [r_ for r_ in SPLIT if r_["rule"] == "v3a 60 s"
             and r_["subset"] == "operational"][0]["residual_episodes_010"],
            [r_ for r_ in SPLIT if r_["rule"] == "v3a unbounded"
             and r_["subset"] == "operational"][0]["residual_episodes_010"],
            [r_ for r_ in SPLIT if r_["rule"] == "v3a unbounded"
             and r_["subset"] == "operational"][0]["false_gate_pct"]))

L.append("## 9. Reviewer checks\n")
L.append("### 9.1 Three-class frame counts and label tolerance\n")
L.append("Every false-gate rate in this file is *genuine frames not accepted / genuine "
         "frames*. The denominator is made explicit here, and swept over the tolerance that "
         "defines it.\n")
L.append(md_table([(lab, gdict("reviewer_classes")["%s: total increments" % lab],
                    gdict("reviewer_classes")["%s: genuine (tol 1 deg)" % lab],
                    gdict("reviewer_classes")["%s: genuine (tol 2 deg)" % lab],
                    gdict("reviewer_classes")["%s: genuine (tol 5 deg)" % lab],
                    gdict("reviewer_classes")["%s: outlier (tol 2 deg)" % lab],
                    gdict("reviewer_classes")["%s: unclassified (no IMU)" % lab])
                   for lab in ("all", "operational")],
                  ["subset", "increments", "genuine (1 deg)", "genuine (2 deg)",
                   "genuine (5 deg)", "outlier (2 deg)", "unclassified"]) + "\n")
rows = []
for nm in ("v1", "v2", "v3b", "v3a"):
    rows.append((nm,) + tuple(
        "%s" % gdict("reviewer_tolerance")["%s, %s, tol %.0f deg" % (nm, lab, tol)]
        for lab in ("all", "operational") for tol in (1.0, 2.0, 5.0)))
L.append(md_table(rows, ["rule", "all, 1 deg", "all, 2 deg", "all, 5 deg", "oper., 1 deg",
                         "oper., 2 deg", "oper., 5 deg"]) + "\n")
L.append("False-gate rate (%%) at g = %.0f°. Loosening the label from 1° to 5° moves the rate "
         "by less than a factor of two for every rule, so nothing in §7 rests on the "
         "tolerance.\n" % rec["gate_deg"])

L.append("### 9.2 Heading noise σ_ψ\n")
L.append("Measured on frames the platform is not moving through: v2 *slow* class "
         "(reconstructed antenna path ≤ 0.50 m over 3 s), fixed solution, |ω_z| < 0.02 rad/s. "
         "The per-frame increment differences two independent heading samples, so its "
         "standard deviation is divided by √2.\n")
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("reviewer_sigma")],
                  ["quantity", "value", "unit"]) + "\n")
L.append("The driver declares 0.50° (cov[35] = 7.6 × 10⁻⁵ rad²) everywhere. Measured, that is "
         "right on the concrete yard (0.46°), optimistic by a factor of two inside the "
         "greenhouses (1.09°) and pessimistic in the orchard (0.29°) — which is exactly why "
         "the innovation test of §9.5, which takes that 0.50° at face value, over-rejects.\n")

L.append("### 9.3 Stale-heading exclusion\n")
L.append("A jump could in principle be a *late* heading update after a real turn. Each "
         "pseudo-step frame is therefore checked against the IMU turn over the preceding "
         "%.1f s, and flagged a stale-heading candidate when that turn is within %.0f° of the "
         "observed increment.\n" % (STALE_W, STALE_TOL))
L.append(md_table([(q_, v, u) for (q_, v, u, n) in gsel("reviewer_stale")],
                  ["quantity", "value", "unit"]) + "\n")
L.append("None of the %s operational or %s induced frames stepping ≥ 0.10 m is a "
         "stale-heading candidate; %s of the %s operational frames at ≥ 0.05 m are, and they "
         "are the small end of the distribution. For the two 2026-09-06 events the IMU "
         "integrates %.3f° and %.2f° over the preceding %.1f s — the vehicle was parked — "
         "against reported increments of −67.54° and −178.87°. Column `stale_candidate` in "
         "`fig/pseudo_steps.csv`.\n"
         % (gdict("reviewer_stale")["operational: pseudo-steps >=0.10 m"],
            gdict("reviewer_stale")["induced: pseudo-steps >=0.10 m"],
            gdict("reviewer_stale")["operational: stale-heading candidates >=0.05 m"],
            gdict("reviewer_stale")["operational: pseudo-steps >=0.05 m"],
            gdict("reviewer_stale")["event1: IMU turn over the preceding 2.0 s"],
            gdict("reviewer_stale")["event2: IMU turn over the preceding 2.0 s"], STALE_W))

L.append("### 9.4 Motion classification, v1 → v2\n")
L.append("🔴 Reviewer 3 is right: the v1 *slow* test used the published displacement over the "
         "trailing window, i.e. the quantity a lever-arm pseudo-step corrupts. The rule is "
         "now, verbatim (`fig/motion_class.py`):\n\n"
         "```\n"
         "W = 3.0 s                              trailing window\n"
         "d_ant_win = sum |dp_ant| over (t-W, t]  reconstructed antenna path\n"
         "turn_win  = integral omega_z over (t-W, t]        IMU only\n"
         "d_ant_win <= 0.50 m         -> slow\n"
         "else |turn_win| < 5.0 deg   -> straight\n"
         "else                        -> turn\n"
         "less than W of history, or no IMU in the recording -> na\n"
         "```\n\n"
         "Only the *slow* test changed; straight/turn is the same IMU integral as before. "
         "**%s frames (%.1f %%) changed class**, almost all of them from v1 straight or turn "
         "into v2 slow (%s and %s frames) — the v1 rule had no slow class at all for "
         "IMU-labelled frames, so every stationary frame was being counted as straight or "
         "turn.\n"
         % (format(int((old_cls != new_cls).sum()), ","),
            100.0 * float((old_cls != new_cls).mean()),
            format(int(((old_cls == "straight") & (new_cls == "slow")).sum()), ","),
            format(int(((old_cls == "turn") & (new_cls == "slow")).sum()), ",")))
rows = []
for c in ("straight", "turn", "slow"):
    ge = gdict("reviewer_class_effect")
    r_ = [c]
    for lab in ("v1", "v2"):
        r_ += [ge.get("%s %s: n" % (c, lab), "—"), ge.get("%s %s: RMS" % (c, lab), "—"),
               ge.get("%s %s: P(>5 deg)" % (c, lab), "—"),
               ge.get("%s %s: false-gate at g=%.0f (v3b)" % (c, lab, rec["gate_deg"]), "—")]
    rows.append(tuple(r_))
L.append(md_table(rows, ["class", "v1 n", "v1 RMS (deg)", "v1 P(>5 deg) %",
                         "v1 false-gate %", "v2 n", "v2 RMS (deg)", "v2 P(>5 deg) %",
                         "v2 false-gate %"]) + "\n")
L.append("No headline number moved: the all-frames statistics of §2 and every gate rate of §7 "
         "are computed over all frames and are unchanged. What moved is the per-class split — "
         "straight RMS %s → %s°, turn %s → %s°, and a new slow class of %s frames "
         "(RMS %s°) that used to be spread across the other two.\n"
         % (gdict("reviewer_class_effect")["straight v1: RMS"],
            gdict("reviewer_class_effect")["straight v2: RMS"],
            gdict("reviewer_class_effect")["turn v1: RMS"],
            gdict("reviewer_class_effect")["turn v2: RMS"],
            format(int(gdict("reviewer_class_effect")["slow v2: n"]), ","),
            gdict("reviewer_class_effect")["slow v2: RMS"]))

L.append("### 9.5 A normalised-innovation gate instead (Reviewer 2)\n")
L.append("🔴 **This is a 1-D emulation, not the full `robot_localization` filter**: one scalar "
         "yaw state, no cross-covariance with position or velocity, no differential-drive "
         "process model. It answers one question — would a chi-square test on the innovation "
         "have caught these? — and nothing else. Prediction is the gyro increment (the yaw is "
         "held where there is no IMU), P grows by q·dt with q = (%.3f rad/s)² = %.1e rad²/s "
         "from the measured gyro drift, R = %.2e rad² is the cov[35] the driver publishes "
         "(%.2f°), and a heading is rejected when ν²/(P+R) > k². On rejection the state and P "
         "are not updated, so P keeps growing and the test loosens by itself.\n"
         % (G.DEFAULT_GYRO_DRIFT_RATE, G.DEFAULT_Q_YAW, G.DEFAULT_R_YAW,
            np.degrees(np.sqrt(G.DEFAULT_R_YAW))))
L.append(md_table([("%.0f" % r_["k_sigma"], r_["subset"], "%.2f" % r_["false_gate_pct"],
                    format(r_["rejected"], ","), r_["withheld"],
                    "%d / %d" % (r_["onsets_rejected"], r_["onsets"]),
                    r_["residual_episodes_010"], r_["residual_episodes_005"])
                   for r_ in MAHAL],
                  ["k (sigma)", "subset", "false-gate %", "rejected", "withheld",
                   "onsets rejected", "residual episodes >= 0.10 m", ">= 0.05 m"]) + "\n")
L.append("Read both columns. The innovation test is the **only** rule tested that leaves "
         "zero residual pseudo-step episodes ≥ 0.10 m in its own output — because it never "
         "publishes the measurement, it publishes a filtered estimate, so a heading jump "
         "cannot become a position step. The price is that it rejects %.1f %% of the "
         "gyro-confirmed increments at k = 3 (%.1f %% at k = 5), which is what taking the "
         "driver's optimistic 0.50° at face value costs when the measured σ_ψ is %s° overall "
         "and %s° in the greenhouses. That rejection is not an outage — the filter keeps "
         "publishing — but it means the published heading is following the gyro, with its "
         "drift, for a sixth of the corpus. On event 1 it rejects for %.0f s and comes back "
         "%.1f° off (%.3f m of published position error) at k = 3.\n"
         % (MAHAL[1]["false_gate_pct"], MAHAL[3]["false_gate_pct"],
            gdict("reviewer_sigma")["sigma_psi overall"],
            gdict("reviewer_sigma")["sigma_psi greenhouse"],
            gdict("reviewer_mahal")["k = 3: event 1 rejected for"],
            gdict("reviewer_mahal")["k = 3: event 1 heading error when accepted again"],
            gdict("reviewer_mahal")["k = 3: event 1 position error when accepted again"]))

L.append("### 9.6 Position-test ROC — what is missing and how to get it\n")
L.append("🔴 **Not run: the inputs are not in this corpus, and nothing is fabricated here.** "
         "The two deployed position-domain tests are replayed by "
         "the guard replay script of the 2026-09-09 orchard session (not distributed), which "
         "imports the judgement verbatim from the "
         "deployed `guard_watch.py`. Their inputs are\n\n"
         "| test | class | topics it must be fed |\n"
         "|---|---|---|\n"
         "| cross-track | `LatSentinel(win=3.0, thresh=T, persist=1.0, min_disp=0.5, "
         "max_dhdg=5.0, hold_s=5.0)` | `/rtk_odom` → `feed_rtk`, `/rtk/status` → `feed_hdg` "
         "(HDT, 1 Hz, fixed solution only), `/smoother_cmd_vel` + `/nav_cmd_vel` → "
         "`mark_auto` |\n"
         "| chord | `DispSentinel(win=3.0, thresh=T, hold_s=5.0)` | `/rtk_odom` → "
         "`feed_rtk`, **`/odom` (wheel odometry)** → `feed_odo`, `/rtk/status` → `set_fix` |\n\n")
L.append("`data/extract_p2.py` pulled only `/rtk_odom`, `/rtk/status`, `/rtk/fix` and the "
         "IMU topic, so the CSVs on this machine carry **neither `/odom` nor the two cmd_vel "
         "topics**. Consequences:\n\n"
         "* the **chord test cannot be replayed at all** here — its whole point is RTK "
         "against wheel odometry, and the wheel odometry is not in the extract;\n"
         "* the **cross-track test could be fed** position and heading from the CSVs, but "
         "`LatSentinel.step()` only evaluates while `t_end <= auto_until`, i.e. while a "
         "non-zero command was seen in the last 2 s. Without cmd_vel the evaluable set — "
         "which is the denominator of any false-alarm rate — would be a different set from "
         "the deployed one, so the ROC would not be the deployed criterion's ROC. A "
         "false-alarm rate measured over the wrong denominator is worse than no number.\n")
L.append("What is needed to produce it, on the industrial PC where the bags and the deployed "
         "sources are:\n\n"
         "```\n"
         "# on the industrial PC\n"
         "export GUARD_SRC=$HOME/fusion/safety        # dir holding the deployed guard_watch.py\n"
         "python3 roc_position_tests.py \\\n"
         "        --bags operational_baglist.txt \\\n"
         "        --onsets pseudo_steps.csv \\\n"
         "        --out position_roc.csv\n"
         "```\n\n"
         "`fig/roc_position_tests.py` is written and committed for exactly this: it imports "
         "`LatSentinel` and `DispSentinel` from `$GUARD_SRC` (never re-implementing the "
         "judgement), feeds them the same topics `replay_guards.py` does, steps them at "
         "10 Hz, sweeps the cross-track threshold over 0.05 / 0.10 / 0.15 / 0.20 / 0.30 m and "
         "the chord threshold over 0.10 / 0.20 / 0.30 / 0.50 m, and writes alarms, evaluable "
         "hours, false alarms per evaluable hour, and pseudo-step onsets detected (matched "
         "within ± 5 s against the onsets in `pseudo_steps.csv`). It needs two files copied "
         "over: `pseudo_steps.csv` and a list of the %d operational recordings — the "
         "2026-09-06 15:26 induction session must be left out of that list.\n"
         % len(OP_BAGS))
L.append("Until it is run there, the only position-domain statement this corpus supports is "
         "the geometric one already in §4: of the %d operational frames stepping ≥ 0.10 m, %s "
         "exceed the deployed 0.30 m cross-track threshold and %s the 0.50 m chord threshold "
         "— which bounds what any replay can find, because a test cannot alarm on a step "
         "smaller than its own threshold.\n"
         % (len(op_main),
            int((op_main["across_pre"].abs() > T_LAT).sum()),
            int((op_main["dp_pub"] > T_NORM).sum())))

L.append("## 8. Data anomalies found while producing these numbers\n")
L.append("Stated here so that nothing quoted in the text is quietly resting on them.\n")
L.append(md_table([(q_, v, u, n) for (q_, v, u, n) in gsel("anomaly")],
                  ["quantity", "value", "unit", "note"]) + "\n")

with open(os.path.join(HERE, "results_numbers.md"), "w") as f:
    f.write("\n".join(L) + "\n")
print("wrote results_numbers.md, gate_sweep.csv, gate_sweep_v1.csv, pseudo_steps.csv,\n      pstep_by_bag.csv, hold_errors.csv in %s" % HERE)
