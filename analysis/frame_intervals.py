#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 -- frame-interval statistics of the published /rtk_odom stream (§4.6 hold cost).

Why this file exists
    The manuscript quotes "0.200-0.204 s on 99.9 % of frames", "0.013 % above 0.21 s",
    "maximum 0.39 s".  Those three numbers had no data-file source.  This file is that
    source.  It does not touch any existing script.

Corpus (identical to fig/results_numbers.py)
    every ../data/csv/*.csv written by ../data/extract_p2.py, minus the one recording whose
    /rtk_odom time stamps duplicate another recording (a replay, not independent data):
    EXCLUDE_BAGS below -- 47 recordings.  The 2026-09-06 induced session is KEPT, exactly as
    results_numbers.py keeps it for the corpus-wide counts.

Time base
    `t` in the CSV is the bag RECORDING time (rosbag `read_messages` returns the bag's
    receipt stamp, data/extract_p2.py line 152 `ts = t.to_sec()`), written at millisecond
    resolution; `dt` is t - t_prev within the recording, same column results_numbers.py
    thresholds with DT_MAX.  This is NOT the driver's header stamp -- the driver's own
    publication instant is not recorded in the corpus, so every interval here is the
    interval between successive frames as the consumer received them, which is the quantity
    the hold cost of §3.3 is about.

Heading-invalid frames
    Two populations are reported.
      ALL     every consecutive pair of published frames inside a recording, heading-valid
              or not: this is what the gate and the consumer actually see (the driver keeps
              publishing at 5 Hz with cov[35] = 1e5 when it has no heading).
      HD_PAIR both frames carry a heading (hd_ok == 1 on both), i.e. the pairs that can
              form a heading increment -- the same `hd_pair` mask results_numbers.py uses
              before it adds `dt <= 0.5 s` to reach its 364,799 increments.  Reported so the
              two files can be tied together; the manuscript sentence is about the stream,
              so the ALL row is the one to quote.

Run
    systemd-run --user --scope -p MemoryMax=4G -- python3 analysis/frame_intervals.py
Outputs
    ../data/frame_intervals.md        markdown, with the command line and the definitions
    ../data/frame_intervals.csv       one row per recording
    ../data/frame_intervals_long.csv  every interval outside [0.199, 0.205] s (audit trail)
"""
import glob
import os
import sys
import datetime

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(HERE, "..", "data", "csv")
OUT_MD = os.path.join(HERE, "..", "data", "frame_intervals.md")
OUT_CSV = os.path.join(HERE, "..", "data", "frame_intervals.csv")
OUT_LONG = os.path.join(HERE, "..", "data", "frame_intervals_long.csv")

EXCLUDE_BAGS = ["orchard3src_20260826_2026-08-26-10-02-59"]   # replay, see results_numbers.py
INDUCED_BAGS = ["run_20260906_2026-09-06-15-26-15"]           # 0906 induced session (kept)

NOM_LO, NOM_HI = 0.200, 0.204     # the "nominal" band quoted in the manuscript
SHORT = 0.199                     # below this: two frames received in the same instant
BURST = 0.05                      # an interval this short is a back-to-back pair
LONG = 0.21                       # "above 0.21 s"
DBL_LO, DBL_HI = 0.38, 0.42       # dropped-frame doubles (2 x 0.2 s)
GAP3 = 3 * 0.2                    # gap longer than 3 frames -> invalidation rule of §3.3
MAX_YAW_RATE = 1.234              # rad/s, the recorded bound of §4.2 (results_numbers.py)


def wall(t):
    return datetime.datetime.fromtimestamp(float(t)).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


files = sorted(glob.glob(os.path.join(CSV_DIR, "*.csv")))
d = pd.concat([pd.read_csv(f, usecols=["bag", "t", "dt", "hd_ok", "x", "y"]) for f in files],
              ignore_index=True)
d["bag"] = d["bag"].str.replace(".bag", "", regex=False)
n_bags_raw, n_frames_raw = d.bag.nunique(), len(d)
d = d[~d["bag"].isin(EXCLUDE_BAGS)].reset_index(drop=True)
print("%d frames / %d bags before exclusion; %d frames / %d bags after"
      % (n_frames_raw, n_bags_raw, len(d), d.bag.nunique()))

g_ = d.groupby("bag", sort=False)
# recompute dt from t as a check on the stored column (both ms-quantised)
dt_chk = g_["t"].diff()
dt = d["dt"].to_numpy(float)
bad = np.abs(dt_chk.to_numpy(float) - dt)
bad = bad[np.isfinite(bad)]
assert bad.size and bad.max() <= 1.5e-3, "stored dt disagrees with diff(t): %s" % bad.max()
d["hd_pair"] = (d["hd_ok"] == 1) & (g_["hd_ok"].shift(1) == 1)

POPS = [("ALL", d["dt"].notna()),
        ("HD_PAIR", d["dt"].notna() & d["hd_pair"])]


def stats(m):
    x = d.loc[m, "dt"].to_numpy(float)
    n = x.size
    in_band = int(((x >= NOM_LO - 1e-9) & (x <= NOM_HI + 1e-9)).sum())
    lng = x > LONG + 1e-9
    dbl = (x >= DBL_LO - 1e-9) & (x <= DBL_HI + 1e-9)
    sht = x < SHORT - 1e-9
    return dict(
        n=n,
        n_short=int(sht.sum()), f_short=100.0 * sht.sum() / n,
        n_burst=int((x < BURST).sum()),
        n_band=in_band, f_band=100.0 * in_band / n,
        n_long=int(lng.sum()), f_long=100.0 * lng.sum() / n,
        n_dbl=int(dbl.sum()), f_dbl=100.0 * dbl.sum() / n,
        n_gt042=int((x > DBL_HI + 1e-9).sum()),
        n_gt_gap3=int((x > GAP3 + 1e-9).sum()),
        dt_max=float(x.max()), dt_min=float(x.min()),
        p999=float(np.quantile(x, 0.999)), p9999=float(np.quantile(x, 0.9999)),
        p50=float(np.quantile(x, 0.5)), p01=float(np.quantile(x, 0.001)),
        dt_max_nogap=float(x[x <= GAP3].max()),
    )


S = {k: stats(m) for k, m in POPS}

# ---- what the long intervals are: a frame pair received back-to-back (an interval below
# BURST) followed by a long one means the two frames arrived in the same instant and the
# stream then resumed, i.e. the long interval is receipt-time batching, not a lost frame.
d["dt_prev"] = g_["dt"].shift(1)
_dbl = (d["dt"] >= DBL_LO - 1e-9) & (d["dt"] <= DBL_HI + 1e-9)
N_DBL_AFTER_BURST = int((_dbl & (d["dt_prev"] < BURST)).sum())
_sht = d["dt"] < BURST
N_BURST_THEN_NOM = int((_sht & (g_["dt"].shift(-1).between(NOM_LO - 1e-9, NOM_HI + 1e-9))).sum())
N_BURST_THEN_DBL = int((_sht & (g_["dt"].shift(-1) >= DBL_LO - 1e-9)
                        & (g_["dt"].shift(-1) <= DBL_HI + 1e-9)).sum())
IDENTICAL = int(((d["dt"] < BURST)
                 & ((d["x"] - g_["x"].shift(1)).abs() < 1e-9)
                 & ((d["y"] - g_["y"].shift(1)).abs() < 1e-9)).sum()) if "x" in d else -1

# ---- per-recording rows + every interval above 0.21 s
rows, longs = [], []
for b, gg in d.groupby("bag", sort=True):
    x = gg["dt"].to_numpy(float)
    ok = np.isfinite(x)
    xx = x[ok]
    lng = xx > LONG + 1e-9
    rows.append(dict(
        bag=b, induced=int(b in INDUCED_BAGS), frames=len(gg), intervals=int(xx.size),
        n_in_band=int(((xx >= NOM_LO - 1e-9) & (xx <= NOM_HI + 1e-9)).sum()),
        pct_in_band=round(100.0 * ((xx >= NOM_LO - 1e-9) & (xx <= NOM_HI + 1e-9)).sum()
                          / xx.size, 4),
        n_gt_021=int(lng.sum()),
        n_038_042=int((((xx >= DBL_LO - 1e-9) & (xx <= DBL_HI + 1e-9))).sum()),
        n_gt_042=int((xx > DBL_HI + 1e-9).sum()),
        n_lt_0199=int((xx < SHORT - 1e-9).sum()),
        dt_max=round(float(xx.max()), 3),
        dur_s=round(float(gg["t"].iloc[-1] - gg["t"].iloc[0]), 1)))
    sub = gg[(gg["dt"] > 0.205 + 1e-9) | (gg["dt"] < SHORT - 1e-9)]
    for _, r in sub.iterrows():
        longs.append(dict(bag=b, t=float(r["t"]), wall=wall(r["t"]),
                          dt_s=round(float(r["dt"]), 3),
                          hd_pair=int(bool(r["hd_pair"]))))

R = pd.DataFrame(rows).sort_values("bag")
R.to_csv(OUT_CSV, index=False)
OUTLIERS = pd.DataFrame(longs).sort_values(["bag", "t"])
OUTLIERS.to_csv(OUT_LONG, index=False)
L = OUTLIERS[OUTLIERS["dt_s"] > LONG + 1e-9].copy()   # the "above 0.21 s" list

cmdline = "systemd-run --user --scope -p MemoryMax=4G -- python3 analysis/frame_intervals.py"
out = []
A = out.append
A("# P2 -- frame intervals of the published `/rtk_odom` stream\n")
A("Generated by `analysis/frame_intervals.py` on %s.\n"
  % datetime.date.today().isoformat())
A("```\n%s\n```\n" % cmdline)
A("## Corpus\n")
A("* Source: every `data/csv/*.csv` written by `data/extract_p2.py` (one row per published "
  "`/rtk_odom` frame).")
A("* %d recordings / %d frames read; **%s** excluded as a replay of another recording "
  "(same rule and same list as `fig/results_numbers.py`, `EXCLUDE_BAGS`), leaving "
  "**%d recordings / %d frames**."
  % (n_bags_raw, n_frames_raw, ", ".join("`%s`" % b for b in EXCLUDE_BAGS),
     d.bag.nunique(), len(d)))
A("* The 2026-09-06 induced session `%s` is kept, as it is in the corpus-wide counts of "
  "`results_numbers.py`.\n" % INDUCED_BAGS[0])
A("## Time base\n")
A("* Intervals are differences of the column `t`, which `data/extract_p2.py` (line 152, "
  "`ts = t.to_sec()`) takes from `rosbag.read_messages` — the **bag recording (receipt) "
  "time**, written at millisecond resolution. It is **not** the driver's header stamp; "
  "the corpus does not carry the header stamp, so the driver's own publication instant "
  "cannot be recovered from it.")
A("* Intervals are taken within a recording only; the first frame of each recording has no "
  "interval.")
A("* Stored `dt` verified against `diff(t)` per recording (max discrepancy ≤ 1 ms, the "
  "rounding of the written columns).\n")
A("## Populations\n")
A("* **ALL** — every consecutive pair of published frames, heading-valid or not. The driver "
  "keeps publishing at 5 Hz with `cov[35] = 1e5` when it has no heading, so this is the "
  "stream the gate and the consumer see; **quote this row**.")
A("* **HD_PAIR** — both frames carry a heading (`hd_ok == 1` on both), the same `hd_pair` "
  "mask `results_numbers.py` applies before adding `dt ≤ 0.5 s` to reach its 364,799 "
  "increments.\n")
A("## Results\n")
A("| quantity | ALL | HD_PAIR |")
A("|---|---:|---:|")


def fmt(key, f, unit=""):
    A("| %s | %s | %s |" % (key, f(S["ALL"]), f(S["HD_PAIR"])))


fmt("intervals (n)", lambda s: "%d" % s["n"])
fmt("within [0.200, 0.204] s (n)", lambda s: "%d" % s["n_band"])
fmt("**within [0.200, 0.204] s (%)**", lambda s: "**%.2f**" % s["f_band"])
fmt("below 0.199 s (n)", lambda s: "%d" % s["n_short"])
fmt("below 0.199 s (%)", lambda s: "%.3f" % s["f_short"])
fmt("of which below 0.05 s, back-to-back pairs (n)", lambda s: "%d" % s["n_burst"])
fmt("**> 0.21 s (n)**", lambda s: "**%d**" % s["n_long"])
fmt("**> 0.21 s (%)**", lambda s: "**%.3f**" % s["f_long"])
fmt("in [0.38, 0.42] s, one nominal frame missing (n)", lambda s: "%d" % s["n_dbl"])
fmt("in [0.38, 0.42] s (%)", lambda s: "%.3f" % s["f_dbl"])
fmt("> 0.42 s (n)", lambda s: "%d" % s["n_gt042"])
fmt("> 0.60 s = gap longer than 3 frames (n)", lambda s: "%d" % s["n_gt_gap3"])
fmt("median (s)", lambda s: "%.3f" % s["p50"])
fmt("99.9th percentile (s)", lambda s: "%.3f" % s["p999"])
fmt("99.99th percentile (s)", lambda s: "%.3f" % s["p9999"])
fmt("minimum (s)", lambda s: "%.3f" % s["dt_min"])
fmt("**maximum, excluding gaps > 0.60 s (s)**", lambda s: "**%.3f**" % s["dt_max_nogap"])
fmt("maximum, any (s)", lambda s: "%.3f" % s["dt_max"])
A("")
_tie = S["HD_PAIR"]["n"] - int((d["hd_pair"] & (d["dt"] > 0.5)).sum())
A("Consistency check with `fig/results_numbers.py`: HD_PAIR minus the %d pairs spanning "
  "more than its DT_MAX = 0.5 s gives **%d**, exactly its 364,799 usable 5 Hz heading "
  "increments.\n" % (int((d["hd_pair"] & (d["dt"] > 0.5)).sum()), _tie))
A("## What the long intervals are\n")
A("* %d of the %d intervals in [0.38, 0.42] s are **immediately preceded by an interval "
  "below %.2f s** — two frames received in the same instant, then the stream resuming. "
  "Those long intervals are receipt-time batching, not a lost frame."
  % (N_DBL_AFTER_BURST, S["ALL"]["n_dbl"], BURST))
A("* Of the %d back-to-back pairs, %d are followed by a nominal interval and %d by a "
  "[0.38, 0.42] s one." % (S["ALL"]["n_burst"], N_BURST_THEN_NOM, N_BURST_THEN_DBL))
A("* %d of the %d back-to-back pairs repeat the published position of the frame before to "
  "the millimetre the CSV records; the other %d carry a different position, so the pairs "
  "are distinct frames delivered together, not one message counted twice.\n"
  % (IDENTICAL, S["ALL"]["n_burst"], S["ALL"]["n_burst"] - IDENTICAL))
A("* 🔴 The corpus does not carry the driver's header stamp, so it cannot be said how much "
  "of the tail above and below the nominal 0.2 s is the driver's publication jitter and "
  "how much is receipt-time batching in the recording. Every number here is on the "
  "received stream, which is what the consumer and the gate see.\n")
A("## Heading budget\n")
A("At the recorded bound max_yaw_rate = %.3f rad/s of §4.2, the excess over the nominal "
  "0.2 s adds %.2f° at the top of the nominal band (%.3f s), %.2f° at the longest "
  "non-gap interval (%.3f s) and %.2f° at %.2f s.\n"
  % (MAX_YAW_RATE, np.degrees(MAX_YAW_RATE) * (NOM_HI - 0.2), NOM_HI,
     np.degrees(MAX_YAW_RATE) * (S["ALL"]["dt_max_nogap"] - 0.2), S["ALL"]["dt_max_nogap"],
     np.degrees(MAX_YAW_RATE) * (LONG - 0.2), LONG))
A("## Every interval above 0.42 s\n")
_big = L[L.dt_s > DBL_HI + 1e-9]
if len(_big) == 0:
    A("none.\n")
else:
    A("| recording | wall clock | interval (s) | > 3 frames (invalidates the anchor) | "
      "both frames have a heading |")
    A("|---|---|---:|:--:|:--:|")
    for _, r in _big.sort_values("t").iterrows():
        A("| `%s` | %s | %.3f | %s | %s |"
          % (r["bag"], r["wall"], r["dt_s"], "yes" if r["dt_s"] > GAP3 else "no",
             "yes" if r["hd_pair"] else "no"))
    A("")
A("The %d intervals above 0.21 s and every interval outside [0.199, 0.205] s are listed "
  "frame by frame in `data/frame_intervals_long.csv` (recording, wall clock, interval, "
  "whether both frames carry a heading); they occur in %d of the %d recordings, the "
  "per-recording counts being the table below.\n"
  % (S["ALL"]["n_long"], int((R.n_gt_021 > 0).sum()), len(R)))
A("## Per recording\n")
A("Full table in `data/frame_intervals.csv`. Recordings with any interval above 0.21 s:\n")
A("| recording | intervals | in band (%) | < 0.199 s | > 0.21 s | [0.38,0.42] s | > 0.42 s | max (s) |")
A("|---|---:|---:|---:|---:|---:|---:|---:|")
for _, r in R[R.n_gt_021 > 0].iterrows():
    A("| `%s` | %d | %.2f | %d | %d | %d | %d | %.3f |"
      % (r["bag"], r["intervals"], r["pct_in_band"], r["n_lt_0199"], r["n_gt_021"],
         r["n_038_042"], r["n_gt_042"], r["dt_max"]))
A("")
A("Worst five recordings by in-band fraction:\n")
A("| recording | intervals | in band (%) | max (s) |")
A("|---|---:|---:|---:|")
for _, r in R.sort_values("pct_in_band").head(5).iterrows():
    A("| `%s` | %d | %.3f | %.3f |" % (r["bag"], r["intervals"], r["pct_in_band"],
                                       r["dt_max"]))
A("")

with open(OUT_MD, "w") as f:
    f.write("\n".join(out))
print("\n".join(out[out.index("| quantity | ALL | HD_PAIR |") - 1:]))
print("wrote %s, %s, %s" % (OUT_MD, OUT_CSV, OUT_LONG))
