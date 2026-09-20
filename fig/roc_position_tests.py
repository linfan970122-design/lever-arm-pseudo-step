#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 -- ROC of the two DEPLOYED position-domain tests (Reviewers 1 and 3).

🔴 This script does NOT run on the laptop: the inputs it needs are not in
`data/csv/*.csv`.  It is written to be run on the industrial PC, where the bags and the
deployed sources live.  See results_numbers.md §9.6 for what exactly is missing.

What it does, per recording and per threshold:
  * cross-track test  -- guard_watch.LatSentinel(win=3.0, thresh=T, persist=1.0,
                         min_disp=0.5, max_dhdg=5.0, hold_s=5.0), fed exactly as
                         the guard replay script of the 2026-09-09 orchard session (not distributed) feeds it:
                             /rtk_odom            -> feed_rtk(t, x, y)
                             /rtk/status          -> feed_hdg(t, HDT) while the solution is fixed
                             /smoother_cmd_vel,
                             /nav_cmd_vel         -> mark_auto(t)   (auto-mode gating)
                         stepped at 10 Hz, T swept over 0.05 / 0.10 / 0.15 / 0.20 / 0.30 m
  * chord test        -- guard_watch.DispSentinel(win=3.0, thresh=T, hold_s=5.0):
                             /rtk_odom            -> feed_rtk
                             /odom  (wheel)       -> feed_odo
                             /rtk/status          -> set_fix
                         stepped at 10 Hz, T swept over 0.10 / 0.20 / 0.30 / 0.50 m

The judgement is never re-implemented: both classes are imported from the deployed source
(`$GUARD_SRC/guard_watch.py`), exactly as the deployed replay tool does.

Scoring, written to the output CSV, one row per (test, threshold):
  alarms                 alarm onsets over the operational corpus
  evaluable_h            hours the criterion was actually evaluable (not gated)
  false_alarms_per_h     alarms that do NOT fall within +/- 5 s of a pseudo-step onset,
                         divided by evaluable_h   (these are alarms on genuine frames)
  onsets_detected        pseudo-step onsets (>= 0.10 m, from pseudo_steps.csv) with an
                         alarm onset inside +/- 5 s
  onsets_total

Usage on the industrial PC:
    export GUARD_SRC=$HOME/fusion/safety          # or wherever the deployed copies are
    python3 roc_position_tests.py --bags baglist.txt \\
            --onsets pseudo_steps.csv --out position_roc.csv
where baglist.txt has one bag path per line (the 46 operational recordings; the
2026-09-06 15:26 induction session must NOT be in it).
"""
import argparse
import csv
import importlib.util
import os
import sys

LAT_T = (0.05, 0.10, 0.15, 0.20, 0.30)      # m, cross-track thresholds to sweep
CHORD_T = (0.10, 0.20, 0.30, 0.50)          # m, chord thresholds to sweep
MATCH_S = 5.0                               # s, an alarm counts for an onset within this
STEP = 0.1                                  # s, the deployed nodes step at 10 Hz


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def onsets_from(path, bags):
    """Pseudo-step onsets >= 0.10 m, grouped at 5 s, restricted to the given recordings."""
    rows = [r for r in csv.DictReader(open(path))
            if float(r["dp_pub"]) >= 0.10 and int(r.get("induced", 0)) == 0]
    by = {}
    for r in rows:
        by.setdefault(r["bag"], []).append(float(r["t"]))
    out = {}
    for b, ts in by.items():
        ts.sort()
        ons = [ts[0]]
        for t in ts[1:]:
            if t - ons[-1] > 5.0:
                ons.append(t)
        out[b] = ons
    return out


def replay(bag, gw, mode, thresh, rtk_topic="/rtk_odom"):
    """Returns (alarm onset times, evaluable seconds)."""
    import rosbag
    if mode == "lat":
        S = gw.LatSentinel(win=3.0, thresh=thresh, persist=1.0, min_disp=0.5,
                           max_dhdg=5.0, hold_s=5.0)
        topics = [rtk_topic, "/rtk/status", "/smoother_cmd_vel", "/nav_cmd_vel"]
    else:
        S = gw.DispSentinel(win=3.0, thresh=thresh, hold_s=5.0)
        topics = [rtk_topic, "/odom", "/rtk/status"]
    msgs = [(t, m, ts.to_sec())
            for t, m, ts in rosbag.Bag(bag).read_messages(topics=topics)]
    if not msgs:
        return [], 0.0
    alarms, evaluable = [], 0.0
    tnext = msgs[0][2]
    for topic, m, tt in msgs:
        while tt >= tnext:
            if mode == "lat":
                al, e, lat = S.step(tnext)
                if not S.gated:
                    evaluable += STEP
            else:
                al, e, dr, do = S.step(tnext)
                if dr is not None and do is not None:
                    evaluable += STEP
            if e == "ALARM":
                alarms.append(tnext)
            tnext += STEP
        if topic == rtk_topic:
            p = m.pose.pose.position
            S.feed_rtk(tt, p.x, p.y)
        elif topic == "/odom":
            p = m.pose.pose.position
            S.feed_odo(tt, p.x, p.y)
        elif topic == "/rtk/status":
            d = gw.dec(m.data)
            if mode == "lat":
                if "固定解(4)" in d:      # "fixed solution (4)" in the driver's recorded log line
                    g = gw.RX_HDG.search(d)
                    if g:
                        S.feed_hdg(tt, float(g.group(1)))
            else:
                S.set_fix("固定解(4)" in d)   # recorded-text literal, see above
        else:
            if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6:
                S.mark_auto(tt)
    return alarms, evaluable


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags", required=True, help="file with one bag path per line")
    ap.add_argument("--onsets", required=True, help="pseudo_steps.csv from the laptop")
    ap.add_argument("--out", default="position_roc.csv")
    ap.add_argument("--src", default=os.environ.get("GUARD_SRC", ""))
    a = ap.parse_args()
    if not a.src:
        sys.exit("set GUARD_SRC (or --src) to the directory holding the deployed "
                 "guard_watch.py")
    gw = load("gw", os.path.join(a.src, "guard_watch.py"))
    bags = [l.strip() for l in open(a.bags) if l.strip() and not l.startswith("#")]
    ons = onsets_from(a.onsets, bags)

    rows = []
    for mode, thresholds in (("lat", LAT_T), ("disp", CHORD_T)):
        for T in thresholds:
            n_alarm = n_false = n_hit = n_ons = 0
            ev_h = 0.0
            for bag in bags:
                key = os.path.basename(bag).replace(".bag", "")
                al, ev = replay(bag, gw, mode, T)
                ev_h += ev / 3600.0
                o = ons.get(key, [])
                n_ons += len(o)
                for t in al:
                    n_alarm += 1
                    if not any(abs(t - x) <= MATCH_S for x in o):
                        n_false += 1
                for x in o:
                    if any(abs(t - x) <= MATCH_S for t in al):
                        n_hit += 1
            rows.append(dict(test=("cross-track" if mode == "lat" else "chord"),
                             threshold_m=T, alarms=n_alarm, false_alarms=n_false,
                             evaluable_h=round(ev_h, 4),
                             false_alarms_per_h=round(n_false / ev_h, 4) if ev_h else "",
                             onsets_detected=n_hit, onsets_total=n_ons))
            print(rows[-1])
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("wrote %s" % a.out)


if __name__ == "__main__":
    main()
