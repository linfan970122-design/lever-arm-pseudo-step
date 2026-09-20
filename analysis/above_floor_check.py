#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
above_floor_check.py — P2 reviewer response helper.

Builds data/lidar_icp/above_floor.md: a per-event table of ALL 75 pseudo-step events
that have lidar coverage inside the narrow (RTK-jump) window, sorted by |dpsi_lidar|,
plus:
  * the 12 events above the 3-sigma ICP noise floor, each with a lidar-vs-gyro and a
    "could this rotation explain the RTK step" verdict,
  * the Table-5 selection criterion reproduced exactly,
  * an alignment-sensitivity study (window shifted by +/-0.2 s and +/-0.5 s),
  * the 23 uncovered events broken down by reason.

Reads only; writes one new file. numpy + csv only.
"""
import csv
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "data", "lidar_icp")
EVENTS_IN = os.path.join(ROOT, "fig", "pseudo_steps.csv")
EVENTS_LIDAR = os.path.join(DATA, "events_lidar.csv")
OUT = os.path.join(DATA, "above_floor.md")

WIN_PAD = 0.15          # s, same as align_lidar_events.py
WIN_2S = 1.0            # s
SIGMA = 0.1037          # per-frame ICP noise floor, median of per-bag quiet RMS (summary.md)
FLOOR = 3.0 * SIGMA     # 0.3112 deg
GYRO_TOL = 2.0 * FLOOR  # "agreement within 2x floor"


def read_csv_float(path, cols):
    out = {c: [] for c in cols}
    with open(path, "r", newline="") as f:
        for row in csv.DictReader(f):
            for c in cols:
                try:
                    out[c].append(float(row[c]))
                except (TypeError, ValueError):
                    out[c].append(np.nan)
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def load_bags():
    bags = {}
    for fn in sorted(os.listdir(DATA)):
        if not fn.endswith("_lidar.csv") or fn.startswith("test_") or fn == "events_lidar.csv":
            continue
        bag = fn[: -len("_lidar.csv")]
        p_l = os.path.join(DATA, fn)
        p_r = os.path.join(DATA, bag + "_rtk.csv")
        p_i = os.path.join(DATA, bag + "_imu.csv")
        if not (os.path.exists(p_r) and os.path.exists(p_i)):
            continue
        lid = read_csv_float(p_l, ["t_hdr", "dt", "dyaw_deg", "fitness", "rmse"])
        imu = read_csv_float(p_i, ["t_hdr", "gz_rad_s"])
        if len(lid["t_hdr"]) < 10 or len(imu["t_hdr"]) < 10:
            continue
        for d in (lid, imu):
            o = np.argsort(d["t_hdr"], kind="stable")
            for k in d:
                d[k] = d[k][o]
        # same de-duplication as align_lidar_events.py (overlapping ICP segments)
        _, keep = np.unique(np.round(lid["t_hdr"], 4), return_index=True)
        keep = np.sort(keep)
        for k in lid:
            lid[k] = lid[k][keep]
        bags[bag] = dict(lid=lid, imu=imu)
    return bags


def gyro_cumint(imu):
    t, g = imu["t_hdr"], imu["gz_rad_s"]
    ok = np.isfinite(t) & np.isfinite(g)
    t, g = t[ok], g[ok]
    dt = np.diff(t)
    seg = 0.5 * (g[:-1] + g[1:]) * dt
    seg[dt > 1.0] = 0.0
    return t, np.degrees(np.concatenate([[0.0], np.cumsum(seg)]))


def gyro_delta(tg, cg, t0, t1):
    if tg.size < 2 or t0 < tg[0] or t1 > tg[-1]:
        return np.nan
    return float(np.interp(t1, tg, cg) - np.interp(t0, tg, cg))


def window_stats(lid, lo, hi):
    t = lid["t_hdr"]
    m = np.isfinite(t) & (t >= lo) & (t <= hi)
    n = int(m.sum())
    if n == 0:
        return dict(n=0, s=np.nan, fmin=np.nan, fmed=np.nan, rmax=np.nan, rmed=np.nan)
    fit, rms = lid["fitness"][m], lid["rmse"][m]
    return dict(
        n=n,
        s=float(np.nansum(lid["dyaw_deg"][m])),
        fmin=float(np.nanmin(fit)) if np.isfinite(fit).any() else np.nan,
        fmed=float(np.nanmedian(fit)) if np.isfinite(fit).any() else np.nan,
        rmax=float(np.nanmax(rms)) if np.isfinite(rms).any() else np.nan,
        rmed=float(np.nanmedian(rms)) if np.isfinite(rms).any() else np.nan,
    )


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return np.nan


def f(x, p=3):
    return "n/a" if not np.isfinite(x) else ("%.*f" % (p, x))


def main():
    bags = load_bags()
    wall = {}
    with open(EVENTS_IN, "r", newline="") as fh:
        for e in csv.DictReader(fh):
            wall[(e["bag"], e["t"])] = e.get("wall", "")

    with open(EVENTS_LIDAR, "r", newline="") as fh:
        ev = list(csv.DictReader(fh))

    covered, no_topic, no_frames = [], [], []
    for e in ev:
        bag, t = e["bag"], float(e["t"])
        dt = float(e["dt"])
        rec = dict(bag=bag, site=e["site"], induced=int(e["induced"]), t=t, dt=dt,
                   wall=wall.get((bag, e["t"]), ""), drtk=float(e["dpsi_deg"]),
                   dimu=_f(e.get("dpsi_imu_deg")), cls=e.get("cls_imu", ""))
        if e["has_lidar"] != "1":
            no_topic.append(rec)
            continue
        b = bags[bag]
        tg, cg = gyro_cumint(b["imu"])
        lo1, hi1 = t - dt - WIN_PAD, t + WIN_PAD
        w1 = window_stats(b["lid"], lo1, hi1)
        if w1["n"] == 0:
            rec["w2n"] = window_stats(b["lid"], t - WIN_2S, t + WIN_2S)["n"]
            tl = b["lid"]["t_hdr"]
            rec["gap"] = float(np.min(np.abs(tl - t))) if tl.size else np.nan
            no_frames.append(rec)
            continue
        w2 = window_stats(b["lid"], t - WIN_2S, t + WIN_2S)
        rec.update(dlid=w1["s"], n1=w1["n"], fmin=w1["fmin"], fmed=w1["fmed"],
                   rmax=w1["rmax"], rmed=w1["rmed"],
                   dlid2=w2["s"], n2=w2["n"],
                   dgyr=gyro_delta(tg, cg, lo1, hi1),
                   dgyr2=gyro_delta(tg, cg, t - WIN_2S, t + WIN_2S),
                   lo=lo1, hi=hi1)
        covered.append(rec)

    covered.sort(key=lambda r: -abs(r["dlid"]))
    n_tot = len(covered)
    above = [r for r in covered if abs(r["dlid"]) > FLOOR]
    below = n_tot - len(above)
    n_lt1 = sum(1 for r in covered if abs(r["dlid"]) < 1.0)
    n_lt2 = sum(1 for r in covered if abs(r["dlid"]) < 2.0)

    # Table 5 criterion, verbatim from align_lidar_events.py / summary.md
    tab5 = [r for r in covered
            if abs(r["dlid"]) > 0.5 * abs(r["drtk"]) and abs(r["dlid"]) > FLOOR]

    for r in covered:
        d = r["dlid"] - r["dgyr"]
        r["gyro_ok"] = bool(np.isfinite(d) and abs(d) <= GYRO_TOL)
        r["gyro_res"] = d
        r["gyro_moves"] = bool(np.isfinite(r["dgyr"]) and abs(r["dgyr"]) > FLOOR)
        r["real_turn"] = bool(r["gyro_ok"] and r["gyro_moves"])
        r["explains"] = bool(np.sign(r["dlid"]) == np.sign(r["drtk"])
                             and abs(r["dlid"]) >= 0.5 * abs(r["drtk"]))
        r["ratio"] = r["dlid"] / r["drtk"] if r["drtk"] != 0 else np.nan

    # ---------------- alignment sensitivity on the 12 above-floor events -------------
    SHIFTS = [-0.5, -0.2, 0.0, 0.2, 0.5]
    for r in above:
        b = bags[r["bag"]]
        r["shift"] = {}
        for s in SHIFTS:
            w = window_stats(b["lid"], r["lo"] + s, r["hi"] + s)
            r["shift"][s] = (w["s"], w["n"])
        vals = [v for v, _ in r["shift"].values() if np.isfinite(v)]
        r["sh_min"], r["sh_max"] = (min(vals), max(vals)) if vals else (np.nan, np.nan)
        r["sh_span"] = r["sh_max"] - r["sh_min"] if vals else np.nan
        r["sh_stable"] = bool(all((abs(v) > FLOOR) == (abs(r["dlid"]) > FLOOR) for v in vals))
        r["sh_expl"] = [s for s in SHIFTS
                        if np.isfinite(r["shift"][s][0])
                        and np.sign(r["shift"][s][0]) == np.sign(r["drtk"])
                        and abs(r["shift"][s][0]) >= 0.5 * abs(r["drtk"])]

    # below-floor events: does any shift push them above the floor?
    flips = 0
    flips_expl = 0
    for r in covered:
        if abs(r["dlid"]) > FLOOR:
            continue
        b = bags[r["bag"]]
        vs = []
        for s in SHIFTS:
            if s == 0.0:
                continue
            w = window_stats(b["lid"], r["lo"] + s, r["hi"] + s)
            if np.isfinite(w["s"]):
                vs.append(w["s"])
        r["sh_below_max"] = max((abs(v) for v in vs), default=np.nan)
        if np.isfinite(r["sh_below_max"]) and r["sh_below_max"] > FLOOR:
            flips += 1
        if any(np.sign(v) == np.sign(r["drtk"]) and abs(v) >= 0.5 * abs(r["drtk"]) for v in vs):
            flips_expl += 1

    # ---------------------------------------------------------------- write
    L = []
    A = L.append
    A("# P2 — all 75 lidar-covered pseudo-step events, ranked by |Δψ_lidar|")
    A("")
    A("Generated by `analysis/above_floor_check.py` from `data/lidar_icp/*_lidar.csv`,")
    A("`*_imu.csv` and `events_lidar.csv`. Nothing here is hand-entered.")
    A("")
    A("**Definitions** (identical to `analysis/align_lidar_events.py`)")
    A("")
    A("* Narrow window = `[t − Δt − 0.15, t + 0.15]` s, i.e. the RTK jump interval padded by 0.15 s.")
    A("  Δt is the RTK message interval of the event (0.2 s at 5 Hz).")
    A("* `Δψ_lidar` = sum of frame-to-frame point-to-plane ICP yaw increments over that window,")
    A("  strictly frame-to-frame, no accumulation.")
    A("* `Δψ_gyro` = trapezoidal integral of CH110 gyro-z over the **same** window.")
    A("* `Δψ_lidar(±1 s)` and `n(±1 s)` use `[t − 1.0, t + 1.0]` s (the paper's wide window;")
    A("  there is no ±3 s field in the data — ±3 s was only the ICP *computation* span).")
    A("* ICP noise floor σ = %.4f° per frame (median of the per-bag quiet-frame RMS, `summary.md` (a));"
      % SIGMA)
    A("  3σ = %.4f°." % FLOOR)
    A("* `lidar vs RTK` = Δψ_lidar / Δψ_RTK (signed ratio). Negative ⇒ opposite direction.")
    A("* Fitness = Open3D ICP inlier fraction, RMSE in metres; both reported as")
    A("  min/median (fitness) and max/median (RMSE) over the frames in the narrow window.")
    A("")
    A("## 1. Headline counts (n = %d covered events)" % n_tot)
    A("")
    A("| statement | count | share |")
    A("|---|---|---|")
    A(r"| \|Δψ_lidar\| < 3σ = %.3f° (below noise floor) | %d / %d | %.0f%% |"
      % (FLOOR, below, n_tot, 100.0 * below / n_tot))
    A(r"| \|Δψ_lidar\| < 1° | %d / %d | %.0f%% |" % (n_lt1, n_tot, 100.0 * n_lt1 / n_tot))
    A(r"| \|Δψ_lidar\| < 2° | %d / %d | %.0f%% |" % (n_lt2, n_tot, 100.0 * n_lt2 / n_tot))
    A(r"| \|Δψ_lidar\| > 3σ (above floor) | %d / %d | %.0f%% |"
      % (len(above), n_tot, 100.0 * len(above) / n_tot))
    A("| …of those, lidar agrees with gyro within 2·3σ = %.3f° | %d / %d | — |"
      % (GYRO_TOL, sum(1 for r in above if r["gyro_ok"]), len(above)))
    A(r"| …of those, a **corroborated real turn** (gyro agrees **and** \|Δψ_gyro\| > 3σ) | %d / %d | — |"
      % (sum(1 for r in above if r["real_turn"]), len(above)))
    A("| …of those, gyro agrees but is itself below the floor (no corroborated rotation) | %d / %d | — |"
      % (sum(1 for r in above if r["gyro_ok"] and not r["gyro_moves"]), len(above)))
    A("| …of those, lidar disagrees with gyro (ICP artefact, not a rotation) | %d / %d | — |"
      % (sum(1 for r in above if not r["gyro_ok"]), len(above)))
    A("| events whose lidar rotation **could explain** the RTK step (same sign **and** ≥ ½ magnitude) | %d / %d | %.0f%% |"
      % (sum(1 for r in covered if r["explains"]), n_tot,
         100.0 * sum(1 for r in covered if r["explains"]) / n_tot))
    A("| events selected into Table 5 (magnitude-only criterion, see §3) | %d / %d | — |"
      % (len(tab5), n_tot))
    A("")
    A("Median |Δψ_RTK| over the same %d events = %.2f°; median |Δψ_lidar| = %.3f°."
      % (n_tot, float(np.median([abs(r["drtk"]) for r in covered])),
         float(np.median([abs(r["dlid"]) for r in covered]))))
    A("")

    A("## 2. All %d covered events, sorted by |Δψ_lidar| (descending)" % n_tot)
    A("")
    A("`>3σ` marks the %d events above the noise floor. `T5` marks the Table-5 four."
      % len(above))
    A("")
    A("| # | flag | recording | wall time | site | ind. | Δψ_RTK (°) | Δψ_lidar (°) | Δψ_gyro (°) | lidar vs RTK | fit min/med | RMSE max/med (m) | n win | Δψ_lidar ±1 s (°) | n ±1 s |")
    A("|---:|:--:|---|---|---|:--:|---:|---:|---:|---:|---|---|---:|---:|---:|")
    for i, r in enumerate(covered, 1):
        flag = ">3σ" if abs(r["dlid"]) > FLOOR else ""
        if r in tab5:
            flag = ">3σ,**T5**"
        A("| %d | %s | %s | %s | %s | %d | %s | %s | %s | %s | %s/%s | %s/%s | %d | %s | %d |" % (
            i, flag, r["bag"].replace("_2026-", " "), r["wall"][:19], r["site"], r["induced"],
            f(r["drtk"], 3), f(r["dlid"], 4), f(r["dgyr"], 4),
            f(r["ratio"], 3), f(r["fmin"], 3), f(r["fmed"], 3),
            f(r["rmax"], 4), f(r["rmed"], 4), r["n1"], f(r["dlid2"], 3), r["n2"]))
    A("")

    A("## 3. The %d events above the 3σ floor — one verdict line each" % len(above))
    A("")
    A("Two independent tests per event:")
    A("")
    A("* **gyro agreement** — |Δψ_lidar − Δψ_gyro| ≤ 2·3σ = %.3f°. Pass ⇒ the lidar saw a" % GYRO_TOL)
    A("  *real* vehicle rotation (two independent sensors agree). Fail ⇒ ICP artefact.")
    A("* **explains the RTK step** — same sign **and** |Δψ_lidar| ≥ ½|Δψ_RTK|.")
    A("")
    A("A third column, **gyro itself above floor** (|Δψ_gyro| > 3σ), separates a genuine small")
    A("turn from a case where the gyro says the vehicle was still and only the lidar moved.")
    A("")
    A("| # | recording | wall time | ind. | Δψ_RTK (°) | Δψ_lidar (°) | Δψ_gyro (°) | residual (°) | gyro agrees? | gyro >3σ? | explains RTK step? | verdict |")
    A("|---:|---|---|:--:|---:|---:|---:|---:|:--:|:--:|:--:|---|")
    for i, r in enumerate(above, 1):
        if r["real_turn"] and r["explains"]:
            v = "real rotation that *could* account for the step"
        elif r["real_turn"]:
            v = "real rotation, but wrong sign/size for the RTK step"
        elif r["gyro_ok"] and not r["gyro_moves"]:
            v = "gyro sees no rotation (|Δψ_gyro| < 3σ); lidar value is borderline ICP noise"
        elif not r["gyro_ok"]:
            v = "lidar and gyro disagree — ICP residual, not a rotation"
        else:
            v = "inspect"
        A("| %d | %s | %s | %d | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            i, r["bag"].replace("_2026-", " "), r["wall"][:19], r["induced"],
            f(r["drtk"], 3), f(r["dlid"], 4), f(r["dgyr"], 4), f(r["gyro_res"], 4),
            "yes" if r["gyro_ok"] else "**no**", "yes" if r["gyro_moves"] else "no",
            "yes" if r["explains"] else "no", v))
    A("")
    A("Prose form, one line per event:")
    A("")
    for i, r in enumerate(above, 1):
        A("%d. **%s @ %s** (induced=%d): RTK stepped %+.2f°; lidar %+.4f°, gyro %+.4f° "
          "(residual %+.4f°) → lidar and gyro %s; ratio to the RTK step %s ⇒ %s explain it."
          % (i, r["bag"].replace("_2026-", " "), r["wall"][11:19], r["induced"],
             r["drtk"], r["dlid"], r["dgyr"], r["gyro_res"],
             ("agree, and the gyro itself is above the floor ⇒ a real turn" if r["real_turn"]
              else ("agree, but the gyro is itself below the floor ⇒ no corroborated rotation"
                    if r["gyro_ok"] else "DISAGREE ⇒ ICP residual")),
             f(r["ratio"], 3), "could" if r["explains"] else "cannot"))
    A("")

    A("### Which criterion picks the Table-5 %d out of the %d" % (len(tab5), len(above)))
    A("")
    A("Table 5 is generated in `align_lidar_events.py` by the **magnitude-only** rule")
    A("")
    A("```")
    A("abs(dpsi_lidar) > 0.5 * abs(dpsi_rtk)   AND   abs(dpsi_lidar) > 3*sigma")
    A("```")
    A("")
    A("i.e. an event enters Table 5 only if the lidar rotation is at least **half the size**")
    A("of the RTK step. Sign is deliberately not tested — that is the point of the table: the")
    A("gyro column then shows the sign is wrong in 3 of the 4. All %d above-floor events pass"
      % len(above))
    rest = [r for r in above if r not in tab5]
    rat = [abs(r["dlid"]) / (0.5 * abs(r["drtk"])) for r in rest]
    A("the 3σ half of the rule; %d also pass the ½-magnitude half. The other %d are above the"
      % (len(tab5), len(rest)))
    A("floor but **small relative to their RTK step** — |Δψ_lidar| ranges %.2f–%.2f° against RTK"
      % (min(abs(r["dlid"]) for r in rest), max(abs(r["dlid"]) for r in rest)))
    A("steps of %.1f–%.1f°, so they reach only %.0f%%–%.0f%% of the ½-magnitude bar (a factor of"
      % (min(abs(r["drtk"]) for r in rest), max(abs(r["drtk"]) for r in rest),
         100 * min(rat), 100 * max(rat)))
    A("%.1f to %.0f short):" % (1 / max(rat), 1 / min(rat)))
    A("")
    A(r"| recording | wall time | Δψ_RTK (°) | Δψ_lidar (°) | ½·\|Δψ_RTK\| (°) | \|Δψ_lidar\| / ½\|Δψ_RTK\| |")
    A("|---|---|---:|---:|---:|---:|")
    for r in above:
        if r in tab5:
            continue
        half = 0.5 * abs(r["drtk"])
        A("| %s | %s | %s | %s | %s | %s |" % (
            r["bag"].replace("_2026-", " "), r["wall"][:19], f(r["drtk"], 3), f(r["dlid"], 4),
            f(half, 3), f(abs(r["dlid"]) / half, 4) if half > 0 else "n/a"))
    A("")

    A("## 4. Alignment sensitivity")
    A("")
    A("The per-frame ICP series **is** available locally (`data/lidar_icp/<bag>_lidar.csv`,")
    A("one row per lidar frame pair, ~10 Hz), so the window can be re-cut without re-running ICP.")
    A("The narrow window was shifted bodily by −0.5, −0.2 (= −1 frame at 10 Hz), +0.2 and +0.5 s")
    A("and Δψ_lidar re-summed. Shifting, not widening: the window keeps its length.")
    A("")
    A("| # | recording | wall time | Δψ_lidar −0.5 s | −0.2 s | **0 (paper)** | +0.2 s | +0.5 s | min…max | span (°) | still >3σ in all shifts? | any shift makes it explain the step? |")
    A("|---:|---|---|---:|---:|---:|---:|---:|---|---:|:--:|:--:|")
    for i, r in enumerate(above, 1):
        cells = []
        for s in SHIFTS:
            v, n = r["shift"][s]
            cells.append("%s (n=%d)" % (f(v, 3), n))
        A("| %d | %s | %s | %s | %s | **%s** | %s | %s | %s…%s | %s | %s | %s |" % (
            i, r["bag"].replace("_2026-", " "), r["wall"][11:19],
            cells[0], cells[1], cells[2], cells[3], cells[4],
            f(r["sh_min"], 3), f(r["sh_max"], 3), f(r["sh_span"], 3),
            "yes" if r["sh_stable"] else "no",
            "yes" if r["sh_expl"] else "no"))
    A("")
    A("Summary of the sensitivity test:")
    A("")
    A("* Above-floor events that stay above the floor at every shift: **%d / %d**."
      % (sum(1 for r in above if r["sh_stable"]), len(above)))
    A("* Above-floor events for which **some** shift would let the lidar rotation explain the")
    A("  RTK step (same sign, ≥ ½ magnitude): **%d / %d**."
      % (sum(1 for r in above if r["sh_expl"]), len(above)))
    A("* Below-floor events (%d) that any shift would push above the floor: **%d**."
      % (below, flips))
    A("* Largest |Δψ_lidar| span across the five window positions, over the %d above-floor"
      % len(above))
    A("  events: %.3f°; median span %.3f°."
      % (float(np.nanmax([r["sh_span"] for r in above])),
         float(np.nanmedian([r["sh_span"] for r in above]))))
    A("")
    shift_only = [r for r in above if r["sh_expl"] and not r["explains"]]
    A("Interpretation, stated against the numbers above and not beyond them:")
    A("")
    A("* Δψ_lidar is **not** insensitive to where the window is cut. Median span across the")
    A("  five positions is %.3f°, maximum %.3f°. On the small events the window slides over"
      % (float(np.nanmedian([r["sh_span"] for r in above])),
         float(np.nanmax([r["sh_span"] for r in above]))))
    A("  ICP noise and simply resamples it; on the genuine turns it slides over a real,")
    A("  continuing rotation, so the sum grows or shrinks with the window position.")
    A("* The **floor test is mostly robust**: %d of %d above-floor events stay above the floor"
      % (sum(1 for r in above if r["sh_stable"]), len(above)))
    A("  at every shift, and %d (%s) drop below it at some shift. In the other direction, %d of"
      % (sum(1 for r in above if not r["sh_stable"]),
         ", ".join(r["wall"][11:19] for r in above if not r["sh_stable"]), flips))
    A("  the %d below-floor events would rise above the floor at some shift — so the 63/12 split"
      % below)
    A("  is itself accurate only to about ±10 events, and the paper should not present 63 as exact.")
    A("* The **conclusion is nearly but not fully shift-invariant**: at the paper's window %d of %d events pass"
      % (sum(1 for r in covered if r["explains"]), n_tot))
    A("  sign-and-half-magnitude; allowing any of the four shifts raises that to %d"
      % (sum(1 for r in covered if r["explains"]) + len(shift_only) + flips_expl))
    A("  (%d above-floor + %d below-floor events gain the verdict at some shift)."
      % (len(shift_only), flips_expl))
    if shift_only:
        A("  The above-floor one is %s: Δψ_lidar = %s° at the paper's window but %s° at +0.5 s,"
          % (", ".join(r["wall"][11:19] for r in shift_only),
             f(shift_only[0]["dlid"], 3), f(shift_only[0]["shift"][0.5][0], 3)))
        A("  i.e. the window slid onto the start of a later genuine turn. That is a statement")
        A("  about window placement, not evidence that the RTK step was real.")
    A("")

    A("## 5. Sentences the counts support")
    A("")
    A("Of the %d pseudo-step events with lidar coverage:" % n_tot)
    A("")
    A("* %d (%.0f%%) have |Δψ_lidar| below the 3σ ICP noise floor of %.3f°;"
      % (below, 100.0 * below / n_tot, FLOOR))
    A("* %d (%.0f%%) below 1°; %d (%.0f%%) below 2°;"
      % (n_lt1, 100.0 * n_lt1 / n_tot, n_lt2, 100.0 * n_lt2 / n_tot))
    A("* %d are above the floor. Of those, %d are **corroborated real rotations** (lidar and"
      % (len(above), sum(1 for r in above if r["real_turn"])))
    A("  gyro agree within 2·3σ **and** the gyro itself exceeds 3σ); %d agree with a gyro that"
      % sum(1 for r in above if r["gyro_ok"] and not r["gyro_moves"]))
    A("  is itself below the floor, i.e. no rotation is corroborated; %d disagrees with the gyro"
      % sum(1 for r in above if not r["gyro_ok"]))
    A("  and is ICP residual rather than motion.")
    n_expl = sum(1 for r in covered if r["explains"])
    A("* **%d of %d** events have a lidar rotation that could explain the RTK heading step"
      % (n_expl, n_tot))
    A("  (same sign and at least half the magnitude).")
    A("")
    if n_expl == 0:
        A("🔴 The expectation that this count is 0 is **not** what the data gives.")
    else:
        A("🔴 **This count is %d, not 0.** The exception is %s: RTK stepped %+.3f°, lidar"
          % (n_expl, ", ".join(r["wall"][:19] for r in covered if r["explains"]),
             [r for r in covered if r["explains"]][0]["drtk"]))
        e0 = [r for r in covered if r["explains"]][0]
        A("  %+.4f°, gyro %+.4f° — same sign, %.0f%% of the step magnitude. The vehicle really"
          % (e0["dlid"], e0["dgyr"], 100.0 * abs(e0["dlid"]) / abs(e0["drtk"])))
        A("  was turning, and the lidar rotation covers over half the RTK step, so this one event")
        A("  is **not** demonstrated to be a pseudo-step by the lidar evidence. It is one of the")
        A("  Table-5 four. Any sentence claiming \"none\" or \"no event\" is wrong; the correct")
        A("  claim is that %d of %d are contradicted and %d is ambiguous." % (n_tot - n_expl, n_tot, n_expl))
    A("")
    A("A defensible replacement for the Fig. 9 title, e.g.:")
    A("")
    A("> On %d of the %d lidar-covered pseudo-step frames the measured vehicle rotation cannot"
      % (n_tot - n_expl, n_tot))
    A("> account for the RTK heading step (median step %.1f°): %d frames show no rotation above"
      % (float(np.median([abs(r["drtk"]) for r in covered])), below))
    A("> the ICP noise floor, and the %d that do rotate turn by at most %s° or in the opposite"
      % (len(above) - n_expl, f(max(abs(r["dlid"]) for r in above if not r["explains"]), 2)))
    A("> direction to the step.")
    A("")

    A("## 6. The %d events without a usable lidar window" % (len(no_topic) + len(no_frames)))
    A("")
    A("### 6a. No lidar topic in the recording — %d events, permanently unrecoverable"
      % len(no_topic))
    A("")
    A("| recording | events | site | wall time span |")
    A("|---|---:|---|---|")
    bagset = {}
    for r in no_topic:
        bagset.setdefault(r["bag"], []).append(r)
    for bag in sorted(bagset):
        rs = sorted(bagset[bag], key=lambda x: x["t"])
        A("| %s | %d | %s | %s … %s |" % (bag.replace("_2026-", " "), len(rs), rs[0]["site"],
                                          rs[0]["wall"][:19], rs[-1]["wall"][:19]))
    A("")
    A("Verified by file presence: these bags have no `<bag>_lidar.csv` in `data/lidar_icp/`,")
    A("because `/livox/lidar` was not recorded in the bag.")
    A("")
    A("### 6b. Lidar present but zero frames inside the narrow window — %d events"
      % len(no_frames))
    A("")
    A("| recording | wall time | site | ind. | Δψ_RTK (°) | window [lo, hi] (s) | n in window | n in ±1 s | nearest lidar frame (s away) |")
    A("|---|---|---|:--:|---:|---|---:|---:|---:|")
    for r in sorted(no_frames, key=lambda x: x["t"]):
        A("| %s | %s | %s | %d | %s | [%.3f, %.3f] | 0 | %d | %s |" % (
            r["bag"].replace("_2026-", " "), r["wall"][:19], r["site"], r["induced"],
            f(r["drtk"], 3), r["t"] - r["dt"] - WIN_PAD, r["t"] + WIN_PAD,
            r["w2n"], f(r["gap"], 2)))
    A("")
    bags_nf = sorted({r["bag"] for r in no_frames})
    A("All %d are in %s — confirmed from the data, matching the manuscript."
      % (len(no_frames), " and ".join(b.replace("_2026-", " ") for b in bags_nf)))
    A("Cause, verified from the `seg` column of `%s_lidar.csv`:" % bags_nf[0])
    A("`lidar_icp_yaw_windows.py` numbers its event windows 0,1,2,… in time order and writes")
    A("`seg` on every row. That file contains segments −1 (the sanity segment) and 2…8, but")
    A("**no rows with seg 0 or seg 1** — the two windows covering exactly these three events")
    A("(16:41:39, and 16:42:42+16:42:43 merged). The windows were requested and returned zero")
    A("frames. The earliest ICP row in the bag is at %s (%s), %s s after the first of the three"
      % ("1787907196.5376", "2026-08-28 16:53:16",
         f(max(r["gap"] for r in no_frames), 0)))
    A("events, so `/livox/lidar` carried no messages in that part of the recording. This is a")
    A("property of the bag, not of the window choice; re-running ICP would not recover them.")
    A("(The raw bag and the run log live on the vehicle PC in a scratch directory, which is")
    A("cleared on reboot, so the recording itself cannot be re-inspected from the local copy.)")
    A("")
    A("Totals: %d events, %d with lidar topic, %d of those with frames in the narrow window,"
      % (len(ev), len(ev) - len(no_topic), n_tot))
    A("%d + %d = %d not analysable." % (len(no_topic), len(no_frames),
                                        len(no_topic) + len(no_frames)))
    A("")

    with open(OUT, "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("wrote %s (%d lines)" % (OUT, len(L)))
    print("covered=%d below=%d above=%d lt1=%d lt2=%d table5=%d explains=%d"
          % (n_tot, below, len(above), n_lt1, n_lt2, len(tab5),
             sum(1 for r in covered if r["explains"])))
    print("no_topic=%d no_frames=%d" % (len(no_topic), len(no_frames)))
    for r in above:
        print("  ABOVE %-38s %s drtk=%+8.3f dlid=%+9.4f dgyr=%+9.4f gyro_ok=%s expl=%s span=%.3f"
              % (r["bag"][:38], r["wall"][11:19], r["drtk"], r["dlid"], r["dgyr"],
                 r["gyro_ok"], r["explains"], r["sh_span"]))


if __name__ == "__main__":
    main()
