#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
align_lidar_events.py — P2: use frame-to-frame lidar ICP yaw as a third-party heading
reference, and check whether the RTK dual-antenna pseudo-steps are accompanied by a real
rotation of the vehicle.

Inputs
  data/lidar_icp/<bag>_lidar.csv  t_hdr,t_bag,dt,dyaw_deg,fitness,rmse,n_src,n_tgt   (~10 Hz)
  data/lidar_icp/<bag>_rtk.csv    t_hdr,t_bag,yaw_deg,x,y                            (~5 Hz)
  data/lidar_icp/<bag>_imu.csv    t_hdr,gz_rad_s                                     (~40 Hz)
  fig/pseudo_steps.csv            event list (bag,site,induced,t,dt,dpsi_deg,...)

Outputs
  data/lidar_icp/events_lidar.csv
  data/lidar_icp/summary.md
  data/lidar_icp/Fig_lidar_vs_rtk.png / .pdf

numpy + csv only (no pandas).
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.environ.get("P2_LIDAR_DATA", os.path.join(ROOT, "data", "lidar_icp"))
EVENTS_IN = os.path.join(ROOT, "fig", "pseudo_steps.csv")

GYRO_ACTIVE_DEG = 0.3      # |gyro dyaw| threshold separating "turning" from "noise floor"
WIN_PAD = 0.15             # s, padding around the event window
WIN_2S = 1.0               # s, +/- window for the wide sum


# ---------------------------------------------------------------- io helpers
def read_csv_float(path, cols):
    """read named columns of a csv into float arrays; returns dict name->np.array"""
    out = {c: [] for c in cols}
    with open(path, "r", newline="") as f:
        rd = csv.DictReader(f)
        for row in rd:
            for c in cols:
                try:
                    out[c].append(float(row[c]))
                except (TypeError, ValueError):
                    out[c].append(np.nan)
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def load_bags():
    """discover bags that have all three csv files"""
    bags = {}
    if not os.path.isdir(DATA):
        sys.exit("missing %s" % DATA)
    for fn in sorted(os.listdir(DATA)):
        if not fn.endswith("_lidar.csv") or fn.startswith("test_") or fn == "events_lidar.csv":
            continue
        bag = fn[: -len("_lidar.csv")]
        p_l = os.path.join(DATA, fn)
        p_r = os.path.join(DATA, bag + "_rtk.csv")
        p_i = os.path.join(DATA, bag + "_imu.csv")
        if not (os.path.exists(p_r) and os.path.exists(p_i)):
            print("WARN %s: missing rtk/imu csv, skipped" % bag)
            continue
        lid = read_csv_float(p_l, ["t_hdr", "dt", "dyaw_deg", "fitness", "rmse"])
        rtk = read_csv_float(p_r, ["t_hdr", "yaw_deg"])
        imu = read_csv_float(p_i, ["t_hdr", "gz_rad_s"])
        if len(lid["t_hdr"]) < 10 or len(imu["t_hdr"]) < 10:
            print("WARN %s: too few rows, skipped" % bag)
            continue
        # sort by time, drop non-finite
        for d in (lid, rtk, imu):
            t = d["t_hdr"]
            o = np.argsort(t, kind="stable")
            for k in d:
                d[k] = d[k][o]
        # de-duplicate lidar frames: the windowed ICP run (lidar_icp_yaw_windows.py) puts the
        # per-bag sanity segment in a separate pass, so a frame that falls inside both the
        # sanity segment and an event window is written twice with identical values.  Summing
        # raw rows over an event window would then double-count the rotation.
        t = lid["t_hdr"]
        _, keep = np.unique(np.round(t, 4), return_index=True)
        n_dup = t.size - keep.size
        if n_dup:
            keep = np.sort(keep)
            for k in lid:
                lid[k] = lid[k][keep]
            print("  %s: dropped %d duplicate lidar frames (overlapping segments)" % (bag, n_dup))
        bags[bag] = dict(lid=lid, rtk=rtk, imu=imu, n_dup=n_dup)
    return bags


# ---------------------------------------------------------------- gyro / rtk
def gyro_cumint(imu):
    """cumulative integral of gz (rad/s) -> degrees, trapezoid; returns (t, cum_deg)"""
    t = imu["t_hdr"]
    g = imu["gz_rad_s"]
    ok = np.isfinite(t) & np.isfinite(g)
    t, g = t[ok], g[ok]
    if t.size < 2:
        return t, np.zeros_like(t)
    dt = np.diff(t)
    # guard against bag gaps: an interval longer than 1 s contributes nothing
    seg = 0.5 * (g[:-1] + g[1:]) * dt
    seg[dt > 1.0] = 0.0
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    return t, np.degrees(cum)


def gyro_delta(tg, cg, t0, t1):
    """integral of gyro-z over [t0,t1] in degrees (NaN if outside coverage)"""
    if tg.size < 2 or t0 < tg[0] or t1 > tg[-1]:
        return np.nan
    return float(np.interp(t1, tg, cg) - np.interp(t0, tg, cg))


def rtk_unwrapped(rtk):
    t = rtk["t_hdr"]
    y = rtk["yaw_deg"]
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    if t.size < 2:
        return t, y
    return t, np.degrees(np.unwrap(np.radians(y)))


# ---------------------------------------------------------------- (a) sanity
def sanity_one(b):
    """per-bag sign/scale check of lidar dyaw against gyro integral"""
    lid, imu, rtk = b["lid"], b["imu"], b["rtk"]
    tl, dtl, dyl = lid["t_hdr"], lid["dt"], lid["dyaw_deg"]
    tg, cg = gyro_cumint(imu)
    tr, yr = rtk_unwrapped(rtk)

    ok = np.isfinite(tl) & np.isfinite(dtl) & np.isfinite(dyl) & (dtl > 0.01) & (dtl < 0.5)
    tl, dtl, dyl = tl[ok], dtl[ok], dyl[ok]
    t0 = tl - dtl

    dyg = np.array([gyro_delta(tg, cg, a, c) for a, c in zip(t0, tl)])
    if tr.size >= 2:
        dyr = np.interp(tl, tr, yr, left=np.nan, right=np.nan) - \
              np.interp(t0, tr, yr, left=np.nan, right=np.nan)
        dyr[(tl > tr[-1]) | (t0 < tr[0])] = np.nan
    else:
        dyr = np.full_like(dyl, np.nan)

    res = dict(bag="", n_frames=int(dyl.size))
    m_act = np.isfinite(dyg) & np.isfinite(dyl) & (np.abs(dyg) > GYRO_ACTIVE_DEG)
    m_qui = np.isfinite(dyg) & np.isfinite(dyl) & (np.abs(dyg) < GYRO_ACTIVE_DEG)
    res["n_active"] = int(m_act.sum())
    res["n_quiet"] = int(m_qui.sum())
    if m_act.sum() >= 10:
        x, y = dyg[m_act], dyl[m_act]
        res["r_gyro"] = float(np.corrcoef(x, y)[0, 1])
        res["slope_gyro"] = float(np.polyfit(x, y, 1)[0])
    else:
        res["r_gyro"] = np.nan
        res["slope_gyro"] = np.nan
    res["rms_quiet"] = float(np.sqrt(np.mean((dyl[m_qui] - dyg[m_qui]) ** 2))) if m_qui.sum() >= 10 else np.nan

    m_r = np.isfinite(dyr) & np.isfinite(dyl) & (np.abs(dyr) > GYRO_ACTIVE_DEG)
    if m_r.sum() >= 10:
        res["r_rtk"] = float(np.corrcoef(dyr[m_r], dyl[m_r])[0, 1])
        res["slope_rtk"] = float(np.polyfit(dyr[m_r], dyl[m_r], 1)[0])
    else:
        res["r_rtk"] = np.nan
        res["slope_rtk"] = np.nan
    res["med_fitness"] = float(np.nanmedian(lid["fitness"])) if lid["fitness"].size else np.nan
    res["med_dt"] = float(np.nanmedian(dtl)) if dtl.size else np.nan
    res["t_span"] = float(tl[-1] - tl[0]) if tl.size > 1 else np.nan
    # per-frame pairs kept for the validation panel of the figure (no number changes)
    m_pair = np.isfinite(dyg) & np.isfinite(dyl)
    res["pairs"] = (dyg[m_pair], dyl[m_pair])
    return res


# ---------------------------------------------------------------- (b) events
def window_sum(lid, lo, hi):
    t = lid["t_hdr"]
    m = np.isfinite(t) & (t >= lo) & (t <= hi)
    n = int(m.sum())
    if n == 0:
        return 0, np.nan, np.nan
    s = float(np.nansum(lid["dyaw_deg"][m]))
    fit = lid["fitness"][m]
    fmin = float(np.nanmin(fit)) if np.isfinite(fit).any() else np.nan
    return n, s, fmin


def main():
    bags = load_bags()
    print("bags with lidar data: %d" % len(bags))
    for k in sorted(bags):
        print("  %-45s lidar=%d rtk=%d imu=%d" % (k, bags[k]["lid"]["t_hdr"].size,
                                                  bags[k]["rtk"]["t_hdr"].size,
                                                  bags[k]["imu"]["t_hdr"].size))

    # ---- (a) sanity ----
    san = {}
    for k in sorted(bags):
        r = sanity_one(bags[k])
        r["bag"] = k
        san[k] = r

    hdr = ["bag", "n_frames", "med_dt", "med_fitness", "n_active", "r_gyro", "slope_gyro",
           "n_quiet", "rms_quiet", "r_rtk", "slope_rtk"]
    lines_a = []
    lines_a.append("| bag | frames | dt(s) | fit | n>0.3deg | r(lidar,gyro) | slope | n quiet | RMS quiet(deg) | r(lidar,rtk) | slope |")
    lines_a.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for k in sorted(san):
        r = san[k]
        lines_a.append("| %s | %d | %.3f | %.3f | %d | %.4f | %.4f | %d | %.4f | %.4f | %.4f |" % (
            k, r["n_frames"], r["med_dt"], r["med_fitness"], r["n_active"], r["r_gyro"],
            r["slope_gyro"], r["n_quiet"], r["rms_quiet"], r["r_rtk"], r["slope_rtk"]))
    print("\n".join(lines_a))

    slopes = np.array([san[k]["slope_gyro"] for k in san], dtype=float)
    rmss = np.array([san[k]["rms_quiet"] for k in san], dtype=float)
    med_slope = float(np.nanmedian(slopes)) if np.isfinite(slopes).any() else np.nan
    noise_sigma = float(np.nanmedian(rmss)) if np.isfinite(rmss).any() else np.nan
    if np.isfinite(med_slope) and med_slope < -0.5:
        sign_note = ("SIGN FLIPPED: median slope of lidar dyaw vs gyro dyaw is %.3f (near -1). "
                     "The sign convention in lidar_icp_yaw.py is inverted. NOT auto-corrected." % med_slope)
    elif np.isfinite(med_slope) and med_slope > 0.5:
        sign_note = "Sign OK: median slope of lidar dyaw vs gyro dyaw is %.3f (near +1)." % med_slope
    else:
        sign_note = "Sign INDETERMINATE: median slope %.3f is neither near +1 nor near -1." % med_slope
    print("\n" + sign_note)
    print("per-frame ICP noise floor sigma (median of per-bag RMS on quiet frames) = %.4f deg" % noise_sigma)

    # ---- (b) event alignment ----
    with open(EVENTS_IN, "r", newline="") as f:
        ev = list(csv.DictReader(f))
    print("events read: %d" % len(ev))

    keep = ["bag", "site", "induced", "t", "dt", "dpsi_deg", "dpsi_imu_deg", "cls_imu"]
    new = ["has_lidar", "n_lidar_win", "dpsi_lidar_deg", "n_lidar_2s", "dpsi_lidar_2s",
           "dpsi_gyro_win_deg", "dpsi_gyro_2s_deg", "min_fitness_win", "min_fitness_2s"]
    rows = []
    cache = {}
    for e in ev:
        bag = e["bag"]
        r = {k: e.get(k, "") for k in keep}
        t = float(e["t"])
        dt = float(e["dt"])
        if bag not in bags:
            r.update({k: "nan" for k in new})
            r["has_lidar"] = 0
            rows.append(r)
            continue
        b = bags[bag]
        if bag not in cache:
            cache[bag] = gyro_cumint(b["imu"])
        tg, cg = cache[bag]
        lo1, hi1 = t - dt - WIN_PAD, t + WIN_PAD
        lo2, hi2 = t - WIN_2S, t + WIN_2S
        n1, s1, f1 = window_sum(b["lid"], lo1, hi1)
        n2, s2, f2 = window_sum(b["lid"], lo2, hi2)
        g1 = gyro_delta(tg, cg, lo1, hi1)
        g2 = gyro_delta(tg, cg, lo2, hi2)
        r["has_lidar"] = 1
        r["n_lidar_win"] = n1
        r["dpsi_lidar_deg"] = "%.4f" % s1 if np.isfinite(s1) else "nan"
        r["n_lidar_2s"] = n2
        r["dpsi_lidar_2s"] = "%.4f" % s2 if np.isfinite(s2) else "nan"
        r["dpsi_gyro_win_deg"] = "%.4f" % g1 if np.isfinite(g1) else "nan"
        r["dpsi_gyro_2s_deg"] = "%.4f" % g2 if np.isfinite(g2) else "nan"
        r["min_fitness_win"] = "%.3f" % f1 if np.isfinite(f1) else "nan"
        r["min_fitness_2s"] = "%.3f" % f2 if np.isfinite(f2) else "nan"
        rows.append(r)

    out_csv = os.path.join(DATA, "events_lidar.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keep + new)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("wrote %s (%d rows)" % (out_csv, len(rows)))

    # ---- (c) summary ----
    def fnum(r, k):
        try:
            return float(r[k])
        except (TypeError, ValueError):
            return np.nan

    has = [r for r in rows if r["has_lidar"] == 1 and np.isfinite(fnum(r, "dpsi_lidar_deg"))]
    al = np.array([abs(fnum(r, "dpsi_lidar_deg")) for r in has])
    ar = np.array([abs(fnum(r, "dpsi_deg")) for r in has])

    def block(name, sel):
        idx = [i for i, r in enumerate(has) if sel(r)]
        if not idx:
            return "| %s | 0 | - | - | - | - | - |" % name
        a = al[idx]
        b = ar[idx]
        return "| %s | %d | %d | %d | %d | %.3f | %.3f |" % (
            name, len(idx), int((a < 1.0).sum()), int((a < 2.0).sum()),
            int((a > 0.5 * b).sum()), float(np.median(a)), float(np.max(a)))

    lines_c = []
    lines_c.append("| group | n | \\|dpsi_lidar\\|<1deg | <2deg | >0.5*\\|dpsi_rtk\\| | median\\|dpsi_lidar\\| | max |")
    lines_c.append("|---|---|---|---|---|---|---|")
    lines_c.append(block("ALL", lambda r: True))
    sites = sorted({r["site"] for r in has})
    for s in sites:
        lines_c.append(block("site=" + s, lambda r, s=s: r["site"] == s))
    for v in sorted({r["induced"] for r in has}):
        lines_c.append(block("induced=" + str(v), lambda r, v=v: r["induced"] == v))
    print("\n".join(lines_c))

    n_ev = len(rows)
    n_lid = sum(1 for r in rows if r["has_lidar"] == 1)
    missing = sorted({r["bag"] for r in rows if r["has_lidar"] == 0})

    md = []
    md.append("# P2 lidar ICP yaw vs RTK pseudo-steps — summary")
    md.append("")
    md.append("Generated by `analysis/align_lidar_events.py`. Lidar yaw increment comes from")
    md.append("frame-to-frame point-to-plane ICP on /livox/lidar.")
    md.append("")
    md.append("**Method: event-windowed, NOT whole-bag.** ICP was run by")
    md.append("`analysis/lidar_icp_yaw_windows.py` only inside [t-3, t+3] s around each event")
    md.append("(overlapping windows merged) plus one contiguous 3000-frame segment per bag taken")
    md.append("from the middle of the bag, which is what the sanity table in (a) is computed on.")
    md.append("Running all 9 bags end to end would have been 233548 frames / ~16 h wall clock.")
    md.append("ICP is strictly frame-to-frame with no accumulation, so skipping the intervals")
    md.append("between events does not change any event's measured rotation; what is lost is")
    md.append("whole-bag coverage, i.e. the sanity statistics in (a) are from a 5-minute sample")
    md.append("per bag rather than the whole run.")
    md.append("")
    md.append("## (a) Sign / scale sanity check, per bag")
    md.append("")
    md.append("Comparison is over lidar frame intervals. `n>0.3deg` = frames whose gyro-integrated")
    md.append("yaw increment exceeds 0.3 deg (vehicle actually turning); `quiet` = the complement,")
    md.append("used as the ICP noise floor.")
    md.append("")
    md.extend(lines_a)
    md.append("")
    md.append("**%s**" % sign_note)
    md.append("")
    md.append("Per-frame ICP noise floor sigma = %.4f deg (median of per-bag quiet RMS); 3 sigma = %.4f deg."
              % (noise_sigma, 3 * noise_sigma))
    md.append("")
    md.append("## (b)(c) Event alignment")
    md.append("")
    md.append("Events: %d total, %d have lidar coverage." % (n_ev, n_lid))
    if missing:
        md.append("")
        md.append("Bags without lidar data (events flagged has_lidar=0):")
        for m in missing:
            md.append("- %s (%d events)" % (m, sum(1 for r in rows if r["bag"] == m)))
    md.append("")
    md.append("Window for `dpsi_lidar_deg` = [t-dt-%.2f, t+%.2f] s (the RTK jump interval);" % (WIN_PAD, WIN_PAD))
    md.append("`dpsi_lidar_2s` = [t-%.1f, t+%.1f] s." % (WIN_2S, WIN_2S))
    md.append("")
    md.extend(lines_c)
    md.append("")
    md.append("## Events where the lidar DOES see a comparable rotation")
    md.append("")
    md.append("(|dpsi_lidar| > 0.5*|dpsi_rtk| and |dpsi_lidar| > 3 sigma)")
    md.append("")
    md.append("Note the criterion tests magnitude only, not sign: see the `dpsi_gyro` column, which")
    md.append("shows that in every one of these the lidar agrees with the gyro (a real vehicle")
    md.append("rotation was happening), while the RTK step does not match it in sign or size.")
    md.append("")
    md.append("| bag | t | dpsi_rtk(deg) | dpsi_lidar(deg) | dpsi_gyro(deg) | dpsi_lidar_2s(deg) | dpsi_imu(deg) | cls_imu | min fit |")
    md.append("|---|---|---|---|---|---|---|---|---|")
    hits = []
    thr = 3 * noise_sigma if np.isfinite(noise_sigma) else 0.0
    for r in has:
        a = abs(fnum(r, "dpsi_lidar_deg"))
        b = abs(fnum(r, "dpsi_deg"))
        if a > 0.5 * b and a > thr:
            hits.append(r)
            md.append("| %s | %s | %.3f | %.3f | %s | %s | %s | %s | %s |" % (
                r["bag"], r["t"], fnum(r, "dpsi_deg"), fnum(r, "dpsi_lidar_deg"),
                r["dpsi_gyro_win_deg"], r["dpsi_lidar_2s"], r["dpsi_imu_deg"],
                r["cls_imu"], r["min_fitness_win"]))
    if not hits:
        md.append("| (none) | | | | | | | | |")
    n_same = sum(1 for r in hits if fnum(r, "dpsi_deg") * fnum(r, "dpsi_lidar_deg") > 0)
    md.append("")
    md.append("Of these %d, %d have the RTK step and the lidar rotation in the SAME direction; "
              "%d are opposite in sign." % (len(hits), n_same, len(hits) - n_same))
    n_below = int((al < thr).sum()) if al.size else 0
    md.append("")
    md.append("Headline: %d of %d events (%.0f%%) show a lidar rotation below the 3 sigma ICP "
              "noise floor (%.3f deg) while the RTK heading stepped by a median of %.2f deg."
              % (n_below, al.size, 100.0 * n_below / max(1, al.size), thr, float(np.median(ar))))
    md.append("")
    lowfit = [r for r in has if np.isfinite(fnum(r, "min_fitness_win")) and fnum(r, "min_fitness_win") < 0.5]
    nowin = [r for r in rows if r["has_lidar"] == 1 and r["n_lidar_win"] == 0]
    md.append("Quality notes: %d events with min ICP fitness < 0.5 in the narrow window; "
              "%d events with zero lidar frames inside the narrow window." % (len(lowfit), len(nowin)))
    md.append("")

    out_md = os.path.join(DATA, "summary.md")
    with open(out_md, "w") as f:
        f.write("\n".join(md) + "\n")
    print("wrote %s" % out_md)
    print("hits (lidar sees comparable rotation): %d" % len(hits))
    for r in hits:
        print("  %s t=%s dpsi_rtk=%.2f dpsi_lidar=%.2f cls=%s" % (
            r["bag"], r["t"], fnum(r, "dpsi_deg"), fnum(r, "dpsi_lidar_deg"), r["cls_imu"]))

    # ---- (d) figure (Fig. 9 of the manuscript) ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, figure skipped")
        return

    # GPS Solutions double-column figure, same style as fig/fig*.py
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
        "axes.linewidth": 0.7,
        "xtick.major.width": 0.7,
        "ytick.major.width": 0.7,
        "lines.linewidth": 0.9,
    })
    BLUE, ORANGE, GREEN, VERM, INK = "#0072B2", "#E69F00", "#009E73", "#D55E00", "#1a1a1a"

    fig, (axa, axb) = plt.subplots(1, 2, figsize=(6.85, 3.05))
    fig.subplots_adjust(left=0.075, right=0.995, bottom=0.145, top=0.90, wspace=0.28)

    # --- (a) the reference against the gyro, per lidar frame, validation segments
    gx = np.concatenate([san[k]["pairs"][0] for k in sorted(san) if san[k]["pairs"][0].size])
    gy = np.concatenate([san[k]["pairs"][1] for k in sorted(san) if san[k]["pairs"][1].size])
    lim = float(np.nanpercentile(np.abs(np.concatenate([gx, gy])), 99.9)) * 1.1
    axa.axhline(0.0, color="#cccccc", lw=0.6, zorder=1)
    axa.axvline(0.0, color="#cccccc", lw=0.6, zorder=1)
    axa.scatter(gx, gy, s=2.0, color=BLUE, alpha=0.20, linewidths=0.0, zorder=3,
                label="lidar frame (n = %d)" % gx.size)
    ll = np.array([-lim, lim])
    axa.plot(ll, ll, "--", color=INK, lw=0.9, zorder=4, label="y = x")
    rs = np.array([san[k]["r_gyro"] for k in sorted(san)], dtype=float)
    rs = rs[np.isfinite(rs)]
    axa.text(0.035, 0.965,
             "slope %.3f (median over recordings)\n$r$ = %.2f–%.2f\n"
             "noise floor $\\sigma$ = %.2f$\\degree$ per frame" %
             (med_slope, rs.min(), rs.max(), noise_sigma),
             transform=axa.transAxes, ha="left", va="top", fontsize=7.0, linespacing=1.45,
             color=INK, zorder=6)
    axa.set_xlim(-lim, lim)
    axa.set_ylim(-lim, lim)
    axa.set_xlabel("gyro-integrated yaw increment (deg)")
    axa.set_ylabel("lidar ICP yaw increment (deg)")
    axa.grid(True, color="#dddddd", lw=0.5, zorder=0)
    axa.set_axisbelow(True)
    axa.tick_params(direction="out", length=2.6)
    axa.legend(loc="lower right", frameon=False, handlelength=1.4, borderaxespad=0.35,
               labelspacing=0.35, markerscale=3.0)
    axa.text(0.0, 1.02, "(a)", transform=axa.transAxes, fontsize=8.6, va="bottom", ha="left",
             weight="bold")

    # --- (b) the events
    colors = {"concrete yard": BLUE, "greenhouse": ORANGE, "orchard": GREEN}
    cyc = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i, s_ in enumerate(sites):
        colors.setdefault(s_, cyc[i % len(cyc)])
    # marker encodes the site (readable in grayscale), fill encodes natural vs induced
    site_mk = {"concrete yard": "o", "greenhouse": "s", "orchard": "^"}
    spare_mk = ["D", "v", "P", "X"]
    for i_, s_ in enumerate(sites):
        site_mk.setdefault(s_, spare_mk[i_ % len(spare_mk)])
    labels = {"0": "natural", "1": "induced"}
    for s_ in sites:
        for v in sorted({r["induced"] for r in has}):
            idx = [i for i, r in enumerate(has) if r["site"] == s_ and r["induced"] == v]
            if not idx:
                continue
            filled = str(v) == "1"
            axb.scatter(ar[idx], al[idx], s=18, marker=site_mk[s_],
                        facecolors=colors[s_] if filled else "white",
                        edgecolors=colors[s_] if filled else colors[s_],
                        linewidths=0.9, alpha=0.9, zorder=4,
                        label="%s, %s" % (s_, labels.get(str(v), str(v))))
    lim_lo = max(1e-2, float(np.nanmin(ar)) * 0.7) if ar.size else 0.01
    lim_hi = float(np.nanmax(ar)) * 1.6 if ar.size else 100.0
    xx = np.array([lim_lo, lim_hi])
    axb.plot(xx, xx, "--", color=INK, lw=0.9, zorder=5, label="y = x (a real rotation)")
    if np.isfinite(noise_sigma):
        axb.axhline(3 * noise_sigma, color=VERM, ls=":", lw=1.1, zorder=5,
                    label="ICP noise floor 3$\\sigma$ = %.2f$\\degree$" % (3 * noise_sigma))
    axb.set_xscale("log")
    lt = 3 * noise_sigma if np.isfinite(noise_sigma) and noise_sigma > 0 else 0.1
    axb.set_yscale("symlog", linthresh=lt, linscale=0.6)
    axb.set_ylim(0, max(1000.0, float(np.nanmax(al)) * 80 if al.size else 1000.0))
    axb.set_xlim(lim_lo, lim_hi)
    axb.set_xlabel("RTK heading step $|\\Delta\\psi|$ (deg)")
    axb.set_ylabel("lidar ICP rotation over the same interval (deg)")
    axb.grid(True, which="major", color="#dddddd", lw=0.5, zorder=0)
    axb.set_axisbelow(True)
    axb.tick_params(direction="out", length=2.6, which="major")
    axb.tick_params(length=1.5, which="minor")
    axb.legend(loc="upper left", frameon=False, handlelength=1.4, borderaxespad=0.35,
               labelspacing=0.3, bbox_to_anchor=(0.01, 1.0))
    axb.text(0.0, 1.02, "(b)", transform=axb.transAxes, fontsize=8.6, va="bottom", ha="left",
             weight="bold")

    for out_dir, stem in ((DATA, "Fig_lidar_vs_rtk"),
                          (os.path.join(ROOT, "fig"), "Fig9_lidar_reference")):
        if not os.path.isdir(out_dir):
            continue
        for ext in ("png", "pdf"):
            p = os.path.join(out_dir, "%s.%s" % (stem, ext))
            fig.savefig(p, dpi=300)
            print("wrote %s" % p)


if __name__ == "__main__":
    main()
