#!/usr/bin/env python3
"""M8: metrics at the consumer of the real robot_localization chain.

Reads the per-run CSVs pulled back from the vehicle PC into
data/m8_replay/runs/<tag>/ and writes

    data/m8_replay/per_event.csv     one row per (arm, event)
    data/m8_replay/summary.md        the tables that go into the report
    data/m8_replay/gate_equivalence.csv   C++ gate vs fig/gate.py, frame by frame

Reference for the consumer is the recorded /rtk_odom base_link position, which the
injection never touches; the odom and map frames differ by a constant translation,
removed as the median of (p_rtk - p_gps) over the run.
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, 'data', 'm8_replay', 'runs')
OUTD = os.path.join(ROOT, 'data', 'm8_replay')
sys.path.insert(0, os.path.join(ROOT, 'fig'))

LEVER = (0.235, -0.280)
L_NORM = math.hypot(*LEVER)
L_ANG = math.atan2(LEVER[1], LEVER[0])

# arm tag -> (family, q)  ; families: stock / gate / mahal
ARMS = {}
for q, qs in [('0.06', 'q060'), ('0.001', 'q1e3'), ('0.0001', 'q1e4'), ('0.00001', 'q1e5')]:
    for fam in ('stock', 'gate', 'mahal'):
        ARMS['inj_%s_%s' % (fam, qs)] = (fam, float(q), 'bas_%s_%s' % (fam, qs))


def load(tag, name):
    p = os.path.join(RUNS, tag, name)
    if not os.path.isfile(p) or os.path.getsize(p) < 40:
        return None
    return np.genfromtxt(p, delimiter=',', names=True)


def filt_yaw(tag):
    f = load(tag, 'odom_filtered.csv')
    if f is None:
        return None
    return f['t'], np.unwrap(f['yaw'])


def dev_series(tag):
    """|consumer position - recorded base_link position| at every published fix."""
    g = load(tag, 'odom_gps.csv')
    r = load(tag, 'rtk_ref.csv')
    if g is None or r is None or g['t'].size < 10:
        return None
    rx = np.interp(g['t'], r['t'], r['x'])
    ry = np.interp(g['t'], r['t'], r['y'])
    dx, dy = rx - g['x'], ry - g['y']
    ox, oy = np.median(dx), np.median(dy)
    return dict(t=g['t'], dev=np.hypot(dx - ox, dy - oy), gx=g['x'], gy=g['y'],
                ox=ox, oy=oy, rx=rx, ry=ry)


def used_yaw(tag):
    """Back out the heading the node actually rotated the lever arm with."""
    g = load(tag, 'odom_gps.csv')
    r = load(tag, 'rtk_ref.csv')
    if g is None or r is None:
        return None
    d = dev_series(tag)
    rx, ry = d['rx'], d['ry']
    ryaw = np.interp(g['t'], r['t'], np.unwrap(r['yaw']))
    # antenna point in the map frame
    ax = rx + math.cos(0) * 0  # placeholder, filled below
    ax = rx + (np.cos(ryaw) * LEVER[0] - np.sin(ryaw) * LEVER[1])
    ay = ry + (np.sin(ryaw) * LEVER[0] + np.cos(ryaw) * LEVER[1])
    vx = ax - (g['x'] + d['ox'])
    vy = ay - (g['y'] + d['oy'])
    return g['t'], np.arctan2(vy, vx) - L_ANG, np.hypot(vx, vy)


def missing_stamps(ref_t, t, tol=0.002):
    """Reference fixes with no counterpart in this run, inside the common time span."""
    if ref_t is None or t.size == 0:
        return np.zeros(0)
    lo, hi = max(ref_t[0], t[0]), min(ref_t[-1], t[-1])
    r = ref_t[(ref_t >= lo) & (ref_t <= hi)]
    if r.size == 0:
        return np.zeros(0)
    idx = np.clip(np.searchsorted(t, r), 1, t.size - 1)
    near = np.minimum(np.abs(t[idx] - r), np.abs(t[idx - 1] - r))
    return r[near > tol]


def n_missing(ref_t, t, t0, t1, tol=0.002):
    m = missing_stamps(ref_t, t, tol)
    return int(((m >= t0) & (m < t1)).sum())


def warn_counts(tag):
    p = os.path.join(RUNS, tag, 'warns.csv')
    out = []
    if not os.path.isfile(p):
        return out
    with open(p) as f:
        for row in csv.DictReader(f):
            if row['kind'] in ('hold', 'stale', 'invalid', 'reanchor_gyro', 'reanchor_two'):
                out.append((float(row['t']), row['kind']))
    return out


def load_exclude():
    p = os.path.join(OUTD, 'exclude.csv')
    if not os.path.isfile(p):
        return np.zeros(0)
    return np.array([float(r['t_abs']) for r in csv.DictReader(open(p))])


def main():
    events = list(csv.DictReader(open(os.path.join(OUTD, 'events.csv'))))
    excl = load_exclude()
    ref = dev_series('bas_stock_q060')       # stamp list of a run that never withholds
    ref_t = ref['t'] if ref is not None else None

    rows = []
    for tag, (fam, q, bastag) in sorted(ARMS.items()):
        d = dev_series(tag)
        b = dev_series(bastag)
        if d is None:
            continue
        def keep(tt):
            if excl.size == 0:
                return np.ones(tt.size, bool)
            return np.min(np.abs(tt[:, None] - excl[None, :]), axis=1) > 1.0
        base_med = float(np.median(b['dev'][keep(b['t'])])) if b is not None else float('nan')
        thr = max(2.0 * base_med, 0.02)
        fy = filt_yaw(tag)
        wl = warn_counts(tag)
        wt = np.array([x[0] for x in wl]) if wl else np.zeros(0)

        ev_t = np.array([float(e['t_abs']) for e in events])
        for e in events:
            t0 = float(e['t_abs'])
            m = (d['t'] >= t0 - 1e-6) & (d['t'] < t0 + 2.0)
            pre = d['t'] < t0 - 1e-6
            dev_pre = d['dev'][pre][-1] if pre.any() else float('nan')
            if m.sum() == 0:
                peak = float('nan')
                jump = float('nan')
                rec = float('nan')
            else:
                peak = float(d['dev'][m].max())
                jump = float(d['dev'][m][0] - dev_pre)
                ipk = int(np.argmax(d['dev'][m]))
                tpk = d['t'][m][ipk]
                after = (d['t'] > tpk) & (d['t'] < tpk + 8.0)
                ok = np.where(d['dev'][after] < thr)[0]
                rec = float(d['t'][after][ok[0]] - t0) if ok.size else float('nan')
            pbase = float(b['dev'][(b['t'] >= t0) & (b['t'] < t0 + 2.0)].max()) if b is not None else float('nan')
            nheld = n_missing(ref_t, d['t'], t0, t0 + 5.0)
            # the per-frame yaw increment the gate actually sees: the filter's yaw
            # sampled at the fix stamps, which is what navsat_transform reads from tf
            dstep = float('nan')
            ftot = float('nan')
            if fy is not None and ref_t is not None:
                fs = ref_t[(ref_t >= t0 - 0.4) & (ref_t < t0 + 2.0)]
                if fs.size > 2:
                    yi = np.interp(fs, fy[0], fy[1])
                    dstep = float(np.degrees(np.abs(np.diff(yi))).max())
                    ftot = float(np.degrees(np.abs(yi - yi[0])).max())
            nw = int(((wt >= t0 - 0.2) & (wt < t0 + 5.0)).sum()) if wt.size else 0
            rows.append(dict(arm=tag, family=fam, q=q, event_id=e['event_id'],
                             amp_deg=float(e['amp_deg']), dur=int(e['dur_frames']),
                             t_abs=t0, jump_m=round(jump, 4), peak_m=round(peak, 4),
                             peak_base_m=round(pbase, 4), recovery_s=round(rec, 3) if rec == rec else '',
                             withheld=nheld, gate_warns=nw,
                             filt_step_deg=round(dstep, 3) if dstep == dstep else '',
                             filt_exc_deg=round(ftot, 3) if ftot == ftot else ''))

        # false holds / withholds outside +-3 s of any injection
        def far_of(t):
            return (np.all(np.abs(ev_t - t) > 3.0)
                    and (excl.size == 0 or np.all(np.abs(excl - t) > 1.0)))
        n_far_warn = int(sum(1 for t, _ in wl if far_of(t)))
        miss = missing_stamps(ref_t, d['t'])
        n_far_held = int(sum(1 for t in miss if far_of(t)))
        rows.append(dict(arm=tag, family=fam, q=q, event_id='_RUN_', amp_deg='', dur='',
                         t_abs='', jump_m='', peak_m=round(float(np.median(d['dev'][keep(d['t'])])), 4),
                         peak_base_m=round(base_med, 4),
                         recovery_s=round(float(np.percentile(b['dev'][keep(b['t'])], 99)), 4)
                         if b is not None else '',
                         withheld=n_far_held, gate_warns=n_far_warn,
                         filt_step_deg='', filt_exc_deg=''))

    with open(os.path.join(OUTD, 'per_event.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('wrote per_event.csv (%d rows)' % len(rows))
    return rows


if __name__ == '__main__':
    main()
