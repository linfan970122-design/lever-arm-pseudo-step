#!/usr/bin/env python3
"""M8b: metrics at the consumer for the measurement-side-gate arms.

Same metric definitions and the same reference as analysis/m8/analyze_m8.py (that file is
untouched); the differences are only

  * the runs live in data/m8b_measurement_gate/runs/
  * two families, meas (ws_stock + measurement gate) and both (ws_gate + measurement gate)
  * two extra cost columns taken from the gate node's own log meas_gate.csv:
        meas_nopub  frames whose orientation was withheld from the EKF (cov[0] = -1)
        meas_hold   frames where the EKF was given the last accepted (stale) heading
  * the stamp reference for "withheld consumer frames" is M8's bas_stock_q060 run,
    read read-only from data/m8_replay/runs/.

Writes data/m8b_measurement_gate/per_event.csv (same columns as M8's plus the two above).
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, 'data', 'm8b_measurement_gate', 'runs')
OUTD = os.path.join(ROOT, 'data', 'm8b_measurement_gate')
M8D = os.path.join(ROOT, 'data', 'm8_replay')          # read-only

LEVER = (0.235, -0.280)

ARMS = {}
for q, qs in [('0.06', 'q060'), ('0.001', 'q1e3'), ('0.0001', 'q1e4'), ('0.00001', 'q1e5')]:
    for fam in ('meas', 'both'):
        ARMS['inj_%s_%s' % (fam, qs)] = (fam, float(q), 'bas_%s_%s' % (fam, qs))


def load(tag, name, root=RUNS):
    p = os.path.join(root, tag, name)
    if not os.path.isfile(p) or os.path.getsize(p) < 40:
        return None
    return np.genfromtxt(p, delimiter=',', names=True)


def filt_yaw(tag):
    f = load(tag, 'odom_filtered.csv')
    if f is None:
        return None
    return f['t'], np.unwrap(f['yaw'])


def dev_series(tag, root=RUNS):
    g = load(tag, 'odom_gps.csv', root)
    r = load(tag, 'rtk_ref.csv', root)
    if g is None or r is None or g['t'].size < 10:
        return None
    rx = np.interp(g['t'], r['t'], r['x'])
    ry = np.interp(g['t'], r['t'], r['y'])
    dx, dy = rx - g['x'], ry - g['y']
    ox, oy = np.median(dx), np.median(dy)
    return dict(t=g['t'], dev=np.hypot(dx - ox, dy - oy), gx=g['x'], gy=g['y'],
                ox=ox, oy=oy, rx=rx, ry=ry)


def gate_log(tag):
    """(t, state) from the measurement gate's own per-frame log."""
    p = os.path.join(RUNS, tag, 'meas_gate.csv')
    if not os.path.isfile(p):
        return None
    a = np.genfromtxt(p, delimiter=',', names=True)
    return a['t'], a['state'].astype(int)


def missing_stamps(ref_t, t, tol=0.002):
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
    p = os.path.join(M8D, 'exclude.csv')
    if not os.path.isfile(p):
        return np.zeros(0)
    return np.array([float(r['t_abs']) for r in csv.DictReader(open(p))])


def main():
    events = list(csv.DictReader(open(os.path.join(M8D, 'events.csv'))))
    excl = load_exclude()
    ref = dev_series('bas_stock_q060', root=os.path.join(M8D, 'runs'))
    ref_t = ref['t'] if ref is not None else None

    rows = []
    for tag, (fam, q, bastag) in sorted(ARMS.items()):
        d = dev_series(tag)
        b = dev_series(bastag)
        if d is None:
            print('missing run: %s' % tag)
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
        gl = gate_log(tag)

        ev_t = np.array([float(e['t_abs']) for e in events])
        for e in events:
            t0 = float(e['t_abs'])
            m = (d['t'] >= t0 - 1e-6) & (d['t'] < t0 + 2.0)
            pre = d['t'] < t0 - 1e-6
            dev_pre = d['dev'][pre][-1] if pre.any() else float('nan')
            if m.sum() == 0:
                peak = jump = rec = float('nan')
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
            dstep = ftot = float('nan')
            if fy is not None and ref_t is not None:
                fs = ref_t[(ref_t >= t0 - 0.4) & (ref_t < t0 + 2.0)]
                if fs.size > 2:
                    yi = np.interp(fs, fy[0], fy[1])
                    dstep = float(np.degrees(np.abs(np.diff(yi))).max())
                    ftot = float(np.degrees(np.abs(yi - yi[0])).max())
            nw = int(((wt >= t0 - 0.2) & (wt < t0 + 5.0)).sum()) if wt.size else 0
            nnp = nh = ''
            if gl is not None:
                w = (gl[0] >= t0 - 0.2) & (gl[0] < t0 + 5.0)
                nnp = int((gl[1][w] == 2).sum())
                nh = int((gl[1][w] == 1).sum())
            rows.append(dict(arm=tag, family=fam, q=q, event_id=e['event_id'],
                             amp_deg=float(e['amp_deg']), dur=int(e['dur_frames']),
                             t_abs=t0, jump_m=round(jump, 4), peak_m=round(peak, 4),
                             peak_base_m=round(pbase, 4),
                             recovery_s=round(rec, 3) if rec == rec else '',
                             withheld=nheld, gate_warns=nw,
                             filt_step_deg=round(dstep, 3) if dstep == dstep else '',
                             filt_exc_deg=round(ftot, 3) if ftot == ftot else '',
                             meas_nopub=nnp, meas_hold=nh))

        def far_of(t):
            return (np.all(np.abs(ev_t - t) > 3.0)
                    and (excl.size == 0 or np.all(np.abs(excl - t) > 1.0)))

        n_far_warn = int(sum(1 for t, _ in wl if far_of(t)))
        miss = missing_stamps(ref_t, d['t'])
        n_far_held = int(sum(1 for t in miss if far_of(t)))
        far_np = far_h = ''
        if gl is not None:
            far = np.array([far_of(t) for t in gl[0]])
            far_np = int(((gl[1] == 2) & far).sum())
            far_h = int(((gl[1] == 1) & far).sum())
        rows.append(dict(arm=tag, family=fam, q=q, event_id='_RUN_', amp_deg='', dur='',
                         t_abs='', jump_m='',
                         peak_m=round(float(np.median(d['dev'][keep(d['t'])])), 4),
                         peak_base_m=round(base_med, 4),
                         recovery_s=round(float(np.percentile(b['dev'][keep(b['t'])], 99)), 4)
                         if b is not None else '',
                         withheld=n_far_held, gate_warns=n_far_warn,
                         filt_step_deg='', filt_exc_deg='',
                         meas_nopub=far_np, meas_hold=far_h))

    with open(os.path.join(OUTD, 'per_event.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('wrote per_event.csv (%d rows)' % len(rows))
    return rows


if __name__ == '__main__':
    main()
