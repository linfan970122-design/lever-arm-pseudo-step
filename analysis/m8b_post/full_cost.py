#!/usr/bin/env python3
"""M8b post-review: the FULL-RUN cost of each arm, and every >0.10 m excursion.

Why this exists
---------------
data/m8b_measurement_gate/summary.md table 7 reports the baseline cost of the
measurement-side gate as "p99 0.0168 m ~= zero" after removing ten frozen-heading
episodes with a -2 s / +12 s guard.  That guard removes 14 s around each episode, and
the largest cost the gate actually produces -- a ~10 s hold while the vehicle turns,
which STARTS inside a frozen episode but continues long after it -- was removed with it.
This script reports the same runs with

  (a) no exclusion at all,
  (b) only the 85 frozen/zero heading frames themselves removed (no lead, no tail),
  (c) the old exclusion (exclude.csv +-1 s, then episodes -2 s / +12 s) for reference,

plus, for every run, every interval where the consumer deviation exceeds 0.10 m, with a
cause read off the recording and off the gate's own per-frame log.

Writes (new files only):
    data/m8b_measurement_gate/full_cost_table.csv
    data/m8b_measurement_gate/full_cost_excursions.csv
    data/m8b_measurement_gate/full_cost_injection.csv
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (ACCEPT, D_M8, DT_ORIGIN, R_M8B, ROOT, T0_REC, T0_REF, dev_series,
                    events, exclude_instants, frozen_frames, load, run_dir, source,
                    wrap_deg)

OUT = os.path.join(ROOT, 'data', 'm8b_measurement_gate')
THRESH = 0.10           # m, excursion threshold
MERGE = 0.6             # s, gap below which two excursions are one
LEAD, TAIL = 2.0, 12.0  # the old guard
BAS = ['bas_stock_q060', 'bas_gate_q060', 'bas_meas_q060', 'bas_both_q060',
       'bas_stock_q1e5', 'bas_gate_q1e5', 'bas_meas_q1e5', 'bas_both_q1e5']
INJ = ['inj_stock_q060', 'inj_gate_q060', 'inj_meas_q060', 'inj_both_q060',
       'inj_stock_q1e5', 'inj_gate_q1e5', 'inj_meas_q1e5', 'inj_both_q1e5',
       'inj_stock_q1e3', 'inj_gate_q1e3', 'inj_meas_q1e3', 'inj_both_q1e3',
       'inj_stock_q1e4', 'inj_gate_q1e4', 'inj_meas_q1e4', 'inj_both_q1e4']


def stats(d):
    if d.size == 0:
        return (np.nan,) * 3
    return np.median(d), np.percentile(d, 99), d.max()


def excursions(t, d, thresh=THRESH):
    m = d > thresh
    if not m.any():
        return []
    idx = np.flatnonzero(m)
    grp, cur = [], [idx[0]]
    for i in idx[1:]:
        if t[i] - t[cur[-1]] <= MERGE:
            cur.append(i)
        else:
            grp.append(cur)
            cur = [i]
    grp.append(cur)
    out = []
    for gidx in grp:
        gidx = np.array(gidx)
        k = gidx[np.argmax(d[gidx])]
        out.append(dict(t0=t[gidx[0]], t1=t[gidx[-1]], dur=t[gidx[-1]] - t[gidx[0]],
                        peak=d[k], t_peak=t[k], n=gidx.size))
    return out


def label(ex, tag, src, bt, gate, warns, evs):
    """Cause flags for one excursion, read from the recording and the gate logs."""
    t0, t1 = ex['t0'], ex['t1']
    w0, w1 = t0 - 3.0, t1 + 0.2          # the cause can precede the consumer excursion
    st = src['t']
    win = (st >= w0) & (st <= w1)
    f = {}
    f['frozen'] = bool(((bt >= w0) & (bt <= w1)).sum())
    f['hd_ok0'] = bool((src['hd_ok'][win] == 0).sum())
    dps = src['dpsi_deg'][win]
    dgy = src['dpsi_imu_deg'][win]
    big = np.isfinite(dps) & (np.abs(dps) > 17.0)
    f['jump_vs_gyro'] = bool((big & (np.abs(wrap_deg(dps - dgy)) > 17.0)).sum())
    gz = src['gz_max'][win]
    f['turning'] = bool(np.isfinite(gz).any() and np.nanmax(np.abs(gz)) > 0.20)
    f['inj'] = any(w0 <= te <= t1 + 2.0 for _, te, _, _ in evs)
    if gate is not None:
        gw = (gate['t'] >= t0 - 0.5) & (gate['t'] <= t1)
        f['meas_block'] = bool(gw.sum() and (gate['state'][gw] != ACCEPT).sum())
        f['meas_nonaccept'] = int((gate['state'][gw] != ACCEPT).sum())
    else:
        f['meas_block'] = False
        f['meas_nonaccept'] = 0
    f['node_warn'] = ';'.join(sorted(set(k for tw, k in warns if w0 <= tw <= t1 + 0.2)))
    if f['inj']:
        lab = 'injection'
    elif f['meas_block'] and f['turning'] and f['frozen']:
        lab = 'gate hold in turn (re-anchored on frozen heading)'
    elif f['meas_block'] and f['turning']:
        lab = 'gate hold in turn'
    elif f['frozen']:
        lab = 'frozen heading'
    elif f['hd_ok0']:
        lab = 'hd_ok=0'
    elif f['jump_vs_gyro']:
        lab = 'heading jump, gyro contradicts'
    elif f['meas_block'] or f['node_warn']:
        lab = 'gate action'
    else:
        lab = 'other'
    f['label'] = lab
    return f


def warn_list(tag):
    p = os.path.join(run_dir(tag), 'warns.csv')
    out = []
    if not os.path.isfile(p):
        return out
    for row in csv.DictReader(open(p)):
        if row['kind'] in ('hold', 'stale', 'invalid', 'reanchor_gyro', 'reanchor_two'):
            out.append((float(row['t']), row['kind']))
    return out


def main():
    ht, bad, bt, ep = frozen_frames()
    excl = exclude_instants()
    src = source()
    evs = events()
    print('# time origin T0_REF = %.6f  (offset_REC = offset_REF %+.6f)'
          % (T0_REF, -DT_ORIGIN))
    print('# frozen/zero heading frames: %d / %d (%.2f %%) in %d episodes'
          % (bad.sum(), ht.size, 100.0 * bad.sum() / ht.size, len(ep)))
    for a, b in ep:
        print('#   %8.2f - %8.2f s  (%.2f s, %d frames)'
              % (a - T0_REF, b - T0_REF, b - a, ((bt >= a) & (bt <= b)).sum()))

    trows, erows = [], []
    print('\n## baseline runs, full trajectory')
    print('%-16s | %-24s | %-24s | %-24s' % ('run', '(a) raw', '(b) frozen frames out',
                                             '(c) old -2/+12 s guard'))
    print('%-16s | %7s %7s %7s | %7s %7s %7s | %7s %7s %7s | kept(b) kept(c) n'
          % ('', 'med', 'p99', 'max', 'med', 'p99', 'max', 'med', 'p99', 'max'))
    for tag in BAS + INJ:
        t, d = dev_series(tag)
        # (b) only the frozen frames themselves: a consumer sample is dropped when the
        #     nearest frozen heading frame is within half a heading period (0.1 s)
        near = np.min(np.abs(t[:, None] - bt[None, :]), axis=1)
        mb = near > 0.1
        # (c) exactly analysis/m8b/baseline_cost_m8b.py
        kc = np.min(np.abs(t[:, None] - excl[None, :]), axis=1) > 1.0
        mc = kc.copy()
        for a, b in ep:
            mc &= ~((t > a - LEAD) & (t < b + TAIL))
        row = dict(run=tag, n=t.size)
        for key, m in (('a', np.ones(t.size, bool)), ('b', mb), ('c', mc)):
            s = stats(d[m])
            row['med_' + key], row['p99_' + key], row['max_' + key] = s
            row['n_' + key] = int(m.sum())
            if key != 'a':
                row['tmax_' + key] = (t[m][np.argmax(d[m])] - T0_REF) if m.any() else np.nan
        row['tmax_a'] = t[np.argmax(d)] - T0_REF
        trows.append(row)
        if tag in BAS:
            print('%-16s | %7.4f %7.4f %7.4f | %7.4f %7.4f %7.4f | %7.4f %7.4f %7.4f | '
                  '%5d %5d %5d' % (tag, row['med_a'], row['p99_a'], row['max_a'],
                                   row['med_b'], row['p99_b'], row['max_b'],
                                   row['med_c'], row['p99_c'], row['max_c'],
                                   row['n_b'], row['n_c'], row['n']))
        gate = load(tag, 'meas_gate.csv')
        warns = warn_list(tag)
        for ex in excursions(t, d):
            f = label(ex, tag, src, bt, gate, warns, evs)
            erows.append(dict(run=tag, t_start=ex['t0'] - T0_REF, t_end=ex['t1'] - T0_REF,
                              dur_s=ex['dur'], peak_m=ex['peak'],
                              t_peak=ex['t_peak'] - T0_REF, n_frames=ex['n'],
                              label=f['label'], frozen=int(f['frozen']),
                              hd_ok0=int(f['hd_ok0']),
                              jump_vs_gyro=int(f['jump_vs_gyro']),
                              turning=int(f['turning']), injection=int(f['inj']),
                              meas_nonaccept=f['meas_nonaccept'],
                              node_warns=f['node_warn']))

    print('\n## injection runs, full trajectory (same columns)')
    for row in trows:
        if row['run'] in INJ:
            print('%-16s | %7.4f %7.4f %7.4f @%8.2f s | (b) %7.4f %7.4f %7.4f | '
                  '(c) %7.4f %7.4f %7.4f'
                  % (row['run'], row['med_a'], row['p99_a'], row['max_a'], row['tmax_a'],
                     row['med_b'], row['p99_b'], row['max_b'],
                     row['med_c'], row['p99_c'], row['max_c']))

    print('\n## excursions above %.2f m' % THRESH)
    print('%-16s %9s %9s %7s %8s %9s  %s'
          % ('run', 'start', 'end', 'dur', 'peak', 't_peak', 'cause'))
    for r in erows:
        print('%-16s %9.2f %9.2f %7.2f %8.4f %9.2f  %s%s'
              % (r['run'], r['t_start'], r['t_end'], r['dur_s'], r['peak_m'],
                 r['t_peak'], r['label'],
                 (' [' + r['node_warns'] + ']') if r['node_warns'] else ''))

    # ---- injection: full-run max vs the per-event table -------------------------
    print('\n## per-event peaks vs full-run max')
    pe = {}
    for d_ in (os.path.join(D_M8, 'per_event.csv'), os.path.join(OUT, 'per_event.csv')):
        for r in csv.DictReader(open(d_)):
            if r['event_id'] == '_RUN_':      # per-run summary row, not an event
                continue
            pe.setdefault(r['arm'], []).append(float(r['peak_m']))
    irows = []
    for row in trows:
        if row['run'] not in INJ:
            continue
        p = pe.get(row['run'], [])
        pmax = max(p) if p else np.nan
        irows.append(dict(run=row['run'], n_events=len(p), per_event_max=pmax,
                          full_max=row['max_a'], t_full_max=row['tmax_a'],
                          full_max_frozen_out=row['max_b'],
                          t_full_max_frozen_out=row['tmax_b']))
        print('%-16s events=%2d  per_event max=%7.4f | full-run max=%7.4f @ %8.2f s | '
              'frozen-frames-out max=%7.4f @ %8.2f s'
              % (row['run'], len(p), pmax, row['max_a'], row['tmax_a'],
                 row['max_b'], row['tmax_b']))

    with open(os.path.join(OUT, 'full_cost_table.csv'), 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(trows[0].keys()))
        w.writeheader()
        [w.writerow(r) for r in trows]
    with open(os.path.join(OUT, 'full_cost_excursions.csv'), 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(erows[0].keys()))
        w.writeheader()
        [w.writerow(r) for r in erows]
    with open(os.path.join(OUT, 'full_cost_injection.csv'), 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(irows[0].keys()))
        w.writeheader()
        [w.writerow(r) for r in irows]
    print('\nwrote full_cost_table.csv / full_cost_excursions.csv / full_cost_injection.csv')


if __name__ == '__main__':
    main()
