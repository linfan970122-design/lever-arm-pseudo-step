#!/usr/bin/env python3
"""M8b section 7b: with the extra Python node in the loop, is 6x still harmless?

Criterion is M8's (data/m8_replay/design.md section 7): the difference a fast replay
introduces must not exceed the difference a same-rate repeat run introduces.  Same
window (bag 1100-1220 s), same arm (MEAS, q=0.06, no injection), only -r changes.
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, 'data', 'm8b_measurement_gate', 'runs')


def traj(tag):
    a = np.genfromtxt(os.path.join(RUNS, tag, 'odom_gps.csv'), delimiter=',', names=True)
    return a['t'], a['x'], a['y']


def cmp(t1, t2, tol=0.002, settle=0.0):
    """settle: seconds of the common span to drop, so the filters' warm-up (the two runs
    start at slightly different bag instants) is not read as a replay-rate effect."""
    (ta, xa, ya), (tb, xb, yb) = t1, t2
    if settle > 0.0:
        t0 = max(ta[0], tb[0]) + settle
        keep = ta >= t0
        ta, xa, ya = ta[keep], xa[keep], ya[keep]
    idx = np.clip(np.searchsorted(tb, ta), 1, tb.size - 1)
    pick = np.where(np.abs(tb[idx] - ta) < np.abs(tb[idx - 1] - ta), idx, idx - 1)
    ok = np.abs(tb[pick] - ta) <= tol
    d = np.hypot(xa[ok] - xb[pick][ok], ya[ok] - yb[pick][ok])
    return d


def main():
    a, b, c = traj('ver_meas_6x_a'), traj('ver_meas_6x_b'), traj('ver_meas_1x')
    for settle in (0.0, 2.0):
        print('--- settle %.0f s ---' % settle)
        for lbl, d in [('6x vs 6x', cmp(a, b, settle=settle)),
                       ('1x vs 6x', cmp(c, a, settle=settle)),
                       ('1x vs 6x(b)', cmp(c, b, settle=settle))]:
            print('%-12s n=%4d  median=%.6f  p99=%.4f  max=%.4f'
                  % (lbl, d.size, np.median(d), np.percentile(d, 99), d.max()))
    for tag in ('ver_meas_1x', 'ver_meas_6x_a', 'ver_meas_6x_b'):
        t, x, y = traj(tag)
        print('%-14s frames=%d' % (tag, t.size))


if __name__ == '__main__':
    main()
