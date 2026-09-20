#!/usr/bin/env python3
"""M8b: where the measurement gate's apparent baseline cost actually comes from.

The no-injection runs give MEAS-GATE a p99 deviation of ~0.12 m against STOCK's 0.017 m,
which would read as "the gate costs 10 cm".  It does not: every frame above the p99 sits
inside one of ten episodes in which the RECORDED heading itself is frozen or exactly zero
(the driver's hd_ok = 0 windows).  In those windows the recorded /rtk_odom -- the
reference -- is built on a broken heading, so an arm that rejects the broken heading is
scored as deviating.  data/m8_replay/exclude.csv lists only ten such instants; the
detector below finds the full set from the heading stream the replay itself published.

Prints the baseline statistics with and without those episodes.
"""
import csv
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
R = os.path.join(ROOT, 'data', 'm8b_measurement_gate', 'runs')
M8 = os.path.join(ROOT, 'data', 'm8_replay', 'runs')
T0 = 1788911802.737

RUNS = [('bas_stock_q060', M8), ('bas_gate_q060', M8), ('bas_meas_q060', R),
        ('bas_both_q060', R), ('bas_stock_q1e5', M8), ('bas_gate_q1e5', M8),
        ('bas_meas_q1e5', R), ('bas_both_q1e5', R)]
TAIL = 12.0     # s kept out after an episode: the filter needs to re-converge
LEAD = 2.0


def episodes():
    h = np.genfromtxt(os.path.join(R, 'bas_meas_q060', 'heading.csv'),
                      delimiter=',', names=True)
    y, t = h['yaw_true'], h['t']
    bad = np.abs(y) < 1e-12
    bad[1:] |= np.abs(np.diff(y)) < 1e-12
    bt = t[bad]
    ep, cur = [], [bt[0]]
    for x in bt[1:]:
        if x - cur[-1] < 3.0:
            cur.append(x)
        else:
            ep.append((cur[0], cur[-1]))
            cur = [x]
    ep.append((cur[0], cur[-1]))
    return ep, int(bad.sum()), t.size


def dev(tag, root):
    g = np.genfromtxt(os.path.join(root, tag, 'odom_gps.csv'), delimiter=',', names=True)
    r = np.genfromtxt(os.path.join(root, tag, 'rtk_ref.csv'), delimiter=',', names=True)
    rx = np.interp(g['t'], r['t'], r['x'])
    ry = np.interp(g['t'], r['t'], r['y'])
    dx, dy = rx - g['x'], ry - g['y']
    return g['t'], np.hypot(dx - np.median(dx), dy - np.median(dy))


def main():
    excl = np.array([float(r['t_abs'])
                     for r in csv.DictReader(open(os.path.join(ROOT, 'data', 'm8_replay',
                                                               'exclude.csv')))])
    ep, nbad, ntot = episodes()
    print('frozen/zero heading frames in the recording: %d / %d (%.2f %%), %d episodes'
          % (nbad, ntot, 100.0 * nbad / ntot, len(ep)))
    for a, b in ep:
        print('   %.1f - %.1f s (bag offset), %.1f s' % (a - T0, b - T0, b - a))
    print('%-16s %8s %8s %8s | %8s %8s %8s %s'
          % ('run', 'med', 'p99', 'max', 'med*', 'p99*', 'max*', 'kept'))
    for tag, root in RUNS:
        if not os.path.isfile(os.path.join(root, tag, 'odom_gps.csv')):
            continue
        t, d = dev(tag, root)
        k = np.min(np.abs(t[:, None] - excl[None, :]), axis=1) > 1.0
        t, d = t[k], d[k]
        m = np.ones(t.size, bool)
        for a, b in ep:
            m &= ~((t > a - LEAD) & (t < b + TAIL))
        print('%-16s %8.4f %8.4f %8.4f | %8.4f %8.4f %8.4f  %d/%d'
              % (tag, np.median(d), np.percentile(d, 99), d.max(),
                 np.median(d[m]), np.percentile(d[m], 99), d[m].max(), m.sum(), t.size))
    print('* = the ten frozen/zero-heading episodes removed (-%.0f s / +%.0f s), because '
          'the reference itself is wrong there' % (LEAD, TAIL))


if __name__ == '__main__':
    main()
