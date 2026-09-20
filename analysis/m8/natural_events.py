#!/usr/bin/env python3
"""M8: the two recorded 6 September events through the real chain.

The recorded /rtk_odom already carries the fault (the driver applied the bad
heading to the lever arm), so for these events it is not a clean reference.
Two quantities are reported instead:

  step_gps   frame-to-frame step of the consumer's position at the event frame
             -- what the manuscript's Section 4.3 measures on the driver output
  diff_rtk   |p_gps - p_rtk| around the event, i.e. how much of the recorded
             fault this configuration did NOT reproduce
"""
import csv
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, 'data', 'm8_replay', 'runs')
OUTD = os.path.join(ROOT, 'data', 'm8_replay')

# (label, event stamp, recorded published step, recorded heading jump)
EVENTS = [
    ('E_-67.5deg', 1788681423.094, 0.4014, -67.54),
    ('E_-167.5deg', 1788680085.536, 0.7264, -167.51),
]
GROUPS = {'E_-67.5deg': 'nat1', 'E_-167.5deg': 'nat2'}
ARMS = ['stock_q060', 'gate_q060', 'mahal_q060', 'gate_q1e4', 'stock_q1e4']


def main():
    rows = []
    for lbl, t0, rec_step, rec_dpsi in EVENTS:
        pre = GROUPS[lbl]
        for arm in ARMS:
            tag = '%s_%s' % (pre, arm)
            d = os.path.join(RUNS, tag)
            if not os.path.isdir(d) or not os.path.isfile(os.path.join(d, 'odom_gps.csv')):
                continue
            g = np.genfromtxt(os.path.join(d, 'odom_gps.csv'), delimiter=',', names=True)
            r = np.genfromtxt(os.path.join(d, 'rtk_ref.csv'), delimiter=',', names=True)
            if g['t'].size < 20:
                continue
            rx = np.interp(g['t'], r['t'], r['x'])
            ry = np.interp(g['t'], r['t'], r['y'])
            ox, oy = np.median(rx - g['x']), np.median(ry - g['y'])
            diff = np.hypot(rx - g['x'] - ox, ry - g['y'] - oy)

            i = int(np.searchsorted(g['t'], t0))
            if i <= 0 or i >= g['t'].size:
                continue
            step = float(np.hypot(g['x'][i] - g['x'][i - 1], g['y'][i] - g['y'][i - 1]))
            # a gap in the published stream at the event = withheld frames
            gap = float(g['t'][i] - g['t'][i - 1])
            w = (g['t'] >= t0 - 0.5) & (g['t'] < t0 + 10.0)
            wn = os.path.join(d, 'warns.csv')
            kinds = []
            if os.path.isfile(wn):
                for row in csv.DictReader(open(wn)):
                    if row['kind'] in ('hold', 'stale', 'invalid', 'reanchor_gyro',
                                       'reanchor_two') and t0 - 0.5 <= float(row['t']) < t0 + 10:
                        kinds.append(row['kind'])
            rows.append(dict(event=lbl, arm=arm, rec_step_m=rec_step, rec_dpsi_deg=rec_dpsi,
                             step_gps_m=round(step, 4), pub_gap_s=round(gap, 3),
                             diff_rtk_peak_m=round(float(diff[w].max()), 4) if w.any() else '',
                             diff_rtk_med_m=round(float(np.median(diff)), 4),
                             gate_actions=';'.join(kinds) if kinds else '-'))
    if not rows:
        print('no natural-event runs found')
        return
    with open(os.path.join(OUTD, 'natural_events.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    hdr = list(rows[0].keys())
    print(' | '.join(hdr))
    for r in rows:
        print(' | '.join(str(r[k]) for k in hdr))


if __name__ == '__main__':
    main()
