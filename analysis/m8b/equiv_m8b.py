#!/usr/bin/env python3
"""M8b: the online measurement gate vs fig/gate.py on the identical input stream.

M8 could only compare approximately: its gate read the filter's yaw inside
navsat_transform, which is not recordable from outside the node.  Here the gate's input
IS recordable -- the node logs, for every frame, the heading it received and the
cumulative gyro it held at that moment -- so the offline replay of the very same
fig/gate.py (same parameters) must agree frame for frame.  Anything else is a bug.

Writes data/m8b_measurement_gate/gate_equivalence.csv and prints every mismatch verbatim.
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, 'data', 'm8b_measurement_gate', 'runs')
OUTD = os.path.join(ROOT, 'data', 'm8b_measurement_gate')
sys.path.insert(0, os.path.join(ROOT, 'fig'))
import gate  # noqa: E402

PAR = dict(max_yaw_rate=1.234, yaw_gate_margin=0.05, nominal_dt=0.2,
           max_hold_frames=5, max_gap_frames=3, gyro_drift_rate=0.002,
           use_gyro=True, max_gyro_ref_time=10.0)


def main():
    tags = sorted(t for t in os.listdir(RUNS)
                  if os.path.isfile(os.path.join(RUNS, t, 'meas_gate.csv')))
    rows = []
    for tag in tags:
        a = np.genfromtxt(os.path.join(RUNS, tag, 'meas_gate.csv'),
                          delimiter=',', names=True)
        if a['t'].size < 10:
            continue
        r = gate.gate_series_v3(a['yaw_in'], a['t'], cum_gyro_rad=a['gz_cum'], **PAR)
        st_on = a['state'].astype(int)
        pb_on = a['published'].astype(int).astype(bool)
        uy_on = a['used_yaw']
        st_off = r['state'].astype(int)
        pb_off = r['published']
        uy_off = r['used_yaw']

        ds = np.where(st_on != st_off)[0]
        dp = np.where(pb_on != pb_off)[0]
        both = pb_on & pb_off
        dy = np.where(np.abs(uy_on - uy_off)[both] > 1e-9)[0]
        for i in ds[:20]:
            print('MISMATCH %s frame %d t=%.6f state online=%d offline=%d'
                  % (tag, i, a['t'][i], st_on[i], st_off[i]))
        for i in dp[:20]:
            print('MISMATCH %s frame %d t=%.6f published online=%d offline=%d'
                  % (tag, i, a['t'][i], pb_on[i], pb_off[i]))
        rows.append(dict(tag=tag, n=int(a['t'].size),
                         n_accept=int((st_off == gate.ACCEPT).sum()),
                         n_hold=int((st_off == gate.HOLD).sum()),
                         n_nopub=int((st_off == gate.NOPUB).sum()),
                         state_mismatch=int(ds.size), pub_mismatch=int(dp.size),
                         used_yaw_mismatch=int(dy.size),
                         max_used_yaw_diff_deg=float(np.degrees(
                             np.max(np.abs(uy_on - uy_off)[both])) if both.any() else 0.0),
                         n_reanchor=int(r['n_reanchor']), n_gap=int(r['n_gap']),
                         max_buf=int(a['buf_len'].max())))
        print('%-16s n=%5d  state_mismatch=%d pub_mismatch=%d used_yaw_mismatch=%d'
              % (tag, a['t'].size, ds.size, dp.size, dy.size))
    with open(os.path.join(OUTD, 'gate_equivalence.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    tot = sum(r['state_mismatch'] + r['pub_mismatch'] + r['used_yaw_mismatch'] for r in rows)
    print('TOTAL MISMATCHES: %d over %d runs, %d frames'
          % (tot, len(rows), sum(r['n'] for r in rows)))


if __name__ == '__main__':
    main()
