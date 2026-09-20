#!/usr/bin/env python3
"""M8: is the deployed C++ gate the same rule as the offline twin fig/gate.py?

The C++ gate runs on the yaw navsat_transform reads from tf, i.e. the EKF's yaw
interpolated at the fix stamp.  Feed exactly that sequence to gate_series_v3 with
the same parameters and compare, frame by frame:

  * published        -- C++: the fix appears on /odometry/gps at all
  * used heading     -- C++: recovered from the published point,
                        p_ant - p_gps = R(psi_used) * L

Writes data/m8_replay/gate_equivalence.csv and prints the mismatch counts.
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
import gate as G                                    # noqa: E402

LEVER = (0.235, -0.280)
L_ANG = math.atan2(LEVER[1], LEVER[0])
PARAMS = dict(max_yaw_rate=1.234, yaw_gate_margin=0.05, nominal_dt=0.2,
              max_hold_frames=5, max_gap_frames=3, gyro_drift_rate=0.002,
              use_gyro=True, max_gyro_ref_time=10.0)


def nz(p):
    return np.genfromtxt(p, delimiter=',', names=True)


def run(tag, stock_tag):
    d = os.path.join(RUNS, tag)
    h = nz(os.path.join(d, 'heading.csv'))
    f = nz(os.path.join(d, 'odom_filtered.csv'))
    g = nz(os.path.join(d, 'odom_gps.csv'))
    gs = nz(os.path.join(RUNS, stock_tag, 'odom_gps.csv'))
    rs = nz(os.path.join(RUNS, stock_tag, 'rtk_ref.csv'))

    # stamps of every fix the node was offered: the stock arm never withholds
    t_all = gs['t']
    # the yaw the node read from tf at each of those stamps
    fy = np.unwrap(f['yaw'])
    yaw_in = np.interp(t_all, f['t'], fy)

    # gyro reference: the same rectangular integral the C++ gyroCallback keeps
    if 'gz_cum' in h.dtype.names:
        cum = np.interp(t_all, h['t'], h['gz_cum'])
    else:
        cum = None

    res = G.gate_series_v3(yaw_in, t_all, cum_gyro_rad=cum, **PARAMS)

    # C++ side: published = a fix with this stamp exists on /odometry/gps
    pub_cpp = np.isin(np.round(t_all, 4), np.round(g['t'], 4))

    # C++ side: which heading rotated the lever arm
    rx = np.interp(t_all, rs['t'], rs['x'])
    ry = np.interp(t_all, rs['t'], rs['y'])
    ryaw = np.interp(t_all, rs['t'], np.unwrap(rs['yaw']))
    ax = rx + (np.cos(ryaw) * LEVER[0] - np.sin(ryaw) * LEVER[1])
    ay = ry + (np.sin(ryaw) * LEVER[0] + np.cos(ryaw) * LEVER[1])
    gx = np.interp(t_all, g['t'], g['x'])
    gy = np.interp(t_all, g['t'], g['y'])
    ox = np.median(rx - gx)
    oy = np.median(ry - gy)
    used_cpp = np.arctan2(ay - (gy + oy), ax - (gx + ox)) - L_ANG

    used_py = res['used_yaw']
    ok = res['published'] & pub_cpp
    dyaw = np.abs((used_cpp - used_py + np.pi) % (2 * np.pi) - np.pi)
    n_pub_mismatch = int((res['published'] != pub_cpp).sum())
    n_yaw_mismatch = int((ok & (dyaw > math.radians(0.5))).sum())
    return dict(tag=tag, n=int(t_all.size), n_pub_py=int(res['published'].sum()),
                n_pub_cpp=int(pub_cpp.sum()), pub_mismatch=n_pub_mismatch,
                yaw_cmp=int(ok.sum()), yaw_mismatch=n_yaw_mismatch,
                yaw_p95_deg=round(float(np.degrees(np.percentile(dyaw[ok], 95))), 4)
                if ok.any() else float('nan'),
                n_reanchor=int(res['n_reanchor']), n_gap=int(res['n_gap']))


def main():
    pairs = [(a, b) for a, b in [('inj_gate_q060_eq', 'inj_stock_q060'),
                                 ('inj_gate_q1e5_eq', 'inj_stock_q1e5'),
                                 ('inj_gate_q060', 'inj_stock_q060'),
                                 ('inj_gate_q1e5', 'inj_stock_q1e5')]
             if os.path.isdir(os.path.join(RUNS, a)) and os.path.isdir(os.path.join(RUNS, b))]
    rows = [run(a, b) for a, b in pairs]
    with open(os.path.join(OUTD, 'gate_equivalence.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(r)


if __name__ == '__main__':
    main()
