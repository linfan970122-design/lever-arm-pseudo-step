#!/usr/bin/env python3
"""M8b post-review: frame-by-frame dissection of the 1520-1560 s episode of bag9.

This is the window the -2 s / +12 s guard of analysis/m8b/baseline_cost_m8b.py removed
(its episode 1531.5-1533.5 s in the T0_REC origin = 1527.3-1529.3 s here), and it is
where the measurement-side gate costs the consumer the most.

What the window contains (all offsets against T0_REF = 1788911806.950244):
  1525.5-1527.1  the RTK heading degrades (sats 21 -> 13 -> 10, quality 4 -> 5 -> 2) and is
                 already ~30 deg away from the course over ground, while hd_ok stays 1
  1527.1-1527.5  heading frozen at 242.585 deg
  1527.7-1529.3  heading frozen at 270.584 deg while the vehicle turns
  1529.5-1530.5  heading on a wrong branch (course over ground disagrees by 76-121 deg)
  1530.5         +91.5 deg step back onto the correct branch; from here the recorded
                 heading agrees with the course over ground to within ~8 deg, i.e. the
                 reference is good again
  1530.7-1539.6  the measurement gate rejects those correct headings
  1539.6-1549.6  (BOTH only) the node-side gate treats the measurement gate's correction
                 as a fault and holds for another 10 s

Writes (new file):  data/m8b_measurement_gate/episode_1527_1550.csv
and prints the re-anchor arithmetic.
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (ACCEPT, HOLD, NOPUB, ROOT, STATE_NAME, T0_REF, dev_series, load,
                    run_dir, source, wrap_deg)
sys.path.insert(0, os.path.join(ROOT, 'fig'))
import gate as gatemod                                    # fig/gate.py, verbatim

OUT = os.path.join(ROOT, 'data', 'm8b_measurement_gate')
T_A, T_B = 1520.0, 1560.0
PAR = dict(max_yaw_rate=1.234, yaw_gate_margin=0.05, nominal_dt=0.2, max_hold_frames=5,
           max_gap_frames=3, gyro_drift_rate=0.002, use_gyro=True, max_gyro_ref_time=10.0)
ARMS4 = ['bas_stock_q060', 'bas_gate_q060', 'bas_meas_q060', 'bas_both_q060']


def node_warns(tag):
    p = os.path.join(run_dir(tag), 'warns.csv')
    return [(float(r['t']) - T0_REF, r['kind'], r['msg']) for r in csv.DictReader(open(p))
            if r['kind'] != 'other']


def main():
    g = load('bas_meas_q060', 'meas_gate.csv')
    t = g['t']
    # the gate, re-run offline over the whole stream (fig/gate.py verbatim); the online
    # decisions were already shown frame-identical (summary.md table 6), re-asserted here
    res = gatemod.gate_series_v3(g['yaw_in'], t, cum_gyro_rad=g['gz_cum'], **PAR)
    assert (res['state'] == g['state'].astype(int)).all(), 'offline gate != online log'
    src = source()
    st, sx, sy = src['t'], src['x'], src['y']
    dt = np.gradient(st)
    dt[dt <= 0] = np.nan
    course = np.degrees(np.arctan2(np.gradient(sy) / dt, np.gradient(sx) / dt))
    speed = np.hypot(np.gradient(sx) / dt, np.gradient(sy) / dt)

    devs = {}
    pubs = {}
    for tag in ARMS4:
        tt, dd = dev_series(tag)
        devs[tag] = (tt, dd)
        pubs[tag] = tt
    warns_both = node_warns('bas_both_q060')
    warns_gate = node_warns('bas_gate_q060')

    def node_state(tag, ti, warns):
        """Node-side gate state at ti, reconstructed from its own warnings."""
        s = 'accept'
        for tw, kind, _ in warns:
            if tw > ti:
                break
            if kind == 'hold':
                s = 'hold'
            elif kind == 'stale':
                s = 'stale (withholding /odometry/gps)'
            elif kind.startswith('reanchor'):
                s = 'accept (re-anchored by %s)' % kind.split('_')[1]
        return s

    rows = []
    m = (t - T0_REF >= T_A) & (t - T0_REF <= T_B)
    gz_prev = None
    for i in np.flatnonzero(m):
        ti = t[i] - T0_REF
        j = int(np.argmin(np.abs(st - t[i])))
        gyro_inc = np.degrees(g['gz_cum'][i] - g['gz_cum'][i - 1]) if i > 0 else np.nan
        r = dict(t_ref=round(ti, 3), t_rec=round(ti + 4.213244, 3),
                 hdg_deg=src['hdg_deg'][j], dpsi_deg=src['dpsi_deg'][j],
                 dpsi_imu_deg=src['dpsi_imu_deg'][j], gyro_inc_deg=gyro_inc,
                 gz_cum_deg=np.degrees(g['gz_cum'][i]),
                 hd_ok=int(src['hd_ok'][j]), quality=src['quality'][j],
                 sats=src['sats'][j], gz_max=src['gz_max'][j],
                 course_deg=course[j], speed_ms=speed[j],
                 course_minus_hdgyaw=wrap_deg(course[j] - wrap_deg(90.0 - src['hdg_deg'][j])),
                 yaw_in_deg=np.degrees(g['yaw_in'][i]),
                 meas_state=STATE_NAME[int(g['state'][i])],
                 meas_published=int(g['published'][i]),
                 meas_used_yaw_deg=np.degrees(g['used_yaw'][i]),
                 meas_hold_count=int(g['hold_count'][i]),
                 meas_valid=int(g['valid'][i]), meas_reanchor=int(g['reanchor'][i]),
                 meas_fallback_b=int(g['fallback_b'][i]),
                 gate_delta_deg=np.degrees(res['delta'][i]),
                 gate_budget_deg=np.degrees(res['budget'][i]),
                 anchor_t_ref=t[int(res['anchor'][i])] - T0_REF,
                 age_since_anchor_s=t[i] - t[int(res['anchor'][i])],
                 node_state_both=node_state('bas_both_q060', ti, warns_both),
                 node_state_gated=node_state('bas_gate_q060', ti, warns_gate))
        for tag, key in zip(ARMS4, ('dev_stock', 'dev_gated', 'dev_meas', 'dev_both')):
            tt, dd = devs[tag]
            k = int(np.argmin(np.abs(tt - t[i])))
            r[key] = dd[k] if abs(tt[k] - t[i]) < 0.12 else np.nan
            r[key + '_published'] = int(abs(tt[k] - t[i]) < 0.12)
        rows.append(r)

    p = os.path.join(OUT, 'episode_1527_1550.csv')
    with open(p, 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (('%.6g' % v) if isinstance(v, float) else v)
                        for k, v in r.items()})
    print('wrote', p, len(rows), 'frames')

    # ---------------- the re-anchor arithmetic, printed -------------------------
    print('\n--- measurement gate, decisive frames (offsets against T0_REF) ---')
    for i in np.flatnonzero(m):
        ti = t[i] - T0_REF
        if not (1526.5 <= ti <= 1540.2):
            continue
        print('%8.3f yaw_in=%9.3f  %-6s pub=%d used=%9.3f  d=%8.3f budget=%6.3f  '
              'anchor@%8.3f age=%5.2f s%s%s'
              % (ti, np.degrees(g['yaw_in'][i]), STATE_NAME[int(g['state'][i])],
                 g['published'][i], np.degrees(g['used_yaw'][i]),
                 np.degrees(res['delta'][i]), np.degrees(res['budget'][i]),
                 t[int(res['anchor'][i])] - T0_REF, t[i] - t[int(res['anchor'][i])],
                 '  REANCHOR' if res['reanchor'][i] else '',
                 '  fallback_b' if res['fallback_b'][i] else ''))
    print('\nmax_gyro_ref_time = %.1f s (analysis/m8b/m8b.launch line 61 and '
          'analysis/m8/m8.launch line 133; node default in m8b_gate_node.py is also 10.0)'
          % PAR['max_gyro_ref_time'])
    print('\n--- node-side gate (from its own warnings) ---')
    for tag, w_ in (('bas_gate_q060', warns_gate), ('bas_both_q060', warns_both)):
        for tw, kind, msg in w_:
            if 1520 <= tw <= 1560:
                print('%-15s %9.3f %-14s %s' % (tag, tw, kind, msg.strip('"')))


if __name__ == '__main__':
    main()
