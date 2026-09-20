#!/usr/bin/env python3
"""M8: build the synthetic heading-outlier injection schedule for the replay.

Amplitudes 5/10/20/45/90 deg x durations 1/2/5 frames x 3 instants = 45 events,
all placed inside straight, moving segments of run_20260909_2026-09-09-07-56-42.bag
(classification cls_imu == 'straight' taken from data/csv/, i.e. from the IMU, not
from the heading under test).

Writes data/m8_replay/events.csv with bag-relative offsets (seconds).
"""
import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, 'data', 'm8_replay', 'events.csv')

BAG = 'run_20260909_2026-09-09-07-56-42'
BAG_T0 = 1788911802.737          # first /rtk_odom stamp, from data/csv/<BAG>.csv

# straight, moving windows (bag-relative s), from cls_imu; 5 s margin already applied
# 922-1000 s is left out: the recording's four natural pseudo-steps sit at 984-994 s
WINDOWS = [
    (1590.0, 1972.0),   # W3, 382 s
    (1000.0, 1086.0),   # W2, 86 s
    (106.0, 226.0),     # W1, 120 s
]
SPACING = 12.0          # s between injection instants; > hold + re-anchor + 2 s metric window

AMPS = [5.0, 10.0, 20.0, 45.0, 90.0]     # deg
DURS = [1, 2, 5]                          # /rtk_odom frames (0.2 s each)


def slots():
    out = []
    for lo, hi in WINDOWS:
        t = lo
        while t <= hi:
            out.append(t)
            t += SPACING
    return out


def main():
    combos = [(a, d) for a in AMPS for d in DURS]     # 15
    plan = []
    for rep in range(3):
        for a, d in combos:
            plan.append((a, d, rep))
    s = slots()
    assert len(s) >= len(plan), (len(s), len(plan))
    rows = []
    for i, (a, d, rep) in enumerate(plan):
        off = s[i]
        rows.append(dict(event_id='E%02d' % i, bag=BAG, t_offset=round(off, 3),
                         t_abs=round(BAG_T0 + off, 3), amp_deg=a, dur_frames=d,
                         rep=rep, kind='synthetic'))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('wrote %s  (%d events, slots available %d)' % (OUT, len(rows), len(s)))


if __name__ == '__main__':
    main()
