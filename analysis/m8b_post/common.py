#!/usr/bin/env python3
"""Shared loaders for the M8b post-review cost analysis (analysis/m8b_post/).

Nothing here writes into analysis/m8, analysis/m8b, data/m8_replay, or any file that
already existed in data/m8b_measurement_gate; new files only.

TIME ORIGIN --------------------------------------------------------------------------
Every offset printed by this package is  t_abs - T0_REF  with

    T0_REF = 1788911806.950244 s   (= rtk_ref.csv t.min() of runs/bas_meas_q060)

analysis/m8b/baseline_cost_m8b.py instead uses T0_REC = 1788911802.737 s, the first row
of the source recording data/csv/run_20260909_2026-09-09-07-56-42.csv, which is also the
origin of data/m8_replay/events.csv (t_abs = T0_REC + t_offset).

    offset_REF = offset_REC - 4.213244  s

So the frozen-heading episode that baseline_cost_m8b.py prints as 1531.5-1533.5 s is
1527.3-1529.3 s here.
"""
import csv
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
R_M8B = os.path.join(ROOT, 'data', 'm8b_measurement_gate', 'runs')
R_M8 = os.path.join(ROOT, 'data', 'm8_replay', 'runs')
D_M8 = os.path.join(ROOT, 'data', 'm8_replay')
SRC_CSV = os.path.join(ROOT, 'data', 'csv',
                       'run_20260909_2026-09-09-07-56-42.csv')

T0_REF = 1788911806.950244
T0_REC = 1788911802.737
DT_ORIGIN = T0_REC - T0_REF          # -4.213244 s

ACCEPT, HOLD, NOPUB = 0, 1, 2
STATE_NAME = {0: 'ACCEPT', 1: 'HOLD', 2: 'NOPUB'}

# arm tag -> (pretty name, runs root)
ARMS = {
    'stock': ('STOCK', R_M8),
    'gate': ('GATED', R_M8),
    'meas': ('MEAS', R_M8B),
    'both': ('BOTH', R_M8B),
}
QS = [('q060', '0.06'), ('q1e3', '1e-3'), ('q1e4', '1e-4'), ('q1e5', '1e-5')]


def run_dir(tag):
    fam = tag.split('_')[1]
    return os.path.join(ARMS[fam][1], tag)


def load(tag, name):
    p = os.path.join(run_dir(tag), name)
    if not os.path.isfile(p) or os.path.getsize(p) < 40:
        return None
    return np.genfromtxt(p, delimiter=',', names=True)


def dev_series(tag):
    """Consumer deviation from the recorded reference, same definition as
    analysis/m8/analyze_m8.py and analysis/m8b/baseline_cost_m8b.py: the constant
    frame offset between the two is removed as the median over the whole run."""
    g = load(tag, 'odom_gps.csv')
    r = load(tag, 'rtk_ref.csv')
    rx = np.interp(g['t'], r['t'], r['x'])
    ry = np.interp(g['t'], r['t'], r['y'])
    dx, dy = rx - g['x'], ry - g['y']
    return g['t'], np.hypot(dx - np.median(dx), dy - np.median(dy))


def frozen_frames():
    """The 85 frozen/zero heading frames of the recording, detected exactly as
    analysis/m8b/baseline_cost_m8b.py does it (yaw_true of runs/bas_meas_q060), plus
    the episode grouping (gap > 3 s starts a new episode)."""
    h = np.genfromtxt(os.path.join(R_M8B, 'bas_meas_q060', 'heading.csv'),
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
    return t, bad, bt, ep


def exclude_instants():
    return np.array([float(r['t_abs']) for r in
                     csv.DictReader(open(os.path.join(D_M8, 'exclude.csv')))])


def source():
    """The per-frame source recording (t, hdg_deg, dpsi_deg, hd_ok, dpsi_imu_deg, gz_max...)."""
    return np.genfromtxt(SRC_CSV, delimiter=',', names=True, dtype=None, encoding='utf8')


def events():
    return [(r['event_id'], float(r['t_abs']), float(r['amp_deg']), int(r['dur_frames']))
            for r in csv.DictReader(open(os.path.join(D_M8, 'events.csv')))]


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0
