#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Augment $P2_OUT/summary.csv (default ./out/summary.csv):
 - gyro bound recomputed after dropping non-physical CH110 samples (|wz|>5 rad/s)
 - pseudo-step counts: published position jumped while the reconstructed
   antenna point did not, with the solution reported fixed (quality 4).
"""
import csv, glob, os, math

D = os.environ.get('P2_OUT', 'out')

def q(v, p):
    if not v: return float('nan')
    s = sorted(v); k = (len(s)-1)*p; i = int(k); f = k-i
    return s[i] if i+1 >= len(s) else s[i]*(1-f)+s[i+1]*f

rows = list(csv.DictReader(open(os.path.join(D, 'summary.csv'))))
add = {}
for f in sorted(glob.glob(os.path.join(D, 'csv', '*.csv'))):
    gz = []; ps = {0.05: 0, 0.10: 0, 0.20: 0, 0.30: 0}; nbad = 0
    name = None
    for r in csv.DictReader(open(f)):
        name = r['bag']
        if r['gz_max']:
            v = float(r['gz_max'])
            if v > 5.0: nbad += 1
            else: gz.append(v)
        if r['dp_pub'] and r['dp_ant'] and r['fixed'] == '1' and r['hd_ok'] == '1':
            dp = float(r['dp_pub']); da = float(r['dp_ant'])
            if da <= 0.02:
                for k in ps:
                    if dp >= k: ps[k] += 1
    add[name] = dict(
        gyro_p99_deg_frame_filt='%.4f' % (math.degrees(q(gz, .99))*0.2) if gz else '',
        gyro_max_deg_frame_filt='%.4f' % (math.degrees(max(gz))*0.2) if gz else '',
        gyro_bad_samples=nbad,
        pstep_5cm=ps[0.05], pstep_10cm=ps[0.10], pstep_20cm=ps[0.20],
        pstep_30cm=ps[0.30])
keys = list(rows[0].keys()) + ['gyro_p99_deg_frame_filt', 'gyro_max_deg_frame_filt',
                               'gyro_bad_samples', 'pstep_5cm', 'pstep_10cm',
                               'pstep_20cm', 'pstep_30cm']
for r in rows:
    r.update(add.get(r['bag'], {}))
with open(os.path.join(D, 'summary.csv'), 'w', newline='') as fo:
    w = csv.DictWriter(fo, fieldnames=keys); w.writeheader()
    for r in rows: w.writerow(r)
# pooled
gz = []; tot = {5: 0, 10: 0, 20: 0, 30: 0}
for r in rows:
    tot[5] += int(r['pstep_5cm']); tot[10] += int(r['pstep_10cm'])
    tot[20] += int(r['pstep_20cm']); tot[30] += int(r['pstep_30cm'])
print('pooled pseudo-step frames (fixed, hd_ok, dp_ant<=2cm): >=5cm %d  >=10cm %d  >=20cm %d  >=30cm %d' %
      (tot[5], tot[10], tot[20], tot[30]))
print('bags with bad gyro samples:', sum(1 for r in rows if r.get('gyro_bad_samples')))
