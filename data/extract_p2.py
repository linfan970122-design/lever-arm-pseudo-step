#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 lever-arm pseudo-step extraction. Read-only on bags; writes only under $P2_OUT.

Replicates the deployed conversion in rtk_odom.py (L594-609):
    veh_heading = (hdt + 180) % 360        # heading_offset_deg = 180 (launch L35)
    yaw         = radians(90 - veh_heading)
    p_pub = p_ant - R(yaw) * (Lx, Ly),  Lx=0.235, Ly=-0.280   (z: u -= Lz, no rotation)
so the antenna point is reconstructed as
    p_ant = p_pub + R(yaw) * (Lx, Ly)
    x_ant = x + cos(yaw)*Lx - sin(yaw)*Ly
    y_ant = y + sin(yaw)*Lx + cos(yaw)*Ly
Only valid when the driver actually applied the compensation, i.e. hd_ok
(cov[35] < 1e5). When cov[35] == 1e6 the driver published the raw ANT1 point
with yaw = 0, so p_ant = p_pub there.

Usage: extract_p2.py <bag> [<bag> ...]
Output directory: $P2_OUT (default ./out).  ENU origin file: $P2_ORIGIN_FILE
(default ~/.ros/rtk_origin.yaml, the file the driver writes on the vehicle).
"""
import sys, os, math, re, bisect, time, csv

import rosbag

LX, LY, LZ = 0.235, -0.280, -0.175
LNORM = math.hypot(LX, LY)
W = 3.0                       # window used for straight/turn classification
SLOW_DISP = 0.5               # m, disp_stats.py rule
STRAIGHT_DPSI = 5.0           # deg, disp_stats.py rule
OUT = os.environ.get('P2_OUT', 'out')
ORIGIN_FILE = os.path.expanduser(
    os.environ.get('P2_ORIGIN_FILE', '~/.ros/rtk_origin.yaml'))

# The driver prints its diagnostic line with Chinese field names; these patterns
# match that recorded text verbatim and must not be translated.
RX_HDG = re.compile(r"航向=(nan|-?[\d.]+)")          # heading=
RX_SATS = re.compile(r"星=(\d+)")                    # satellites=
RX_AGE = re.compile(r"龄期=(-?[\d.]+)")              # correction age=
RX_RATE = re.compile(r"固定率=([\d.]+)")             # fixed-solution rate=
RX_Q = re.compile(r"解=([^(]*)\((\d+)\)")           # solution type=

TOP_ODOM = '/rtk_odom'
TOP_STAT = '/rtk/status'
TOP_FIX = '/rtk/fix'
IMU_CANDIDATES = ['/ch110/data_raw', '/imu/data', '/ch110/imu', '/imu/data_raw']

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def llh_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg); lon = math.radians(lon_deg)
    s = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return ((n + alt_m) * math.cos(lat) * math.cos(lon),
            (n + alt_m) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - WGS84_E2) + alt_m) * s)


def llh_to_enu(lat, lon, alt, lat0, lon0, alt0):
    x, y, z = llh_to_ecef(lat, lon, alt)
    x0, y0, z0 = llh_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    la = math.radians(lat0); lo = math.radians(lon0)
    sl, cl = math.sin(la), math.cos(la)
    so, co = math.sin(lo), math.cos(lo)
    return (-so * dx + co * dy,
            -sl * co * dx - sl * so * dy + cl * dz,
            cl * co * dx + cl * so * dy + sl * dz)


def read_origin():
    try:
        lat = lon = alt = None
        for line in open(ORIGIN_FILE):
            line = line.strip()
            if line.startswith('lat:'): lat = float(line.split(':')[1])
            elif line.startswith('lon:'): lon = float(line.split(':')[1])
            elif line.startswith('alt:'): alt = float(line.split(':')[1])
        if None not in (lat, lon, alt):
            return lat, lon, alt
    except Exception:
        pass
    return None


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def quant(v, p):
    if not v: return float('nan')
    s = sorted(v); k = (len(s) - 1) * p
    i = int(k); f = k - i
    return s[i] if i + 1 >= len(s) else s[i] * (1 - f) + s[i + 1] * f


def parse_status(txt):
    d = {'quality': '', 'sats': '', 'age': '', 'fixrate': '', 'hdg': '',
         'fixed': 0}
    m = RX_Q.search(txt)
    if m: d['quality'] = m.group(2)
    m = RX_SATS.search(txt)
    if m: d['sats'] = m.group(1)
    m = RX_AGE.search(txt)
    if m: d['age'] = m.group(1)
    m = RX_RATE.search(txt)
    if m: d['fixrate'] = m.group(1)
    m = RX_HDG.search(txt)
    if m: d['hdg'] = m.group(1)
    d['fixed'] = 1 if '固定解' in txt else 0  # '固定解' = "fixed solution" in the driver's recorded log line
    return d


def interp_xy(seq, ts, t):
    i = bisect.bisect_left(ts, t)
    if i == 0 or i >= len(seq): return None
    (a, x0, y0), (c, x1, y1) = seq[i - 1][:3], seq[i][:3]
    r = (t - a) / (c - a) if c > a else 0.0
    return (x0 + r * (x1 - x0), y0 + r * (y1 - y0))


def interp_hdg(hd, th, t):
    i = bisect.bisect_left(th, t)
    if i == 0 or i >= len(hd): return None
    (a, h0), (c, h1) = hd[i - 1], hd[i]
    if min(abs(t - a), abs(t - c)) > 2.0: return None
    d = wrap180(h1 - h0)
    r = (t - a) / (c - a) if c > a else 0.0
    return (h0 + r * d) % 360.0


def process(path, origin_global, summary_rows):
    name = os.path.basename(path)
    t_start = time.time()
    bag = rosbag.Bag(path, 'r')
    info = bag.get_type_and_topic_info()[1]
    have = set(info.keys())
    if TOP_ODOM not in have:
        bag.close()
        print('  SKIP: no %s' % TOP_ODOM); sys.stdout.flush()
        return 'skip'
    imu_topic = None
    for c in IMU_CANDIDATES:
        if c in have and info[c].msg_type == 'sensor_msgs/Imu':
            imu_topic = c; break
    has_fix = TOP_FIX in have
    topics = [TOP_ODOM, TOP_STAT] + ([TOP_FIX] if has_fix else []) \
             + ([imu_topic] if imu_topic else [])

    odom = []      # (t, x, y, z, yaw, cov0, cov35)
    stat = []      # (t, dict)
    fixs = []      # (t, lat, lon, alt)
    imu_t = []; imu_gz = []
    for tp, m, t in bag.read_messages(topics=topics):
        ts = t.to_sec()
        if tp == TOP_ODOM:
            p = m.pose.pose.position; q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            cov = m.pose.covariance
            odom.append((ts, p.x, p.y, p.z, yaw, cov[0], cov[35]))
        elif tp == TOP_STAT:
            stat.append((ts, parse_status(m.data)))
        elif tp == TOP_FIX:
            fixs.append((ts, m.latitude, m.longitude, m.altitude))
        elif imu_topic and tp == imu_topic:
            imu_t.append(ts); imu_gz.append(m.angular_velocity.z)
    bag.close()

    if len(odom) < 2:
        print('  SKIP: only %d %s frames' % (len(odom), TOP_ODOM))
        sys.stdout.flush(); return 'skip'

    t_odom = [o[0] for o in odom]
    t_stat = [s[0] for s in stat]
    hd = [(s[0], float(s[1]['hdg'])) for s in stat
          if s[1]['hdg'] not in ('', 'nan')]
    t_hd = [h[0] for h in hd]
    # RTK xy sequence for window displacement
    rtk_xy = [(o[0], o[1], o[2]) for o in odom]

    # fix -> ENU, same origin as the driver if available
    fix_enu = []
    origin_used = ''
    if has_fix and fixs:
        if origin_global:
            o = origin_global; origin_used = 'rtk_origin.yaml'
        else:
            o = (fixs[0][1], fixs[0][2], fixs[0][3]); origin_used = 'first_fix'
        for (ts, la, lo, al) in fixs:
            e, n, u = llh_to_enu(la, lo, al, o[0], o[1], o[2])
            fix_enu.append((ts, e, n))
    t_fix = [f[0] for f in fix_enu]

    # cumulative IMU yaw (rad) for independent turn classification
    if imu_t:
        cum = [0.0] * len(imu_t)
        for i in range(1, len(imu_t)):
            dt = imu_t[i] - imu_t[i - 1]
            if dt <= 0 or dt > 0.5: dt = 0.0
            cum[i] = cum[i - 1] + imu_gz[i] * dt
    else:
        cum = []

    def imu_yaw_at(t):
        if not imu_t: return None
        i = bisect.bisect_left(imu_t, t)
        if i == 0 or i >= len(imu_t): return None
        a, c = imu_t[i - 1], imu_t[i]
        r = (t - a) / (c - a) if c > a else 0.0
        return cum[i - 1] + r * (cum[i] - cum[i - 1])

    def imu_gzmax(t0, t1):
        if not imu_t: return None
        i = bisect.bisect_left(imu_t, t0); j = bisect.bisect_right(imu_t, t1)
        if j <= i: return None
        return max(abs(g) for g in imu_gz[i:j])

    hdr = ('bag,t,dt,x,y,z,yaw_deg,hdg_deg,dpsi_deg,dp_pub,x_ant,y_ant,dp_ant,'
           'pred_jump,resid,along,across,theta_body_deg,cov00,cov35,hd_ok,'
           'quality,sats,age_s,fixrate,hdg_txt,dt_status,fixed,'
           'x_fix,y_fix,dp_fix,dpsi_imu_deg,gz_max,cls,cls_imu\n')
    fo = open(os.path.join(OUT, 'csv', name.replace('.bag', '') + '.csv'), 'w')
    fo.write(hdr)

    allv = []; cat = {'straight': [], 'turn': [], 'slow': [], 'na': []}
    cat_imu = {'straight': [], 'turn': [], 'na': []}
    n_gt = {5: 0, 10: 0, 20: 0, 45: 0}
    fixed_frames = 0; hdok_false = 0
    max_dp_pub = 0.0; max_dp_ant = 0.0; max_dp_fix = 0.0
    gz_all = []

    px = py = pyaw = pt = None
    pxa = pya = None
    pfix = None
    for (ts, x, y, z, yaw, c0, c35) in odom:
        hd_ok = 0 if c35 > 1e5 else 1
        if not hd_ok: hdok_false += 1
        yaw_deg = math.degrees(yaw)
        hdg_deg = (90.0 - yaw_deg) % 360.0
        cy, sy = math.cos(yaw), math.sin(yaw)
        if hd_ok:
            xa = x + (cy * LX - sy * LY)
            ya = y + (sy * LX + cy * LY)
        else:
            xa, ya = x, y      # driver skipped the compensation
        # nearest status
        st = {'quality': '', 'sats': '', 'age': '', 'fixrate': '', 'hdg': '',
              'fixed': 0}
        dts = ''
        if t_stat:
            i = bisect.bisect_left(t_stat, ts)
            cand = [k for k in (i - 1, i) if 0 <= k < len(t_stat)]
            k = min(cand, key=lambda k: abs(t_stat[k] - ts))
            st = stat[k][1]; dts = '%.2f' % (ts - t_stat[k])
            if st['fixed'] and abs(t_stat[k] - ts) <= 1.5: fixed_frames += 1
        # fix nearest
        xf = yf = ''
        dpf = ''
        cur_fix = None
        if t_fix:
            i = bisect.bisect_left(t_fix, ts)
            cand = [k for k in (i - 1, i) if 0 <= k < len(t_fix)]
            k = min(cand, key=lambda k: abs(t_fix[k] - ts))
            if abs(t_fix[k] - ts) <= 0.15:
                cur_fix = (fix_enu[k][1], fix_enu[k][2])
                xf = '%.3f' % cur_fix[0]; yf = '%.3f' % cur_fix[1]

        if pt is None:
            dpsi = dp = dpa = pred = resid = along = across = thb = float('nan')
            dt = float('nan')
        else:
            dt = ts - pt
            dpsi = wrap180(hdg_deg - phdg)
            dx, dy = x - px, y - py
            dp = math.hypot(dx, dy)
            dpa = math.hypot(xa - pxa, ya - pya)
            pred = 2.0 * LNORM * math.sin(math.radians(abs(dpsi)) / 2.0)
            resid = dp - pred
            along = cy * dx + sy * dy          # rotate ENU->body by -yaw
            across = -sy * dx + cy * dy
            thb = math.degrees(math.atan2(across, along))
            if cur_fix and pfix:
                dpf = '%.4f' % math.hypot(cur_fix[0] - pfix[0],
                                          cur_fix[1] - pfix[1])
                max_dp_fix = max(max_dp_fix, float(dpf))
            max_dp_pub = max(max_dp_pub, dp)
            max_dp_ant = max(max_dp_ant, dpa)

        # imu per-frame integrated yaw
        dpsi_imu = ''
        gzm = ''
        if imu_t and pt is not None:
            a = imu_yaw_at(pt); b = imu_yaw_at(ts)
            if a is not None and b is not None:
                # IMU yaw is CCW-positive (ENU); heading is CW-positive
                dpsi_imu = '%.3f' % (-math.degrees(b - a))
            g = imu_gzmax(pt, ts)
            if g is not None:
                gzm = '%.4f' % g; gz_all.append(g)

        # classification, disp_stats.py rule on a 3 s window ending at t
        cls = 'na'
        if ts - odom[0][0] >= W:
            a0 = interp_xy(rtk_xy, t_odom, ts - W); a1 = (x, y)
            if a0:
                d = math.hypot(a1[0] - a0[0], a1[1] - a0[1])
                h0 = interp_hdg(hd, t_hd, ts - W)
                h1 = interp_hdg(hd, t_hd, ts)
                dh = None if (h0 is None or h1 is None) else wrap180(h1 - h0)
                if d <= SLOW_DISP: cls = 'slow'
                elif dh is not None and abs(dh) < STRAIGHT_DPSI: cls = 'straight'
                elif dh is None: cls = 'na'
                else: cls = 'turn'
        cls_imu = 'na'
        if imu_t and ts - odom[0][0] >= W:
            a = imu_yaw_at(ts - W); b = imu_yaw_at(ts)
            if a is not None and b is not None:
                cls_imu = 'straight' if abs(math.degrees(b - a)) < STRAIGHT_DPSI \
                          else 'turn'

        if pt is not None:
            av = abs(dpsi)
            allv.append(av)
            cat[cls].append(av)
            cat_imu[cls_imu].append(av)
            for k in n_gt:
                if av > k: n_gt[k] += 1

        fo.write('%s,%.3f,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%s,%s,%.3f,%.3f,%s,'
                 '%s,%s,%s,%s,%s,%.6g,%.6g,%d,%s,%s,%s,%s,%s,%s,%d,'
                 '%s,%s,%s,%s,%s,%s,%s\n' % (
            name, ts, ('%.3f' % dt) if dt == dt else '',
            x, y, z, yaw_deg, hdg_deg,
            ('%.4f' % dpsi) if dpsi == dpsi else '',
            ('%.4f' % dp) if pt is not None else '',
            xa, ya, ('%.4f' % dpa) if pt is not None else '',
            ('%.4f' % pred) if pt is not None else '',
            ('%.4f' % resid) if pt is not None else '',
            ('%.4f' % along) if pt is not None else '',
            ('%.4f' % across) if pt is not None else '',
            ('%.2f' % thb) if pt is not None else '',
            c0, c35, hd_ok,
            st['quality'], st['sats'], st['age'], st['fixrate'], st['hdg'], dts,
            st['fixed'], xf, yf, dpf, dpsi_imu, gzm, cls, cls_imu))
        px, py, pyaw, pt, phdg = x, y, yaw, ts, hdg_deg
        pxa, pya = xa, ya
        if cur_fix: pfix = cur_fix
    fo.close()

    dur = odom[-1][0] - odom[0][0]
    imu_hz = (len(imu_t) / dur) if (imu_t and dur > 0) else 0.0
    row = {
        'bag': name, 'n_frames': len(odom), 'duration_s': '%.1f' % dur,
        'odom_hz': '%.2f' % (len(odom) / dur if dur > 0 else 0),
        'fixed_frames': fixed_frames, 'hdok_false_frames': hdok_false,
        'has_fix': int(bool(t_fix)), 'fix_origin': origin_used,
        'imu_topic': imu_topic or '', 'imu_hz': '%.1f' % imu_hz,
        'dpsi_med': '%.4f' % quant(allv, .5),
        'dpsi_p90': '%.4f' % quant(allv, .9),
        'dpsi_p99': '%.4f' % quant(allv, .99),
        'dpsi_max': '%.4f' % (max(allv) if allv else float('nan')),
    }
    for k in ('straight', 'turn', 'slow', 'na'):
        v = cat[k]
        row['n_' + k] = len(v)
        row[k + '_p99'] = '%.4f' % quant(v, .99)
        row[k + '_max'] = '%.4f' % (max(v) if v else float('nan'))
        row[k + '_med'] = '%.4f' % quant(v, .5)
    for k in ('straight', 'turn'):
        v = cat_imu[k]
        row['n_imu_' + k] = len(v)
        row['imu_' + k + '_p99'] = '%.4f' % quant(v, .99)
        row['imu_' + k + '_max'] = '%.4f' % (max(v) if v else float('nan'))
    for k in (5, 10, 20, 45):
        row['n_gt%d' % k] = n_gt[k]
    row['max_dp_pub'] = '%.4f' % max_dp_pub
    row['max_dp_ant'] = '%.4f' % max_dp_ant
    row['max_dp_fix'] = '%.4f' % max_dp_fix
    # physical per-frame yaw bound from gyro: |gz| * 0.2 s at 5 Hz
    if gz_all:
        row['gyro_p99_deg_frame'] = '%.4f' % (math.degrees(quant(gz_all, .99)) * 0.2)
        row['gyro_max_deg_frame'] = '%.4f' % (math.degrees(max(gz_all)) * 0.2)
    else:
        row['gyro_p99_deg_frame'] = ''
        row['gyro_max_deg_frame'] = ''
    row['runtime_s'] = '%.1f' % (time.time() - t_start)
    summary_rows.append(row)
    print('  %d frames, %.1f s, %.1f s runtime, fix=%d imu=%s, '
          'max|dpsi|=%s max dp_pub=%s max dp_ant=%s'
          % (len(odom), dur, time.time() - t_start, bool(t_fix),
             imu_topic or '-', row['dpsi_max'], row['max_dp_pub'],
             row['max_dp_ant']))
    sys.stdout.flush()
    return 'ok'


def main():
    os.makedirs(os.path.join(OUT, 'csv'), exist_ok=True)
    origin = read_origin()
    print('ENU origin from %s: %s' % (ORIGIN_FILE, origin))
    rows = []; skipped = []
    for p in sys.argv[1:]:
        print('==== %s' % os.path.basename(p)); sys.stdout.flush()
        try:
            r = process(p, origin, rows)
            if r == 'skip': skipped.append(os.path.basename(p))
        except Exception as e:
            import traceback; traceback.print_exc()
            skipped.append(os.path.basename(p) + ' ERR:' + str(e))
    if rows:
        keys = list(rows[0].keys())
        with open(os.path.join(OUT, 'summary.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows: w.writerow(r)
    print('done: %d ok, %d skipped' % (len(rows), len(skipped)))
    for s in skipped: print('  skipped:', s)


if __name__ == '__main__':
    main()
