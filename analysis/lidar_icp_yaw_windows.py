#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_icp_yaw_windows.py — event-windowed version of lidar_icp_yaw.py (P2).

Why: running frame-to-frame ICP over all 9 bags means 233548 lidar frames, which at the
measured throughput needs ~16 h wall clock.  We only need the yaw increment (a) inside a
few seconds around each RTK pseudo-step and (b) over one contiguous stretch per bag for
the sign/scale sanity check.  That is ~2.3e4 frames, i.e. ~40 min.

ICP here is strictly frame-to-frame (no accumulation across frames), so skipping the
intervals between events changes nothing about any event's measured rotation.  The only
thing lost relative to the full run is whole-bag coverage.

Segments processed per bag:
  - one window [t-PAD, t+PAD] per event of that bag in pseudo_steps.csv, overlapping
    windows merged;
  - one contiguous "sanity" segment of SANITY_FRAMES frames taken from the middle of the
    bag (default 3000 frames = 300 s at 10 Hz).
ICP state (previous cloud, transform guess) is reset at every segment boundary, so no
bogus increment is ever emitted across a gap.

Outputs (csv, same columns as lidar_icp_yaw.py plus `seg`):
  <out>_lidar.csv : t_hdr, t_bag, dt, dyaw_deg, fitness, rmse, n_src, n_tgt, seg
  <out>_rtk.csv   : t_hdr, t_bag, yaw_deg, x, y        (whole bag, cheap)
  <out>_imu.csv   : t_hdr, gz_rad_s                    (whole bag, decimated)
`seg` is the segment index; `seg` < 0 marks the sanity segment.

Usage (vehicle PC, ROS Noetic + ws_livox sourced):
  OMP_NUM_THREADS=4 python3 lidar_icp_yaw_windows.py --bag X.bag --out OUTDIR/X \
      --events fig/pseudo_steps.csv
"""
import argparse
import csv
import math
import os
import struct
import sys
import time

import numpy as np

try:
    import rosbag
    import rospy
except ImportError:
    sys.exit("rosbag not importable: source /opt/ros/noetic/setup.bash first")
try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d not importable")

# ---- identical tunables to lidar_icp_yaw.py (do not diverge) ----
R_MIN, R_MAX = 1.0, 40.0
Z_MIN, Z_MAX = -0.15, 3.0
VOXEL = 0.15
ICP_MAX_CORR = 0.6
ICP_ITERS = 30
IMU_DECIM = 5

CUSTOM_POINT = np.dtype([("off", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                         ("refl", "u1"), ("tag", "u1"), ("line", "u1")])   # 19 bytes


def parse_custommsg(buf):
    o = 0
    seq, sec, nsec = struct.unpack_from("<III", buf, o); o += 12
    (flen,) = struct.unpack_from("<I", buf, o); o += 4 + flen
    timebase, point_num, lidar_id = struct.unpack_from("<QIB", buf, o); o += 8 + 4 + 1 + 3
    (n,) = struct.unpack_from("<I", buf, o); o += 4
    pts = np.frombuffer(buf, dtype=CUSTOM_POINT, count=n, offset=o)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
    return sec + nsec * 1e-9, xyz


_PC2_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def parse_pointcloud2(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name); formats.append(_PC2_TYPES[f.datatype]); offsets.append(f.offset)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    return msg.header.stamp.to_sec(), xyz


def preprocess(xyz):
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    m = (r > R_MIN) & (r < R_MAX) & (xyz[:, 2] > Z_MIN) & (xyz[:, 2] < Z_MAX) & np.isfinite(xyz).all(1)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz[m].astype(np.float64))
    pc = pc.voxel_down_sample(VOXEL)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.6, max_nn=20))
    return pc


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def merge_intervals(iv):
    iv = sorted(iv)
    out = []
    for lo, hi in iv:
        if out and lo <= out[-1][1]:
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([lo, hi])
    return out


def event_windows(events_csv, bag_key, pad):
    ts = []
    with open(events_csv, "r", newline="") as f:
        for row in csv.DictReader(f):
            if row["bag"] == bag_key:
                ts.append(float(row["t"]))
    return merge_intervals([[t - pad, t + pad] for t in ts]), len(ts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--events", required=True, help="pseudo_steps.csv")
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--pad", type=float, default=3.0, help="+/- s around each event")
    ap.add_argument("--sanity-frames", type=int, default=3000)
    ap.add_argument("--lidar-hz", type=float, default=10.0)
    a = ap.parse_args()

    bag_key = os.path.basename(a.bag)[:-4] if a.bag.endswith(".bag") else os.path.basename(a.bag)

    bag = rosbag.Bag(a.bag, "r")
    info = bag.get_type_and_topic_info()[1]
    if a.topic not in info:
        sys.exit("topic %s not in bag" % a.topic)
    ttype = info[a.topic].msg_type
    is_custom = ttype.endswith("CustomMsg")
    t_start, t_end = bag.get_start_time(), bag.get_end_time()
    print("bag=%s topic=%s type=%s msgs=%d span=%.1fs" % (
        a.bag, a.topic, ttype, info[a.topic].message_count, t_end - t_start), flush=True)

    # ---- whole-bag rtk + imu dumps (cheap) ----
    with open(a.out + "_rtk.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_hdr", "t_bag", "yaw_deg", "x", "y"])
        for _, m, t in bag.read_messages(topics=["/rtk_odom"]):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            w.writerow([f"{m.header.stamp.to_sec():.4f}", f"{t.to_sec():.4f}", f"{math.degrees(yaw):.4f}",
                        f"{m.pose.pose.position.x:.4f}", f"{m.pose.pose.position.y:.4f}"])
    n_imu = 0
    with open(a.out + "_imu.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_hdr", "gz_rad_s"])
        for _, m, t in bag.read_messages(topics=["/livox/imu"]):
            n_imu += 1
            if n_imu % IMU_DECIM: continue
            w.writerow([f"{m.header.stamp.to_sec():.4f}", f"{m.angular_velocity.z:.6f}"])
    print("rtk/imu dumped", flush=True)

    # ---- build segments ----
    ev_iv, n_ev = event_windows(a.events, bag_key, a.pad)
    ev_iv = [[max(lo, t_start), min(hi, t_end)] for lo, hi in ev_iv]
    ev_iv = [x for x in ev_iv if x[1] - x[0] > 0.5]
    san_half = 0.5 * a.sanity_frames / a.lidar_hz
    mid = 0.5 * (t_start + t_end)
    san = [max(t_start, mid - san_half), min(t_end, mid + san_half)]
    segs = [(i, lo, hi) for i, (lo, hi) in enumerate(ev_iv)] + [(-1, san[0], san[1])]
    total_s = sum(hi - lo for _, lo, hi in segs)
    print("segments: %d event windows (from %d events) + 1 sanity, %.1f s total, ~%d frames"
          % (len(ev_iv), n_ev, total_s, int(total_s * a.lidar_hz)), flush=True)

    # ---- lidar ICP, per segment ----
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_ITERS)
    est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    n_tot = 0
    t0 = time.time()
    with open(a.out + "_lidar.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_hdr", "t_bag", "dt", "dyaw_deg", "fitness", "rmse", "n_src", "n_tgt", "seg"])
        for si, lo, hi in segs:
            prev = None; prev_t = None; T_guess = np.eye(4)   # reset at every segment boundary
            n_seg = 0
            for _, m, t in bag.read_messages(topics=[a.topic], raw=is_custom,
                                             start_time=rospy.Time.from_sec(lo),
                                             end_time=rospy.Time.from_sec(hi)):
                stamp, xyz = parse_custommsg(m[1]) if is_custom else parse_pointcloud2(m)
                pc = preprocess(xyz)
                if prev is not None and len(pc.points) > 100 and len(prev.points) > 100:
                    res = o3d.pipelines.registration.registration_icp(
                        prev, pc, ICP_MAX_CORR, T_guess, est, crit)
                    T = res.transformation
                    dyaw = -yaw_of(T)
                    T_guess = T.copy()
                    w.writerow([f"{stamp:.4f}", f"{t.to_sec():.4f}", f"{stamp - prev_t:.4f}",
                                f"{math.degrees(dyaw):.4f}", f"{res.fitness:.3f}",
                                f"{res.inlier_rmse:.4f}", len(prev.points), len(pc.points), si])
                prev, prev_t = pc, stamp
                n_seg += 1; n_tot += 1
            print("  seg %3d [%.1f,%.1f] %d frames  (cum %d, %.1f fps)"
                  % (si, lo - t_start, hi - t_start, n_seg, n_tot, n_tot / max(1e-6, time.time() - t0)),
                  flush=True)
    print("done frames=%d  %.0f s" % (n_tot, time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
