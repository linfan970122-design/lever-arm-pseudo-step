#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_icp_yaw.py — frame-to-frame lidar yaw increment (third-party heading reference for P2)

Reads /livox/lidar (livox_ros_driver2/CustomMsg or sensor_msgs/PointCloud2) with raw
deserialisation (numpy, no per-point python loop), voxel-downsamples each frame, runs
open3d point-to-plane ICP against the previous frame and writes the yaw increment.
Also dumps /rtk_odom yaw and /livox/imu gyro-z so the comparison can be done offline.

Outputs (csv):
  <out>_lidar.csv : t_hdr, t_bag, dt, dyaw_deg, fitness, rmse, n_src, n_tgt
  <out>_rtk.csv   : t_hdr, t_bag, yaw_deg
  <out>_imu.csv   : t_hdr, gz_rad_s        (downsampled to every N-th sample)

Usage (on the vehicle PC, ROS Noetic + ws_livox sourced):
  python3 lidar_icp_yaw.py --bag X.bag --out OUTDIR/X [--start T0 --end T1] [--max-frames N]
Yaw increment sign: +ve = counter-clockwise viewed from above (same as ENU heading).
"""
import argparse
import csv
import math
import struct
import sys
import time

import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("rosbag not importable: source /opt/ros/noetic/setup.bash first")
try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d not importable")

# ---------------- tunables ----------------
R_MIN, R_MAX = 1.0, 40.0        # keep points in this horizontal range (m); <1 m is the car body
Z_MIN, Z_MAX = -0.15, 3.0       # lidar frame; ground is at about z=-0.30 (mount height)
VOXEL = 0.15                    # m
ICP_MAX_CORR = 0.6              # m
ICP_ITERS = 30
IMU_DECIM = 5                   # keep every 5th imu sample (200 Hz -> 40 Hz)

CUSTOM_POINT = np.dtype([("off", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                         ("refl", "u1"), ("tag", "u1"), ("line", "u1")])   # 19 bytes


def parse_custommsg(buf):
    """livox_ros_driver2/CustomMsg raw bytes -> (stamp_sec, xyz float32 [N,3])"""
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True, help="output prefix")
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--start", type=float, default=None, help="unix s (bag time)")
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()

    bag = rosbag.Bag(a.bag, "r")
    info = bag.get_type_and_topic_info()[1]
    if a.topic not in info:
        sys.exit("topic %s not in bag" % a.topic)
    ttype = info[a.topic].msg_type
    is_custom = ttype.endswith("CustomMsg")
    print("bag=%s topic=%s type=%s msgs=%d" % (a.bag, a.topic, ttype, info[a.topic].message_count), flush=True)

    import rospy
    kw = {}
    if a.start is not None: kw["start_time"] = rospy.Time.from_sec(a.start)
    if a.end is not None: kw["end_time"] = rospy.Time.from_sec(a.end)

    # ---- rtk + imu dumps (cheap) ----
    with open(a.out + "_rtk.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_hdr", "t_bag", "yaw_deg", "x", "y"])
        for _, m, t in bag.read_messages(topics=["/rtk_odom"], **kw):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            w.writerow([f"{m.header.stamp.to_sec():.4f}", f"{t.to_sec():.4f}", f"{math.degrees(yaw):.4f}",
                        f"{m.pose.pose.position.x:.4f}", f"{m.pose.pose.position.y:.4f}"])
    n_imu = 0
    with open(a.out + "_imu.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_hdr", "gz_rad_s"])
        for _, m, t in bag.read_messages(topics=["/livox/imu"], **kw):
            n_imu += 1
            if n_imu % IMU_DECIM: continue
            w.writerow([f"{m.header.stamp.to_sec():.4f}", f"{m.angular_velocity.z:.6f}"])
    print("rtk/imu dumped", flush=True)

    # ---- lidar ICP ----
    prev = None; prev_t = None; T_guess = np.eye(4)
    n = 0; t0 = time.time()
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_ITERS)
    est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    with open(a.out + "_lidar.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_hdr", "t_bag", "dt", "dyaw_deg", "fitness", "rmse", "n_src", "n_tgt"])
        for _, m, t in bag.read_messages(topics=[a.topic], raw=is_custom, **kw):
            if is_custom:
                stamp, xyz = parse_custommsg(m[1])
            else:
                stamp, xyz = parse_pointcloud2(m)
            pc = preprocess(xyz)
            if prev is not None and len(pc.points) > 100 and len(prev.points) > 100:
                # T maps source(prev) -> target(current): rotation = car yaw change (sign-flipped)
                res = o3d.pipelines.registration.registration_icp(
                    prev, pc, ICP_MAX_CORR, T_guess, est, crit)
                T = res.transformation
                # prev cloud expressed in the new body frame: body rotated by +dyaw => points rotate by -dyaw
                dyaw = -yaw_of(T)
                T_guess = T.copy()
                w.writerow([f"{stamp:.4f}", f"{t.to_sec():.4f}", f"{stamp - prev_t:.4f}", f"{math.degrees(dyaw):.4f}",
                            f"{res.fitness:.3f}", f"{res.inlier_rmse:.4f}", len(prev.points), len(pc.points)])
            prev, prev_t = pc, stamp
            n += 1
            if n % 500 == 0:
                print("frames %d  %.1f fps" % (n, n / (time.time() - t0)), flush=True)
            if a.max_frames and n >= a.max_frames:
                break
    print("done frames=%d  %.0f s" % (n, time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
