#!/usr/bin/env python3
"""Independent lidar heading reference (first preparation step for the P2 lever-arm
pseudo-step study).

Purpose: in an orchard with tree rows, estimate the vehicle heading psi_lidar
independently from the /livox/lidar point cloud and compare it with the frames that P2
classifies as pseudo-steps.  If the lidar heading does not jump at the same time, the
step is in the published position only, not in the real attitude.

Verified: the 2026-09-06 showcase event (E3b) is the induced-multipath experiment run in
a greenhouse, not an orchard, and has no tree rows, so this script does not apply to it;
it is valid only for orchard runs.  Check that the `site` field in fig/pseudo_steps.csv
says orchard before picking a bag.

Topic (checked on the 2026-09-06 greenhouse bag; orchard bags TO VERIFY):
  /livox/lidar   livox_ros_driver2/CustomMsg   ~9.8 Hz (998 s / 9763 msg)
  Points are msg.points[i].x/y/z (body frame, metres); line/tag/offset_time also present.

Usage:
  python3 lidar_heading_ref.py --bag RUN.bag --topic /livox/lidar --out out.csv

Needs a ROS Noetic environment with livox_ros_driver2 and the bag files, i.e. the
vehicle PC; it does not run on a workstation without ROS.
"""
import argparse
import csv
import sys

import numpy as np

# -------- parameters (collected here; check against the site before a run) --------
SIDE_MIN_M = 1.0      # body-frame |y| lower bound: excludes the vehicle itself and near noise
SIDE_MAX_M = 4.0      # body-frame |y| upper bound: drops points beyond the tree-row spacing
HEIGHT_MIN_M = 0.30    # body-frame z lower bound: avoids the ground (extrinsics must match the lidar mounting height)
HEIGHT_MAX_M = 2.00    # body-frame z upper bound: avoids sparse canopy tops and antenna-mast returns
MIN_INLIERS = 30       # minimum RANSAC inliers; below this the frame is marked as a fit failure
RANSAC_ITERS = 200
RANSAC_THRESH_M = 0.08  # point-to-line distance threshold


def fit_line_ransac(pts_xy, iters=RANSAC_ITERS, thresh=RANSAC_THRESH_M, rng=None):
    """RANSAC line fit on one side's points (N,2); returns (direction angle in rad, inlier count) or (None, 0)."""
    n = pts_xy.shape[0]
    if n < 2:
        return None, 0
    rng = rng or np.random.default_rng(0)
    best_inliers = -1
    best_dir = None
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        p1, p2 = pts_xy[i], pts_xy[j]
        d = p2 - p1
        norm = np.hypot(*d)
        if norm < 1e-6:
            continue
        d = d / norm
        normal = np.array([-d[1], d[0]])
        dist = np.abs((pts_xy - p1) @ normal)
        inliers = int((dist < thresh).sum())
        if inliers > best_inliers:
            best_inliers = inliers
            best_dir = d
    if best_dir is None or best_inliers < MIN_INLIERS:
        return None, best_inliers if best_inliers > 0 else 0
    # refine the direction with one PCA over the inliers
    normal = np.array([-best_dir[1], best_dir[0]])
    p0 = pts_xy[0]
    dist = np.abs((pts_xy - p0) @ normal)
    inl_pts = pts_xy[dist < thresh]
    if inl_pts.shape[0] >= 2:
        centered = inl_pts - inl_pts.mean(axis=0)
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        refined = vt[0]
        if refined @ best_dir < 0:
            refined = -refined
        best_dir = refined
    angle = float(np.arctan2(best_dir[1], best_dir[0]))
    return angle, best_inliers


def process_bag(bag_path, topic, out_path):
    try:
        import rosbag  # noqa: F401  (only present in the ROS environment on the vehicle PC)
    except ImportError:
        print("Needs a ROS Noetic environment (source /opt/ros/noetic/setup.bash); "
              "rosbag is not installed here, run this on the vehicle PC.", file=sys.stderr)
        sys.exit(1)
    import rosbag

    with open(out_path, "w", newline="") as f, rosbag.Bag(bag_path) as bag:
        writer = csv.writer(f)
        writer.writerow(["t", "psi_lidar_rad", "left_inliers", "right_inliers", "left_ok", "right_ok"])
        for _, msg, t in bag.read_messages(topics=[topic]):
            pts = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float64)
            if pts.shape[0] == 0:
                continue
            mask_h = (pts[:, 2] > HEIGHT_MIN_M) & (pts[:, 2] < HEIGHT_MAX_M)
            mask_side = (np.abs(pts[:, 1]) > SIDE_MIN_M) & (np.abs(pts[:, 1]) < SIDE_MAX_M)
            pts = pts[mask_h & mask_side]
            if pts.shape[0] < 2 * MIN_INLIERS:
                writer.writerow([t.to_sec(), "", 0, 0, False, False])
                continue
            left = pts[pts[:, 1] > 0][:, :2]
            right = pts[pts[:, 1] < 0][:, :2]
            ang_l, n_l = fit_line_ransac(left)
            ang_r, n_r = fit_line_ransac(right)
            angles = [a for a in (ang_l, ang_r) if a is not None]
            if not angles:
                writer.writerow([t.to_sec(), "", n_l, n_r, False, False])
                continue
            # the two sides can differ by pi (opposite row directions); wrap to [-pi/2, pi/2)
            norm_angles = []
            for a in angles:
                while a >= np.pi / 2:
                    a -= np.pi
                while a < -np.pi / 2:
                    a += np.pi
                norm_angles.append(a)
            psi = float(np.mean(norm_angles))
            writer.writerow([t.to_sec(), psi, n_l, n_r, ang_l is not None, ang_r is not None])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    process_bag(args.bag, args.topic, args.out)
    print(f"done -> {args.out}")


if __name__ == "__main__":
    main()
