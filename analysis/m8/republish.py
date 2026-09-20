#!/usr/bin/env python
"""M8 replay: turn the recorded /rtk_odom + /ch110/data_raw into the raw inputs
that robot_localization's stack expects, optionally injecting synthetic heading
outliers.

The recorded driver already applied the lever arm, so the antenna position is
recovered by inverting it (same inversion as Section 4.2 of the manuscript):

    p_ant_enu = p_pub_enu + R_z(yaw) * L ,  L = (lever_x, lever_y, lever_z)

and then converted back to lat/lon/alt with the ENU origin the driver used.
The published topics are

    /gps/fix          sensor_msgs/NavSatFix   antenna LLH, frame 'gps'   (5 Hz)
    /rtk_heading_imu  sensor_msgs/Imu         RTK heading as orientation (5 Hz)
    /ch110_imu        sensor_msgs/Imu         CH110 yaw rate             (~93 Hz)
    /navsat_imu       sensor_msgs/Imu         heading + CH110 yaw rate   (~93 Hz)

/rtk_heading_imu is the EKF's absolute-yaw input and is the ONLY stream the
injection touches; the antenna position is never perturbed, because a lever-arm
pseudo-step is a heading-only fault.  /navsat_imu is what navsat_transform
subscribes to as imu/data: its orientation is used once for the datum, its
angular_velocity.z is what the patch's gyro reference integrates.

Nothing here reads or writes the operational configuration; the ENU origin is
read from the driver's origin file and never logged.
"""
import csv
import math
import os

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def llh_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return ((n + alt_m) * math.cos(lat) * math.cos(lon),
            (n + alt_m) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - WGS84_E2) + alt_m) * s)


def ecef_to_llh(x, y, z):
    """Bowring, good to well below a millimetre for terrestrial points."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    b = WGS84_A * (1.0 - WGS84_F)
    ep2 = (WGS84_A ** 2 - b ** 2) / b ** 2
    th = math.atan2(WGS84_A * z, b * p)
    lat = math.atan2(z + ep2 * b * math.sin(th) ** 3,
                     p - WGS84_E2 * WGS84_A * math.cos(th) ** 3)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def enu_to_llh(e, n, u, lat0, lon0, alt0):
    x0, y0, z0 = llh_to_ecef(lat0, lon0, alt0)
    la, lo = math.radians(lat0), math.radians(lon0)
    sl, cl = math.sin(la), math.cos(la)
    so, co = math.sin(lo), math.cos(lo)
    dx = -so * e - sl * co * n + cl * co * u
    dy = co * e - sl * so * n + cl * so * u
    dz = cl * n + sl * u
    return ecef_to_llh(x0 + dx, y0 + dy, z0 + dz)


def load_origin(path):
    origin = {}
    with open(path) as fp:
        for line in fp:
            line = line.split('#')[0].strip()
            if ':' in line:
                k, _, v = line.partition(':')
                try:
                    origin[k.strip()] = float(v.strip())
                except ValueError:
                    pass
    for k in ('lat', 'lon', 'alt'):
        if k not in origin:
            raise RuntimeError('origin file lacks %s' % k)
    return origin


def load_events(path):
    if not path or not os.path.isfile(path):
        return []
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            out.append(dict(event_id=r['event_id'], t_abs=float(r['t_abs']),
                            amp=math.radians(float(r['amp_deg'])),
                            dur=int(r['dur_frames'])))
    out.sort(key=lambda e: e['t_abs'])
    return out


class Republisher(object):
    def __init__(self):
        self.lever = (rospy.get_param('~lever_arm_x', 0.235),
                      rospy.get_param('~lever_arm_y', -0.280),
                      rospy.get_param('~lever_arm_z', -0.175))
        self.origin = load_origin(rospy.get_param(
            '~origin_file', os.path.expanduser('~/.ros/rtk_origin.yaml')))
        self.events = load_events(rospy.get_param('~events_file', ''))
        self.inject = bool(rospy.get_param('~inject', False))
        self.log_path = rospy.get_param('~heading_log', '')
        self.gps_frame = rospy.get_param('~gps_frame', 'gps')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        self.ev_i = 0
        self.ev_left = 0
        self.ev_amp = 0.0
        self.ev_id = ''
        self.gz = 0.0
        self.gz_cum = 0.0          # same rectangular integration the C++ gyroCallback does
        self.gz_t = None
        self.rows = []

        self.pub_fix = rospy.Publisher('gps/fix', NavSatFix, queue_size=50)
        self.pub_hdg = rospy.Publisher('rtk_heading_imu', Imu, queue_size=50)
        self.pub_gyro = rospy.Publisher('ch110_imu', Imu, queue_size=200)
        self.pub_nav = rospy.Publisher('navsat_imu', Imu, queue_size=200)

        rospy.Subscriber('/ch110/data_raw', Imu, self.cb_imu, queue_size=400)
        rospy.Subscriber('/rtk_odom', Odometry, self.cb_rtk, queue_size=100)
        rospy.on_shutdown(self.dump)

    # ---------------------------------------------------------------- IMU
    def cb_imu(self, msg):
        self.gz = msg.angular_velocity.z
        ts = msg.header.stamp.to_sec()
        if self.gz_t is not None and ts > self.gz_t:
            self.gz_cum += self.gz * (ts - self.gz_t)
        self.gz_t = ts
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
        out.orientation_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pub_gyro.publish(out)

        nav = Imu()
        nav.header = out.header
        nav.angular_velocity = msg.angular_velocity
        nav.angular_velocity_covariance = out.angular_velocity_covariance
        q = tft.quaternion_from_euler(0.0, 0.0, getattr(self, 'last_yaw', 0.0))
        nav.orientation.x, nav.orientation.y = q[0], q[1]
        nav.orientation.z, nav.orientation.w = q[2], q[3]
        nav.orientation_covariance = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 7.61544e-05]
        self.pub_nav.publish(nav)

    # ---------------------------------------------------------------- RTK
    def cb_rtk(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.last_yaw = yaw

        # invert the driver's lever-arm compensation to recover the antenna point
        cy, sy = math.cos(yaw), math.sin(yaw)
        lx, ly, lz = self.lever
        e = p.x + (cy * lx - sy * ly)
        n = p.y + (sy * lx + cy * ly)
        u = p.z + lz
        lat, lon, alt = enu_to_llh(e, n, u, self.origin['lat'],
                                   self.origin['lon'], self.origin['alt'])

        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self.gps_frame
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = lat, lon, alt
        c = msg.pose.covariance[0] if msg.pose.covariance[0] > 0 else 9e-4
        fix.position_covariance = [c, 0, 0, 0, c, 0, 0, 0, 4.0 * c]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        # ---- injection (heading stream only) ----
        amp = 0.0
        ev_id = ''
        if self.inject:
            # events already past when playback started are skipped, not fired late
            while (self.ev_i < len(self.events)
                   and self.events[self.ev_i]['t_abs'] < t - 0.3):
                self.ev_i += 1
            while (self.ev_left == 0 and self.ev_i < len(self.events)
                   and t >= self.events[self.ev_i]['t_abs']):
                ev = self.events[self.ev_i]
                self.ev_i += 1
                self.ev_left = ev['dur']
                self.ev_amp = ev['amp']
                self.ev_id = ev['event_id']
            if self.ev_left > 0:
                amp = self.ev_amp
                ev_id = self.ev_id
                self.ev_left -= 1
        yaw_out = math.atan2(math.sin(yaw + amp), math.cos(yaw + amp))

        hdg = Imu()
        hdg.header.stamp = msg.header.stamp
        hdg.header.frame_id = self.base_frame
        qq = tft.quaternion_from_euler(0.0, 0.0, yaw_out)
        hdg.orientation.x, hdg.orientation.y = qq[0], qq[1]
        hdg.orientation.z, hdg.orientation.w = qq[2], qq[3]
        # the driver's declared heading sigma, 0.5 deg
        hdg.orientation_covariance = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 7.61544e-05]
        hdg.angular_velocity_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
        hdg.linear_acceleration_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pub_hdg.publish(hdg)

        self.rows.append((('%.6f' % t), '%.6f' % yaw, '%.6f' % yaw_out,
                          '%.6f' % amp, ev_id, '%.6f' % self.gz,
                          '%.4f' % p.x, '%.4f' % p.y, '%.6f' % self.gz_cum))

    def dump(self):
        if not self.log_path:
            return
        with open(self.log_path, 'w') as f:
            f.write('t,yaw_true,yaw_out,inj_rad,event_id,gz,x_rtk,y_rtk,gz_cum\n')
            for r in self.rows:
                f.write(','.join(r) + '\n')


if __name__ == '__main__':
    rospy.init_node('m8_republish')
    Republisher()
    rospy.spin()
