#!/usr/bin/env python
"""M8 replay: record the chain's outputs to CSV.

Raw message subscription only -- no `rostopic echo` text parsing (project rule).
Outputs, all under the run directory given by ~out_dir:

    odom_gps.csv       /odometry/gps       (navsat_transform output, base_link in odom)
    odom_filtered.csv  /odometry/filtered  (the EKF's state, downstream consumer)
    rtk_ref.csv        /rtk_odom           (the recorded base_link position, reference)
    warns.csv          /rosout_agg         navsat_transform gate messages, classified
"""
import os
import re

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log

KINDS = [
    ('hold', re.compile(r'Holding the last accepted yaw')),
    ('stale', re.compile(r'is now considered stale')),
    ('invalid', re.compile(r'is now considered invalid')),
    ('reanchor_gyro', re.compile(r're-anchored .* against the gyro reference')),
    ('reanchor_two', re.compile(r're-anchored .* on two consecutive agreeing')),
]


class Rec(object):
    def __init__(self):
        self.out = rospy.get_param('~out_dir', 'run')
        if not os.path.isdir(self.out):
            os.makedirs(self.out)
        self.f_gps = self._open('odom_gps.csv', 't,x,y,z,yaw')
        self.f_flt = self._open('odom_filtered.csv', 't,x,y,z,yaw')
        self.f_rtk = self._open('rtk_ref.csv', 't,x,y,z,yaw')
        self.f_wrn = self._open('warns.csv', 't,kind,msg')
        rospy.Subscriber('/odometry/gps', Odometry, self.cb, ('gps',), queue_size=200)
        rospy.Subscriber('/odometry/filtered', Odometry, self.cb, ('flt',), queue_size=400)
        rospy.Subscriber('/rtk_odom', Odometry, self.cb, ('rtk',), queue_size=200)
        rospy.Subscriber('/rosout_agg', Log, self.cb_log, queue_size=400)
        rospy.on_shutdown(self.close)

    def _open(self, name, header):
        f = open(os.path.join(self.out, name), 'w')
        f.write(header + '\n')
        return f

    def cb(self, msg, which):
        f = {'gps': self.f_gps, 'flt': self.f_flt, 'rtk': self.f_rtk}[which[0]]
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        try:
            yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        except Exception:
            yaw = float('nan')
        f.write('%.6f,%.6f,%.6f,%.6f,%.6f\n'
                % (msg.header.stamp.to_sec(), p.x, p.y, p.z, yaw))

    def cb_log(self, msg):
        if 'navsat' not in msg.name:
            return
        if msg.level < Log.WARN:
            return
        kind = 'other'
        for k, rx in KINDS:
            if rx.search(msg.msg):
                kind = k
                break
        self.f_wrn.write('%.6f,%s,"%s"\n'
                         % (msg.header.stamp.to_sec(), kind,
                            msg.msg.replace('"', "'").replace('\n', ' ')))

    def close(self):
        for f in (self.f_gps, self.f_flt, self.f_rtk, self.f_wrn):
            try:
                f.flush()
                f.close()
            except Exception:
                pass


if __name__ == '__main__':
    rospy.init_node('m8_recorder')
    Rec()
    rospy.spin()
