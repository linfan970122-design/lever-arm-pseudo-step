#!/usr/bin/env python3
"""M8b: the heading-continuity gate applied on the MEASUREMENT side.

It sits between the republished raw RTK heading and the EKF's imu0 input, i.e. it
judges the heading BEFORE the filter smears it, which is the one thing the node-side
gate of M8 cannot do (there the gate reads the filter's own yaw, so a low Kalman gain
lets an outlier in below the fixed per-frame budget).

Rule: NOT reimplemented here.  `fig/gate.py` is copied verbatim to the machine and
imported; `gate.gate_series_v3()` is the only decision function, called with exactly the
parameters the GATED arm uses.  Since that function is a whole-series routine, it is run
online through a state-equivalence identity instead of being rewritten:

    after any ACCEPT, gate_series_v3's internal state is
        (last_accepted=y_i, last_t=t_i, anchor=i, hold_count=0, valid=True, pending=None)
    which is *identical* to its state after the unconditional adoption of the first
    sample.  So running it over the buffer [k .. i], where k is the last ACCEPT, gives
    frame i exactly the decision it would get from a run over the whole series.

Hence: keep a buffer starting at the last ACCEPT, append the new frame, call the
function once, take the last element, and truncate back to one frame on every ACCEPT.
Only differences of the cumulative gyro are used by the rule, so truncation is safe.

Publication (see data/m8b_measurement_gate/design.md section 3):
    ACCEPT -> republish the frame's heading, covariance untouched
    HOLD   -> republish the LAST ACCEPTED heading (same as the node-side gate does),
              covariance untouched
    NOPUB  -> republish the message with orientation_covariance[0] = -1, which
              ros_filter.cpp:516 (commit 040dd17) treats as "no orientation in this
              message", so the EKF skips the yaw measurement for that frame.

Every frame's decision is logged, so the offline replay of fig/gate.py on the very same
input stream must agree frame for frame.
"""
import os
import sys

import numpy as np
import rospy
import tf.transformations as tft
from sensor_msgs.msg import Imu

GATE_DIR = os.environ.get('M8B_GATE_DIR',
                          os.environ.get('P2_WORK',
                                         os.path.join(os.environ.get('TMPDIR', '/tmp'), 'p2_m8')))
sys.path.insert(0, GATE_DIR)
import gate  # noqa: E402  verbatim copy of fig/gate.py


class MeasGate(object):
    def __init__(self):
        self.par = dict(
            max_yaw_rate=float(rospy.get_param('~max_yaw_rate', 1.234)),
            yaw_gate_margin=float(rospy.get_param('~yaw_gate_margin', 0.05)),
            nominal_dt=float(rospy.get_param('~nominal_dt', 0.2)),
            max_hold_frames=int(rospy.get_param('~max_hold_frames', 5)),
            max_gap_frames=int(rospy.get_param('~max_gap_frames', 3)),
            gyro_drift_rate=float(rospy.get_param('~gyro_drift_rate', 0.002)),
            use_gyro=bool(rospy.get_param('~use_gyro', True)),
            max_gyro_ref_time=float(rospy.get_param('~max_gyro_ref_time', 10.0)))
        self.log_path = rospy.get_param('~log', '')
        self.rows = []
        self.buf_t = []
        self.buf_y = []
        self.buf_g = []
        self.gz = 0.0
        self.gz_cum = 0.0        # same rectangular integration as republish.py / gyroCallback
        self.gz_t = None
        self.max_buf = 0
        self.n_acc = self.n_hold = self.n_nopub = 0

        self.pub = rospy.Publisher('heading_out', Imu, queue_size=50)
        rospy.Subscriber('gyro_in', Imu, self.cb_gyro, queue_size=400)
        rospy.Subscriber('heading_in', Imu, self.cb_hdg, queue_size=50)
        rospy.on_shutdown(self.dump)
        rospy.loginfo('m8b measurement gate up: g=%.4f rad (%.1f deg/frame)',
                      self.par['max_yaw_rate'] * self.par['nominal_dt']
                      + self.par['yaw_gate_margin'],
                      np.degrees(self.par['max_yaw_rate'] * self.par['nominal_dt']
                                 + self.par['yaw_gate_margin']))

    # ------------------------------------------------------------------ gyro
    def cb_gyro(self, msg):
        ts = msg.header.stamp.to_sec()
        if self.gz_t is not None and ts > self.gz_t:
            self.gz_cum += msg.angular_velocity.z * (ts - self.gz_t)
        self.gz_t = ts
        self.gz = msg.angular_velocity.z

    # --------------------------------------------------------------- heading
    def cb_hdg(self, msg):
        t = msg.header.stamp.to_sec()
        q = msg.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        gcum = self.gz_cum if self.gz_t is not None else float('nan')

        self.buf_t.append(t)
        self.buf_y.append(yaw)
        self.buf_g.append(gcum)
        n = len(self.buf_t)
        self.max_buf = max(self.max_buf, n)
        if n == 500:
            rospy.logwarn('m8b gate buffer at %d frames (gate has not re-anchored)', n)

        res = gate.gate_series_v3(np.asarray(self.buf_y), np.asarray(self.buf_t),
                                  cum_gyro_rad=np.asarray(self.buf_g), **self.par)
        state = int(res['state'][-1])
        published = bool(res['published'][-1])
        used = float(res['used_yaw'][-1])
        hold = int(res['hold_count'][-1])
        valid = bool(res['valid'][-1])
        reanch = bool(res['reanchor'][-1])
        fb = bool(res['fallback_b'][-1])

        out = Imu()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        cov = list(msg.orientation_covariance)
        if published:
            qq = tft.quaternion_from_euler(0.0, 0.0, used)
            out.orientation.x, out.orientation.y = qq[0], qq[1]
            out.orientation.z, out.orientation.w = qq[2], qq[3]
        else:
            out.orientation = msg.orientation
            cov[0] = -1.0          # ros_filter.cpp:516 -> orientation ignored
        out.orientation_covariance = cov
        self.pub.publish(out)

        if state == gate.ACCEPT:
            self.n_acc += 1
            # state after an ACCEPT == state after the first-sample adoption
            self.buf_t = [t]
            self.buf_y = [yaw]
            self.buf_g = [gcum]
        elif state == gate.HOLD:
            self.n_hold += 1
        else:
            self.n_nopub += 1

        self.rows.append('%.6f,%.9f,%.9f,%d,%d,%.9f,%d,%d,%d,%d,%d'
                         % (t, yaw, gcum, state, int(published), used, hold,
                            int(valid), int(reanch), int(fb), n))

    def dump(self):
        rospy.loginfo('m8b gate: accept=%d hold=%d nopub=%d max_buf=%d',
                      self.n_acc, self.n_hold, self.n_nopub, self.max_buf)
        if not self.log_path:
            return
        with open(self.log_path, 'w') as f:
            f.write('t,yaw_in,gz_cum,state,published,used_yaw,hold_count,valid,'
                    'reanchor,fallback_b,buf_len\n')
            f.write('\n'.join(self.rows) + '\n')


if __name__ == '__main__':
    rospy.init_node('m8b_gate')
    MeasGate()
    rospy.spin()
