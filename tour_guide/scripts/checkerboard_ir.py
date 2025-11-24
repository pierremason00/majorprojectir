#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import tf

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from kobuki_msgs.msg import Sound


class CheckerboardIR(object):
    def __init__(self):
        rospy.init_node("checkerboard_ir")

        self.image_topic = rospy.get_param("~image_topic", "/camera/ir/image_raw")
        self.pattern_rows = int(rospy.get_param("~pattern_rows", 7))
        self.pattern_cols = int(rospy.get_param("~pattern_cols", 5))

        center = rospy.get_param("~door_center_xy", [0.0, 0.0])
        self.door_center_x = float(center[0])
        self.door_center_y = float(center[1])
        self.door_radius = float(rospy.get_param("~door_radius_m", 1.5))

        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # Beep value for "please open the door" (default 3 = button press)
        self.door_beep_value = int(rospy.get_param("~door_beep_value", 3))
        self.announce_cooldown = rospy.Duration(
            rospy.get_param("~announce_cooldown_sec", 5.0)
        )

        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()

        self.sound_pub = rospy.Publisher(
            "/mobile_base/commands/sound", Sound, queue_size=1
        )

        self.at_door_flag = False
        self.last_announce_time = rospy.Time(0)

        rospy.Subscriber("at_door_region", Bool, self._door_flag_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        rospy.loginfo(
            "checkerboard_ir: watching %s for %dx%d checkerboard",
            self.image_topic, self.pattern_cols, self.pattern_rows
        )

    def _door_flag_cb(self, msg):
        self.at_door_flag = bool(msg.data)

    def _near_door_tf(self):
        try:
            self.tf_listener.waitForTransform(
                self.global_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.3)
            )
            (t, _) = self.tf_listener.lookupTransform(
                self.global_frame, self.base_frame, rospy.Time(0)
            )
            dx = t[0] - self.door_center_x
            dy = t[1] - self.door_center_y
            dist = (dx * dx + dy * dy) ** 0.5
            return dist <= self.door_radius
        except Exception:
            return self.at_door_flag

    def _should_check(self):
        return self._near_door_tf() or self.at_door_flag

    def _beep_door(self):
        now = rospy.Time.now()
        if now - self.last_announce_time < self.announce_cooldown:
            return

        self.last_announce_time = now

        msg = Sound()
        msg.value = self.door_beep_value
        rospy.loginfo("checkerboard_ir: publishing door beep value=%d", msg.value)
        self.sound_pub.publish(msg)

    def _image_cb(self, msg):
        if not self._should_check():
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:
            rospy.logwarn("checkerboard_ir: cv_bridge error: %s", str(e))
            return

        pattern_size = (self.pattern_cols, self.pattern_rows)
        found, corners = cv2.findChessboardCorners(
            cv_img, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if found:
            rospy.loginfo("checkerboard_ir: checkerboard detected at door")
            self._beep_door()


if __name__ == "__main__":
    try:
        CheckerboardIR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
