#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Sound


class CheckerboardIR(object):
    def __init__(self):
        rospy.init_node("checkerboard_ir")

        # IR topic (mono16)
        self.image_topic = rospy.get_param("~image_topic", "/camera/ir/image")

        # INNER corners of checkerboard
        self.pattern_rows = int(rospy.get_param("~pattern_rows", 7))
        self.pattern_cols = int(rospy.get_param("~pattern_cols", 5))

        # Beep when detected
        self.beep_value = int(rospy.get_param("~beep_value", 4))
        self.cooldown = rospy.Duration(3.0)
        self.last_beep = rospy.Time(0)

        self.bridge = CvBridge()
        self.sound_pub = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=1)

        # Subscribe to IR image
        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        rospy.loginfo(
            "checkerboard_ir: ALWAYS checking %s for %dx%d inner-corner checkerboard",
            self.image_topic, self.pattern_rows, self.pattern_cols
        )

    def _beep(self):
        now = rospy.Time.now()
        if now - self.last_beep < self.cooldown:
            return
        self.last_beep = now

        msg = Sound()
        msg.value = self.beep_value
        rospy.loginfo("checkerboard_ir: CHECKERBOARD DETECTED -> playing sound %d", msg.value)
        self.sound_pub.publish(msg)

    def _image_cb(self, msg):
        # Convert mono16 -> usable numpy array
        try:
            cv16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)
            return

        # Convert to mono8 for OpenCV
        gray = cv2.convertScaleAbs(cv16, alpha=0.02)

        # Try checkerboard detection (OpenCV expects inner-corners)
        pattern_size = (self.pattern_cols, self.pattern_rows)
        found, _ = cv2.findChessboardCorners(gray, pattern_size)

        if found:
            self._beep()
