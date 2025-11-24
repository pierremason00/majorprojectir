#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf

from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped
from kobuki_msgs.msg import Sound


class HelpWatchdog(object):
    def __init__(self):
        rospy.init_node("help_watchdog")

        self.plan_check_period = rospy.get_param("~plan_check_period", 1.0)
        self.no_plan_threshold = rospy.get_param("~no_plan_threshold_sec", 8.0)

        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # Which beep to play when path is blocked (default 4 = error sound)
        self.help_beep_value = int(rospy.get_param("~help_beep_value", 4))

        self.tf_listener = tf.TransformListener()

        self._no_plan_accum = 0.0
        self._last_check_time = rospy.Time.now()

        # Publisher to Kobuki's built-in speaker
        self.sound_pub = rospy.Publisher(
            "/mobile_base/commands/sound", Sound, queue_size=1
        )

        rospy.loginfo("help_watchdog: waiting for /move_base/make_plan service...")
        rospy.wait_for_service("/move_base/make_plan")
        self.make_plan_srv = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        rospy.loginfo("help_watchdog: connected to make_plan")

        self.timer = rospy.Timer(
            rospy.Duration(self.plan_check_period), self._timer_cb
        )

        rospy.loginfo(
            "help_watchdog: started (threshold=%.1fs, beep=%d)",
            self.no_plan_threshold,
            self.help_beep_value,
        )

    def get_current_goal(self):
        try:
            msg = rospy.wait_for_message(
                "/move_base/current_goal", PoseStamped, timeout=0.2
            )
            return msg
        except rospy.ROSException:
            return None

    def get_current_pose(self, frame_id):
        try:
            self.tf_listener.waitForTransform(
                frame_id, self.base_frame, rospy.Time(0), rospy.Duration(0.5)
            )
            (t, q) = self.tf_listener.lookupTransform(
                frame_id, self.base_frame, rospy.Time(0)
            )
        except Exception as e:
            rospy.logwarn("help_watchdog: TF lookup failed: %s", str(e))
            return None

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = t[0]
        pose.pose.position.y = t[1]
        pose.pose.position.z = t[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def has_plan(self, start, goal):
        req = GetPlanRequest()
        req.start = start
        req.goal = goal
        req.tolerance = 0.20
        try:
            resp = self.make_plan_srv(req)
            return len(resp.plan.poses) > 0
        except Exception as e:
            rospy.logwarn("help_watchdog: make_plan failed: %s", str(e))
            # Avoid spurious help if service hiccups
            return True

    def beep_help(self):
        msg = Sound()
        msg.value = self.help_beep_value
        rospy.loginfo("help_watchdog: publishing help beep value=%d", msg.value)
        self.sound_pub.publish(msg)

    def _timer_cb(self, event):
        now = rospy.Time.now()
        dt = (now - self._last_check_time).to_sec()
        self._last_check_time = now

        goal = self.get_current_goal()
        if goal is None:
            self._no_plan_accum = 0.0
            return

        start = self.get_current_pose(goal.header.frame_id)
        if start is None:
            return

        ok = self.has_plan(start, goal)

        if ok:
            if self._no_plan_accum > 0.0:
                rospy.loginfo("help_watchdog: plan is available again, reset timer")
            self._no_plan_accum = 0.0
            return

        # No valid plan
        self._no_plan_accum += dt
        rospy.logdebug(
            "help_watchdog: no plan for %.2fs", self._no_plan_accum
        )

        if self._no_plan_accum >= self.no_plan_threshold:
            rospy.logwarn(
                "help_watchdog: no route for %.1fs, asking for help",
                self._no_plan_accum,
            )
            self.beep_help()
            # Reset so we don't spam every timer tick
            self._no_plan_accum = 0.0


if __name__ == "__main__":
    try:
        HelpWatchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
