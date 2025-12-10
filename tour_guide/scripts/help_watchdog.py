#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math

from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from kobuki_msgs.msg import Sound


class HelpWatchdog(object):
    def __init__(self):
        rospy.init_node("help_watchdog")

        # How often to check the plan (seconds)
        self.plan_check_period = rospy.get_param("~plan_check_period", 1.0)

        # If there is NO global plan for this many seconds -> ask for help
        self.no_plan_threshold = rospy.get_param("~no_plan_threshold_sec", 8.0)

        # NEW: if there IS a plan but robot hasn't moved for this long -> stuck -> ask for help
        self.stuck_timeout = rospy.get_param("~stuck_timeout_sec", 10.0)

        # NEW: movement threshold to count as "we moved"
        self.min_move_dist = rospy.get_param("~min_move_dist_m", 0.05)

        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # Which beep to play when path is blocked (default 4 = error sound)
        self.help_beep_value = int(rospy.get_param("~help_beep_value", 4))

        self.tf_listener = tf.TransformListener()

        # Accumulator for "no plan" time
        self._no_plan_accum = 0.0
        self._last_check_time = rospy.Time.now()

        # NEW: movement tracking for "stuck with a plan"
        self._last_moving_pose = None
        self._last_moving_time = rospy.Time.now()

        # Publisher to Kobuki's built-in speaker
        self.sound_pub = rospy.Publisher(
            "/mobile_base/commands/sound", Sound, queue_size=1
        )

        # Track pose from localization
        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self._pose_cb, queue_size=1
        )

        rospy.loginfo("help_watchdog: waiting for /move_base/make_plan service...")
        rospy.wait_for_service("/move_base/make_plan")
        self.make_plan_srv = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        rospy.loginfo("help_watchdog: connected to make_plan")

        self.timer = rospy.Timer(
            rospy.Duration(self.plan_check_period), self._timer_cb
        )

        rospy.loginfo(
            "help_watchdog: started (no-plan-threshold=%.1fs, stuck-timeout=%.1fs, min-move=%.2fm, beep=%d)",
            self.no_plan_threshold,
            self.stuck_timeout,
            self.min_move_dist,
            self.help_beep_value,
        )

    # ------------- Pose / movement tracking -------------

    def _pose_cb(self, msg):
        """Track how long it has been since the robot last moved more than min_move_dist."""
        pos = msg.pose.pose.position
        now = rospy.Time.now()

        if self._last_moving_pose is None:
            self._last_moving_pose = pos
            self._last_moving_time = now
            return

        dx = pos.x - self._last_moving_pose.x
        dy = pos.y - self._last_moving_pose.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.min_move_dist:
            self._last_moving_pose = pos
            self._last_moving_time = now

    # ------------- Helper functions -------------

    def get_current_goal(self):
        """Get the current goal that move_base is working on."""
        try:
            msg = rospy.wait_for_message(
                "/move_base/current_goal", PoseStamped, timeout=0.2
            )
            return msg
        except rospy.ROSException:
            return None

    def get_current_pose(self, frame_id):
        """Get the robot's current pose in the specified frame using TF."""
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
        """Ask the global planner if a path exists from start to goal."""
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
        """Publish the help beep (default: error sound 4)."""
        msg = Sound()
        msg.value = self.help_beep_value
        rospy.loginfo(
            "help_watchdog: publishing help beep value=%d", msg.value
        )
        self.sound_pub.publish(msg)

    # ------------- Timer callback: main logic -------------

    def _timer_cb(self, event):
        now = rospy.Time.now()
        dt = (now - self._last_check_time).to_sec()
        self._last_check_time = now

        # If there is no active goal, reset state and do nothing.
        goal = self.get_current_goal()
        if goal is None:
            self._no_plan_accum = 0.0
            return

        start = self.get_current_pose(goal.header.frame_id)
        if start is None:
            return

        ok = self.has_plan(start, goal)

        if ok:
            # There IS a global plan.
            if self._no_plan_accum > 0.0:
                rospy.loginfo("help_watchdog: plan is available again, reset 'no-plan' timer")
            self._no_plan_accum = 0.0

            # Check for "stuck with plan" (haven't moved recently)
            stuck_dt = (now - self._last_moving_time).to_sec()
            rospy.logdebug(
                "help_watchdog: stuck_dt=%.2fs (timeout=%.2fs)",
                stuck_dt,
                self.stuck_timeout,
            )

            if stuck_dt >= self.stuck_timeout:
                rospy.logwarn(
                    "help_watchdog: stuck for %.1fs with valid plan, asking for help",
                    stuck_dt,
                )
                self.beep_help()
                # Reset timer so we don't beep continuously
                self._last_moving_time = now

            return

        # No valid global plan
        self._no_plan_accum += dt
        rospy.logdebug(
            "help_watchdog: no global plan for %.2fs", self._no_plan_accum
        )

        if self._no_plan_accum >= self.no_plan_threshold:
            rospy.logwarn(
                "help_watchdog: no route for %.1fs, asking for help",
                self._no_plan_accum,
            )
            self.beep_help()
            # Reset "no-plan" timer so we don't spam
            self._no_plan_accum = 0.0


if __name__ == "__main__":
    try:
        HelpWatchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
