#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from kobuki_msgs.msg import Sound


class HelpWatchdog(object):
    def __init__(self):
        rospy.init_node("help_watchdog")

        # Frames (for current_goal; pose is in map via amcl)
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame   = rospy.get_param("~base_frame", "base_link")

        # How long we can go without progress before we "ask for help"
        self.stuck_timeout = rospy.get_param("~stuck_timeout_sec", 8.0)

        # Minimum progress (in meters) that counts as "we made progress"
        self.min_progress = rospy.get_param("~min_progress_m", 0.10)

        # Radius within which we treat the goal as "basically reached"
        self.goal_reached_radius = rospy.get_param("~goal_reached_radius_m", 0.35)

        # After asking for help, wait this long before checking again
        self.pause_after_help = rospy.get_param("~pause_after_help_sec", 10.0)

        # Sound to play for help (4 = Kobuki error sound)
        self.help_beep_value = int(rospy.get_param("~help_beep_value", 4))

        # Internal state
        self.current_pose = None      # from /amcl_pose
        self.current_goal = None      # from /move_base/current_goal

        self.last_dist_to_goal = None
        self.last_progress_time = rospy.Time.now()
        self.pause_until = rospy.Time(0)

        # Publisher for sound
        self.sound_pub = rospy.Publisher(
            "/mobile_base/commands/sound", Sound, queue_size=1
        )

        # Subscribers
        rospy.Subscriber(
            "/amcl_pose",
            PoseWithCovarianceStamped,
            self._amcl_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/move_base/current_goal",
            PoseStamped,
            self._goal_cb,
            queue_size=1,
        )

        # Timer to periodically check progress
        self.timer = rospy.Timer(
            rospy.Duration(1.0),  # check once per second
            self._timer_cb
        )

        rospy.loginfo(
            "help_watchdog: started (stuck_timeout=%.1fs, min_progress=%.2fm, "
            "goal_reached_radius=%.2fm, pause_after_help=%.1fs, beep=%d)",
            self.stuck_timeout,
            self.min_progress,
            self.goal_reached_radius,
            self.pause_after_help,
            self.help_beep_value,
        )

    # ---------- Callbacks ----------

    def _amcl_cb(self, msg):
        self.current_pose = msg.pose.pose.position

    def _goal_cb(self, msg):
        self.current_goal = msg.pose.position
        # Reset progress tracking when a new goal arrives
        self.last_dist_to_goal = None
        self.last_progress_time = rospy.Time.now()

    # ---------- Helpers ----------

    def _compute_dist_to_goal(self):
        if self.current_pose is None or self.current_goal is None:
            return None
        dx = self.current_pose.x - self.current_goal.x
        dy = self.current_pose.y - self.current_goal.y
        return math.sqrt(dx * dx + dy * dy)

    def _beep_help(self):
        msg = Sound()
        msg.value = self.help_beep_value
        rospy.loginfo(
            "help_watchdog: publishing help beep value=%d", msg.value
        )
        self.sound_pub.publish(msg)

    # ---------- Timer logic ----------

    def _timer_cb(self, event):
        now = rospy.Time.now()

        # Respect pause window after we've already asked for help
        if now < self.pause_until:
            return

        # Need both goal and pose to reason about progress
        if self.current_goal is None or self.current_pose is None:
            self.last_dist_to_goal = None
            self.last_progress_time = now
            return

        dist = self._compute_dist_to_goal()
        if dist is None:
            return

        # If we're effectively at the goal, reset tracking and do nothing
        if dist <= self.goal_reached_radius:
            if self.last_dist_to_goal is not None:
                rospy.loginfo(
                    "help_watchdog: within goal radius (dist=%.3f m <= %.3f m), no help needed.",
                    dist, self.goal_reached_radius
                )
            self.last_dist_to_goal = dist
            self.last_progress_time = now
            return

        # First time seeing a distance
        if self.last_dist_to_goal is None:
            self.last_dist_to_goal = dist
            self.last_progress_time = now
            return

        # Check if we've made enough progress toward the goal
        progress = self.last_dist_to_goal - dist
        if progress >= self.min_progress:
            # We moved closer by at least min_progress -> reset timer
            rospy.logdebug(
                "help_watchdog: progress=%.3f m (old=%.3f, new=%.3f), resetting timer.",
                progress, self.last_dist_to_goal, dist
            )
            self.last_dist_to_goal = dist
            self.last_progress_time = now
            return

        # Not enough progress: see how long we've been "stuck"
        stuck_time = (now - self.last_progress_time).to_sec()
        rospy.logdebug(
            "help_watchdog: low progress (%.3f m) for %.2f s",
            progress, stuck_time
        )

        if stuck_time >= self.stuck_timeout:
            rospy.logwarn(
                "help_watchdog: no significant progress toward goal for %.1f s (dist=%.3f m) -> asking for help",
                stuck_time, dist
            )
            self._beep_help()
            # Pause checks for a bit so we don't spam beeps
            self.pause_until = now + rospy.Duration(self.pause_after_help)
            # Reset reference for next progress check
            self.last_dist_to_goal = dist
            self.last_progress_time = now


if __name__ == "__main__":
    try:
        HelpWatchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
