#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus


def make_goal(x, y, yaw, frame_id):
    goal = MoveBaseGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # yaw -> quaternion (z, w) in 2D for planar nav
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


class GoalRunner(object):
    def __init__(self):
        rospy.init_node("goal_runner")

        # Parameters (fill these from launch)
        self.frame_id = rospy.get_param("~frame_id", "map")

        f_xy = rospy.get_param("~forward_goal_xy", [0.0, 0.0])
        f_yaw = rospy.get_param("~forward_goal_yaw", 0.0)

        r_xy = rospy.get_param("~return_goal_xy", [0.0, 0.0])
        r_yaw = rospy.get_param("~return_goal_yaw", 0.0)

        # Wider-radius acceptance for goals (meters)
        self.xy_tolerance = float(rospy.get_param("~xy_goal_tolerance", 0.35))

        self.forward_goal = (f_xy[0], f_xy[1], f_yaw)
        self.return_goal = (r_xy[0], r_xy[1], r_yaw)

        # Track current pose from AMCL
        self.current_pose = None
        rospy.Subscriber(
            "/amcl_pose",
            PoseWithCovarianceStamped,
            self._amcl_cb,
            queue_size=1,
        )

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("goal_runner: waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("goal_runner: connected to move_base")

        # Flag for the door IR node
        self.at_door_pub = rospy.Publisher(
            "at_door_region", Bool, queue_size=1, latch=True
        )

        rospy.sleep(1.0)
        self.run_sequence()

    # ---------------- Pose callback ----------------

    def _amcl_cb(self, msg):
        """Store the latest AMCL pose so we can compute distance to goal."""
        self.current_pose = msg.pose.pose.position

    # ---------------- Goal helpers ----------------

    def send_and_wait(self, goal_tuple, label):
        """Send a goal and wait until either:
           - move_base reports success, OR
           - we are within xy_tolerance of the goal coordinates.
        """
        x, y, yaw = goal_tuple
        goal = make_goal(x, y, yaw, self.frame_id)

        rospy.loginfo(
            "goal_runner: sending %s goal (%.2f, %.2f, yaw=%.2f) with xy_tolerance=%.2f m",
            label, x, y, yaw, self.xy_tolerance
        )

        self.client.send_goal(goal)

        state = self.wait_until_close(goal, label)
        rospy.loginfo(
            "goal_runner: %s goal finished with state %d", label, state
        )
        return state

    def wait_until_close(self, goal, label, rate_hz=5.0):
        """Loop until:
           - move_base says SUCCEEDED, OR
           - robot is within xy_tolerance of goal, OR
           - move_base reports a terminal failure state.
        """
        rate = rospy.Rate(rate_hz)

        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y

        while not rospy.is_shutdown():
            state = self.client.get_state()

            # If move_base succeeded normally, accept that
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("goal_runner: %s goal SUCCEEDED (move_base).", label)
                return state

            # If move_base aborted/rejected, give up
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                rospy.logwarn(
                    "goal_runner: %s goal ended with state %d before reaching tolerance",
                    label, state
                )
                return state

            # Otherwise, check our own position against a radius
            if self.current_pose is not None:
                dx = self.current_pose.x - goal_x
                dy = self.current_pose.y - goal_y
                dist = math.sqrt(dx * dx + dy * dy)

                if dist <= self.xy_tolerance:
                    rospy.loginfo(
                        "goal_runner: %s goal within tolerance radius (dist=%.3f m <= %.3f m), accepting.",
                        label, dist, self.xy_tolerance
                    )
                    # Cancel the goal so move_base stops trying to refine it
                    self.client.cancel_goal()
                    return GoalStatus.SUCCEEDED

            rate.sleep()

        # If we exit the loop because of shutdown
        return self.client.get_state()

    # ---------------- Sequence logic ----------------

    def run_sequence(self):
        # 1) Go to back of room
        self.send_and_wait(self.forward_goal, "FORWARD")

        rospy.sleep(0.5)

        # 2) Return to door
        self.send_and_wait(self.return_goal, "RETURN")

        # Mark that we're at/near door so checkerboard node starts looking
        rospy.loginfo("goal_runner: at door region, publishing flag")
        self.at_door_pub.publish(Bool(data=True))

        rospy.loginfo("goal_runner: sequence finished")
        # Keep node alive so publishers don't die immediately
        rospy.spin()


if __name__ == "__main__":
    try:
        GoalRunner()
    except rospy.ROSInterruptException:
        pass
