#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


def make_goal(x, y, yaw, frame_id):
    goal = MoveBaseGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # yaw -> quaternion (z, w) in 2D
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

        self.forward_goal = (f_xy[0], f_xy[1], f_yaw)
        self.return_goal = (r_xy[0], r_xy[1], r_yaw)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("goal_runner: waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("goal_runner: connected to move_base")

        self.at_door_pub = rospy.Publisher(
            "at_door_region", Bool, queue_size=1, latch=True
        )

        rospy.sleep(1.0)
        self.run_sequence()

    def send_and_wait(self, goal_tuple, label):
        x, y, yaw = goal_tuple
        goal = make_goal(x, y, yaw, self.frame_id)

        rospy.loginfo(
            "goal_runner: sending %s goal (%.2f, %.2f, yaw=%.2f)",
            label, x, y, yaw
        )

        self.client.send_goal(goal)
        self.client.wait_for_result()
        state = self.client.get_state()
        rospy.loginfo(
            "goal_runner: %s goal finished with state %d", label, state
        )
        return state

    def run_sequence(self):
        # 1) Go to back of room
        self.send_and_wait(self.forward_goal, "FORWARD")

        rospy.sleep(0.5)

        # 2) Return to door
        self.send_and_wait(self.return_goal, "RETURN")

        # Mark that we're at/near door
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
