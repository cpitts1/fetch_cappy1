#! /usr/bin/env python

import actionlib
import rospy

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)


class MoveTorsoClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso_controller...")
        self.client.wait_for_server()
        rospy.loginfo("...connected")
        self.joint_names = ["torso_lift_joint"]

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid position goal: got %d positions,
                   can only handle one." %len(positions))
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

