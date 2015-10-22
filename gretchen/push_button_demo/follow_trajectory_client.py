#! /usr/bin/env python

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FollowTrajectoryClient(object):
    # Executes 'handmade' trajectories
    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s_controller..." % name)
        self.client.wait_for_server()
        rospy.loginfo("...running")
        self.joint_names = joint_names

    def follow_trajectory(self, trajectory):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()


    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = []  #[0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()


