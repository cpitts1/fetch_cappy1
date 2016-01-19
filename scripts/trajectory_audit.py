#! /usr/bin/env python
# Early version of script used to audit trajectories 
# Author Daniel Palmer

import actionlib
import numpy as np
import rospy
import sys

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from geometry_msgs.msg import PoseStamped
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal

class TrajectorySimulator(): 
       
      def __init__(self): 
          self.group = MoveGroupInterface("arm", "base_link")

          # Sleep to allow RVIZ time to start up.
          print "============ Waiting for RVIZ..."
          rospy.sleep(15)
          print "============ Starting planning"

      def plan_to_pose_goal(self, goal):
          # Extract goal position
          pose_x = goal[0]
          pose_y = goal[1]
          pose_z = goal[2]
               
          # Extract goal orientation--provided in Euler angles
          roll = goal[3]
          pitch = goal[4]
          yaw = goal[5]
               
          # Convert Euler angles to quaternion, create PoseStamped message
          quaternion = quaternion_from_euler(roll, pitch, yaw)
          pose_target = PoseStamped()
          pose_target.pose.position.x = pose_x
	  pose_target.pose.position.y = pose_y
	  pose_target.pose.position.z = pose_z
          pose_target.pose.orientation.x = quaternion[0]
          pose_target.pose.orientation.y = quaternion[1]
          pose_target.pose.orientation.z = quaternion[2]
          pose_target.pose.orientation.w = quaternion[3]
          pose_target.header.frame_id = "base_link"
          pose_target.header.stamp = rospy.Time.now()
          print "============= PoseStamped message filled out"

          # Execute motion
          while not rospy.is_shutdown():
              result = self.group.moveToPose(pose_target, "gripper_link",
                                            tolerance = 0.3, plan_only=True)
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 print "================ Pose Achieved"
                 return result.planned_trajectory 

      def plan_to_jointspace_goal(self, pose):
          joints = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]
          while not rospy.is_shutdown():
              result = self.group.moveToJointPosition(joints, pose)
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 print "============ Pose Achieved"
                 return "Success"


class ManualTrajectoryClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
        self.client.wait_for_server()
        rospy.loginfo("...running")
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]

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
        trajectory.points[0].velocities = [] # [0.0 for _ in positions]
        trajectory.points[0].accelerations = [] #[0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self):
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]

        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
        self.client.wait_for_server()

    def move_to(self, result_trajectory, duration=5.0):
        trajectory = result_trajectory
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()



# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

def get_user_approval():
    ans = None
    while ans not in ['y', 'r', 'q']:
       ans = raw_input("Execute plan now? Answer 'y' for yes, 'q' for quit,"\
					" or 'r' for replan: ")
    return ans

if __name__ == "__main__":
  
   # Create a Node
   rospy.init_node('trajectory_audit')
  
 

   # Instantiate TrajectorySimulator object
   
   t_sim = TrajectorySimulator()
   arm_action = FollowTrajectoryClient()
   manual_arm_action = ManualTrajectoryClient()
   head_action = PointHeadClient()
   print '========= Pointing Head...'
   #head_action.look_at(0.5, -0.2, 0.0, 'map')

   pose_1 = [0.5,-0.5, 1.0, 0.0, -1.57, 0.0]
   poses = []
   poses.append(pose_1)
   """ 
   state = [.707, 0, 0, 0, 0, 0, 0]
   arm_action.move_to(state)
   state_2 = [.707, .707,0,0,0,0,0]
   arm_action.move_to(state_2)
   """
   arm_goal = [-1.6, 0.48, 0, -1.73, 0, -.99, -1.54]
   t_sim.plan_to_jointspace_goal(arm_goal)
   """
   for pose in poses:
     while not rospy.is_shutdown():
       result = t_sim.plan_to_pose_goal(pose)
       #print result.joint_trajectory
       user_approves = get_user_approval()
       if user_approves == 'y':
          # Execute
          arm_action.move_to(result.joint_trajectory) 
          cur_state = result.joint_trajectory.points[-1].positions
          print cur_state
          break
       elif user_approves == 'q':
          break
   next_state = list(cur_state)
   """
   num_waves = 10
   for n in range(num_waves):
     arm_goal[5] = arm_goal[5] + (-1)**n * 1.0
     t_sim.plan_to_jointspace_goal(arm_goal)



