#! /usr/bin/env python

#Importations

import copy
import rospy
import sys

from geometry_msgs.msg import PoseStamped
from trajectory_execution import TrajectoryExecutor
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from tf.transformations import quaternion_from_euler



## Goal Containers ##

class PoseGoal():
      # Container for End Effector Pose Goals 
      def __init__(self, pose):
          self.goal_type = "ee_pose"
          num_args = len(pose)
          if num_args == 6:
             self.pose = pose
          else:
             rospy.loginfo("PoseGoal requires 6 elements (x,y,z,alpha,beta,gamma); %d given" %num_args)


class JointSpaceGoal():
      # Container for JointSpace goals 
      def __init__(self, state):
         self.goal_type = "jointspace"
         num_args = len(state)
         if num_args == 7:
            self.state = state
         else:
            rospy.loginfo("JointSpaceGoal requires 7 joint states; %d given" %num_args)

class GripperPostureGoal():
      # Container for gripper internal posture goals (i.e. finger position)
      def __init__(self, posture):
          self.goal_type = "gripper_posture"
          try: 
             self.posture = [posture/2.0, posture/2.0]
          except TypeError:
             rospy.loginfo("GripperPostureGoal requires exactly 1 argument: %d given" %num_args)

def ask_for_replan():
    choice = 'not_set'
    options = {'r':True, 'q':False}
    while choice.lower() not in options.keys():
        choice = raw_input("Replan (r) using current waypoints, or quit (q)?: ")
    return options[choice] 


## Plannning ##

class TrajectoryGenerator(): 
     # Wrapper class that facilitates trajectory planning.
     # Includes methods for planning to arbitrary joint-space goals
     # and end-effector pose goals.  
       
      def __init__(self): 
          self.group = MoveGroupInterface("arm", "base_link", plan_only=True)
          self.gripper = MoveGroupInterface("gripper", "base_link", plan_only=True)
          self.virtual_arm_state = RobotState()
          self.virtual_gripper_state = RobotState()
          # Sleep to allow RVIZ time to start up.
          print "============ Waiting for RVIZ..."
          rospy.sleep(5)
          print "============ Starting planning"
 
      def generate_trajectories(self, motion_goals, **kwargs):
          approved_trajectories = list()
          supported_args = ("get_approval")
          for arg in kwargs.keys():
              if arg not in supported_args:
                 rospy.loginfo("generate_trajectories: unsupported argument: %s",
                             arg)

          if type(motion_goals) is not type(list()):
             motion_goals = [motion_goals]
          for motion_goal in motion_goals:
             if motion_goal.goal_type is 'ee_pose':
                print "==== Virtual Start State: ", self.virtual_arm_state
                trajectory = self.plan_to_ee_pose_goal(motion_goal.pose)
             elif motion_goal.goal_type is "jointspace":
                trajectory = self.plan_to_jointspace_goal(motion_goal.state)
             elif motion_goal.goal_type is "gripper_posture":
                trajectory = self.plan_to_gripper_goal(motion_goal.posture)
             else:
                rospy.loginfo("Goal type improperly specified. Please specify as either 'ee_pose' or 'jointspace'")
                sys.exit()
             try:
                kwargs["get_approval"]
                approved = self.get_user_approval()
                if approved:
                   approved_trajectories.append(trajectory)
                   self.update_virtual_state(trajectory)
                else:
                   # Recur
                   approved_trajectories += self.generate_trajectories(motion_goal, get_approval=True)
             except KeyError:
                pass
          returnable_approved_trajectories = copy.deepcopy(approved_trajectories)
          return returnable_approved_trajectories


      def get_user_approval(self):
          choice = 'not_set'
          options = {'y':True, 'r':False}
          while choice.lower() not in options.keys():
              choice = raw_input("Add plan to execution queue? Answer 'y' for yes "\
					" or 'r' for replan: ")
          return options[choice]

      def plan_to_ee_pose_goal(self, goal):
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
              result = self.group.moveToPose(pose_target, "wrist_roll_link",
                                            tolerance = 0.02, plan_only=True,
                                            start_state = self.virtual_arm_state,
                                            planner_id = "PRMkConfigDefault")
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 print "================ Pose Achieved"
                 self.trajectory = result.planned_trajectory.joint_trajectory
                 return result.planned_trajectory

      def plan_to_jointspace_goal(self, state):
          
          joints = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]
                     
          while not rospy.is_shutdown():
              result = self.group.moveToJointPosition(joints, state, plan_only=True,
                                                     start_state=self.virtual_arm_state,
                                                     planner_id = "PRMkConfigDefault")
                                                     
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 print "============ Pose Achieved"
                 self.trajectory = result.planned_trajectory.joint_trajectory
                 return result.planned_trajectory

      def plan_to_gripper_goal(self, posture):
          joints = ["l_gripper_finger_joint",
                     "r_gripper_finger_joint"]
          while not rospy.is_shutdown():
              result = self.gripper.moveToJointPosition(joints, posture, plan_only=True,
                                                        start_state=self.virtual_gripper_state)
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 rospy.loginfo("Gripper Posture Plan Successful")
                 self.trajectory = result.planned_trajectory.joint_trajectory
                 return result.planned_trajectory

      def update_virtual_state(self, trajectory):
          # Sets joint names and sets position to final position of previous trajectory 
          if len(trajectory.joint_trajectory.points[-1].positions) == 7:
             self.virtual_arm_state.joint_state.name = [
                                                        "shoulder_pan_joint",
                                "shoulder_lift_joint", "upperarm_roll_joint",
                                  "elbow_flex_joint",   "forearm_roll_joint",
                                    "wrist_flex_joint",   "wrist_roll_joint"]
             self.virtual_arm_state.joint_state.position = trajectory.joint_trajectory.points[-1].positions
             rospy.loginfo("Virtual Arm State Updated")
             print self.virtual_arm_state.joint_state.position

          elif len(trajectory.joint_trajectory.points[-1].positions) == 2:
               self.virtual_gripper_state.joint_state.name = ["l_gripper_finger_joint", "r_gripper_finger_joint"]
               self.virtual_gripper_state.joint_state.position = trajectory.joint_trajectory.points[-1].positions
               rospy.loginfo("Virtual Gripper State Updated")
               print self.virtual_gripper_state.joint_state.position
  
      def clear_virtual_arm_state(self):
          self.virtual_arm_state.joint_state.name = []
          self.virtual_arm_state.joint_state.position = []

      def clear_virtual_gripper_state(self):
          self.virtual_gripper_state.joint_state.names = []
          self.virtual_gripper_state.joint_state.position = []

      def get_virtual_arm_state(self):
          virtual_arm_state = copy.deepcopy(self.virtual_arm_state)
          return virtual_arm_state

      def get_virtual_gripper_state(self):
          virtual_gripper_state = copy.deepcopy(self.virtual_gripper_state)
          return virtual_gripper_state


