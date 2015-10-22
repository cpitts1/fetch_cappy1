#! /usr/bin/env python

import actionlib
import rospy
import numpy as np

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from sensor_msgs.msg import JointState


class TrajectoryExecutor():
  
    def __init__(self):

        self.arm_action = ArmInterfaceClient()
        self.gripper_action = GripperInterfaceClient()
        self.arm_and_torso_action = ArmWithTorsoClient()
        self.execution_monitor = ExecutionMonitor()
          
 
    def execute_trajectories(self, trajectories, error_checking = True ):
        supported_args = ("error_checking")
        self.current_trajectory_index = 0
        for trajectory in trajectories:
            self.current_trajectory_index += 1
            rospy.sleep(0.5)
            num_commanded_joints = len(trajectory.joint_names)
            if num_commanded_joints is 7:
               self.arm_action.execute_trajectory(trajectory)
            elif num_commanded_joints is 2:
               self.gripper_action.execute_trajectory(trajectory)
            elif num_commanded_joints is 8:
               self.arm_and_torso_action.execute_trajectory(trajectory)
            else:
               print("No controller exists for planning group %s"
                        %trajectory.joint_names)
            if error_checking:
               execution_error = self.execution_monitor.verify_goal_achievement(trajectory.joint_trajectory)
               if execution_error:
                  return True

    def get_current_trajectory_index(self):
        return self.current_trajectory_index
         

# Send a trajectory to controller
class ArmInterfaceClient(object):

    def __init__(self):
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]

        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
        self.client.wait_for_server()
        self.finish_time = rospy.Time(10000)
        
        self.state_listener = rospy.Subscriber('joint_states', JointState, self.__state_listener_callback)
        self.most_recent_state = None
 
        self.measured_state = None
        self.believed_state = None

    def __state_listener_callback(self, state_data):
          self.measured_state = state_data.position[5:13] 
          

    def execute_trajectory(self, trajectory, duration=5.0):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        self.client.send_goal(follow_goal)
        #rospy.sleep(1.5)
        #self.client.cancel_goal()
        self.client.wait_for_result()
        self.finish_time = rospy.Time.now() 
        self.believed_state = trajectory.points[-1].positions

    def verify_goal_achievement(self):
        joint_number = 0
        for (i,j) in zip(self.measured_state, self.believed_state):
            diff = subtract_angles(i, j)
            if abs(diff) > .02:
               rospy.loginfo('Diff: ' + str(diff))
               rospy.loginfo('=============== Joint Goal NOT Achieved')
               rospy.loginfo('=============== Current state is: ' + str(self.measured_state))
               return False
            else:
               rospy.loginfo('Joint %d Goal Achieved' %(joint_number))
               joint_number += 1
        rospy.loginfo('=============== Goal Pose Achieved')
        return True

class GripperInterfaceClient():
 
    def __init__(self):
        self.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]

        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action",
                                                   GripperCommandAction)
        rospy.loginfo("Waiting for gripper_controller...")
        
    def execute_trajectory(self, trajectory):
        goal_position = trajectory.points[-1].positions
        print '==========', goal_position
        command_goal = GripperCommandGoal()
        command_goal.command.position = goal_position[0]+goal_position[1]
        command_goal.command.max_effort = 50.0 # Placeholder. TODO Figure out a better way to compute this
        self.client.send_goal(command_goal)
        self.client.wait_for_result()

    def move_to(self, goal_position, max_effort):
        command_goal = GripperCommandGoal()
        command_goal.command.position = goal_position
        command_goal.command.max_effort = max_effort
        self.client.send_goal(command_goal)
        self.client.wait_for_result()

            
class ArmWithTorsoClient():
  
    def __init__(self):
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]

        self.client = actionlib.SimpleActionClient("arm_with_torso_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
        self.client.wait_for_server()
        self.finish_time = rospy.Time(10000)
        
        self.state_listener = rospy.Subscriber('joint_states', JointState, self.__state_listener_callback)
        self.most_recent_state = None
 
        self.measured_state = None
        self.believed_state = None

    def __state_listener_callback(self, state_data):
          self.measured_state = state_data.position[6:13] 
          

    def execute_trajectory(self, trajectory, duration=5.0):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        self.client.send_goal(follow_goal)
        #rospy.sleep(1.5)
        #self.client.cancel_goal()
        self.client.wait_for_result()
        self.finish_time = rospy.Time.now() 
        self.believed_state = trajectory.points[-1].positions

    def verify_goal_achievement(self):
        joint_number = 0
        for (i,j) in zip(self.measured_state, self.believed_state):
            diff = subtract_angles(i, j)
            if abs(diff) > .02:
               rospy.loginfo('Diff: ' + str(diff))
               rospy.loginfo('=============== Joint Goal NOT Achieved')
               rospy.loginfo('=============== Current state is: ' + str(self.measured_state))
               return False
            else:
               rospy.loginfo('Joint %d Goal Achieved' %(joint_number))
               joint_number += 1
        rospy.loginfo('=============== Goal Pose Achieved')
        return True


class ExecutionMonitor():

    def __init__(self):

        self.state_listener = rospy.Subscriber('joint_states', JointState, self.__state_listener_callback)
        
    def __state_listener_callback(self, state_data):
        self.measured_state = state_data.position[6:13] 
    
    def verify_goal_achievement(self, trajectory):
        joint_number = 0
        self.believed_state = trajectory.points[-1].positions
        print self.believed_state
        for (i,j) in zip(self.measured_state, self.believed_state):
            diff = subtract_angles(i, j)
            if abs(diff) > .01:
               rospy.loginfo('Diff: ' + str(diff))
               rospy.loginfo('=============== Joint Goal NOT Achieved')
               rospy.loginfo('=============== Current state is: ' + str(self.measured_state))
               return True
            else:
               rospy.loginfo('Joint %d Goal Achieved' %(joint_number))
               joint_number += 1
        rospy.loginfo('=============== Goal Pose Achieved')
        return False


def subtract_angles(angle_i, angle_j): 

    # Subtraction of two angles using vector method
    # Robust to modality issue occuring for angles having difference
    # greater than +/- pi. 
    pos_vector_i = np.array([np.cos(angle_i), np.sin(angle_i)])
    pos_vector_j = np.array([np.cos(angle_j), np.sin(angle_j)])
    cos_of_angle_i_j = np.dot(pos_vector_i, pos_vector_j)
    angle_i_j = np.arccos(cos_of_angle_i_j)
    
    return angle_i_j

 
        
