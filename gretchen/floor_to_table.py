#! /usr/bin/env python

#Importations

import copy
import rospy
import sys

from geometry_msgs.msg import PoseStamped
from gripper_interface_client import GripperInterfaceClient
from trajectory_execution import TrajectoryExecutor
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from point_head_client import PointHeadClient
from tf.transformations import quaternion_from_euler
from trajectory_audit import FollowTrajectoryClient
from move_base_client import MoveBaseClient
from plan_to_goal import TrajectoryGenerator, JointSpaceGoal
from tf.transformations import euler_matrix

def main():
   
    ## Initialization ##
  
    rospy.init_node("floor_to_table")
    trajectory_generator = TrajectoryGenerator()
    trajectory_executor = TrajectoryExecutor()
    grasping_client = GripperInterfaceClient()

    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    move_base = MoveBaseClient()

 
    ## Raise Torso ##
    torso_action.move_to([0.35, ])
    

    ## Point head at floor ##
    head_action.look_at(0.7, .0, 0.95, "base_link")
    ## Declare sought object ##
    grasping_client.set_target_object("Box", [0.5, 0.0, 0.8, 0, 0, 0], [0.09, 0.04, .195])

    ## Recognize bench object ##
    trajectories = grasping_client.find_graspable_object() 

    ## Pick bench object ##
    trajectories = grasping_client.plan_pick() 
    trajectory_executor.execute_trajectories(trajectories, error_checking = False)
    grasping_client.recognizeAttachedObject()

    tuck_state = JointSpaceGoal([1.32, 1.40, -0.2, 1.72, 0.0, 1.55, 0.0])
    trajectory = trajectory_generator.generate_trajectories(tuck_state, get_approval=True)
    trajectory_executor.execute_trajectories(trajectory)

    torso_action.move_to([0.05, ])
    move_base.goto(0, 0, 1.57)
    head_action.look_at(0.9, -0.7, 0.2, "base_link")
    grasping_client.removeCollisionObjects()   

    trajectories = grasping_client.plan_place([0.70, -0.15, 0.65, 0.0, 0.0, 0.0])
    trajectory_executor.execute_trajectories(trajectories, error_checking=False)

    trajectory_generator.clear_virtual_arm_state()
    trajectory = trajectory_generator.generate_trajectories(tuck_state, get_approval=True)
    trajectory_executor.execute_trajectories(trajectory)

   

if __name__ == "__main__":
   main()






