#! /usr/bin/env python
 

import actionlib
import numpy as np
import rospy
import sys

from math import sin, cos

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from control_msgs.msg import (GripperCommandAction,
                              GripperCommandGoal)
from geometry_msgs.msg import PoseStamped
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.srv import ExecuteKnownTrajectory
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal

class TrajectoryGenerator(): 
     # Wrapper class that facilitates trajectory planning.
     # Includes methods for planning to arbitrary joint-space goals
     # and end-effector pose goals.  
       
      def __init__(self): 
          self.group = MoveGroupInterface("arm", "base_link")
          self.gripper = MoveGroupInterface("gripper", "base_link")
          self.virtual_state = RobotState()
          self.trajectory = None
          # Sleep to allow RVIZ time to start up.
          print "============ Waiting for RVIZ..."
          rospy.sleep(5)
          print "============ Starting planning"
 
      def generate_trajectory(self, goal_poses):
        for goal_pose in goal_poses:
          if goal_pose.goal_type is "ee_pose":
             traj_result = self.plan_to_ee_pose_goal(goal_pose.pose)
          elif goal_pose.goal_type is "jointspace":
             traj_result = self.plan_to_jointspace_goal(goal_pose.state)
          elif goal_pose.goal_type is "gripper_posture":
             traj_result = self.plan_to_gripper_goal(goal_pose.state)
          else:
             rospy.loginfo("Goal type improperly specified. Please specify as either 'ee_pose' or 'jointspace'")
             sys.exit()
          approved_trajectories.append(traj_result)
        return approved_trajectories

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
              result = self.group.moveToPose(pose_target, "gripper_link",
                                            tolerance = 0.02, plan_only=True,
                                            start_state = self.virtual_state)
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 print "================ Pose Achieved"
                 self.trajectory = result.planned_trajectory.joint_trajectory
                 return self.trajectory

      def plan_to_jointspace_goal(self, pose):
          
          joints = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]
          while not rospy.is_shutdown():
              result = self.group.moveToJointPosition(joints, pose, plan_only=True,
                                                     start_state=self.virtual_state
                                                     )
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 print "============ Pose Achieved"
                 self.trajectory = result.planned_trajectory  #.joint_trajectory
                 return self.trajectory

      def plan_to_gripper_goal(self, posture):
          joints = ["gripper_link", "l_gripper_finger_link", "r_gripper_finger_link"]
          while not rospy.is_shutdown():
              result = self.gripper.moveToJointPosition(joints, pose, plan_only=True,
                                                        start_state=self.virtual_state)
              if result.error_code.val == MoveItErrorCodes.SUCCESS:
                 rospy.loginfo("Gripper Posture Plan Successful")
                 self.trajectory = result.planned_trajectory.joint_trajectory
                 return self.trajectory
          

      def update_virtual_state(self):
          # Sets joint names and sets position to final position of previous trajectory 
          if self.trajectory == None:
             print "No trajectory available to provide a new virtual state."
             print "Update can only occur after trajectory has been planned."
          else:
             self.virtual_state.joint_state.name = ["shoulder_pan_joint",
                           "shoulder_lift_joint", "upperarm_roll_joint",
                             "elbow_flex_joint",   "forearm_roll_joint",
                               "wrist_flex_joint",   "wrist_roll_joint"]
             self.virtual_state.joint_state.position = self.trajectory.points[-1].positions
  
      def clear_virtual_state(self):
          self.virtual_state.joint_state.joint_names = []
          self.virtual_state.joint_state.position = []
 
      def execute_trajectory(self, trajectory):
    
          execute_known_trajectory = rospy.ServiceProxy('execute_known_trajectory', ExecuteKnownTrajectory)
          result = execute_known_trajectory(trajectory)



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
            if abs(diff) > .005:
               rospy.loginfo('Diff: ' + str(diff))
               rospy.loginfo('=============== Joint Goal NOT Achieved')
               rospy.loginfo('=============== Current state is: ' + str(self.measured_state))
               return False
            else:
               rospy.loginfo('Joint %d Goal Achieved' %(joint_number))
               joint_number += 1
        rospy.loginfo('=============== Goal Pose Achieved')
        return True

      
class Executor():
  
    def __init__(self):

        self.arm_action = ArmInterfaceClient()
        self.gripper_action = GripperInterfaceClient()
        self.arm_and_torso_action = ArmAndTorsoClient()
          
 
    def execute_trajectories(self, trajectories):
        #trajectories.pop(1)  # Quick n' dirty patch to remove 'pre-grasp' section of pick trajectory
        #for trajectory in trajectories:
        rospy.sleep(2.0)
        num_commanded_joints = len(trajectory.joint_trajectory.joint_names)
        if num_commanded_joints is 7:
           self.arm_action.execute_trajectory(trajectory.joint_trajectory)
        elif num_commanded_joints is 2:
            self.gripper_action.execute_trajectory(trajectory.joint_trajectory)
        elif num_commanded_joints is 8:
             self.arm_and_torso_action.execute_trajectory(trajectory.joint_trajectory)
        else:
             print("No controller exists for planning group %s"
                        %trajectory.joint_trajectory.joint_names)
       
    
class GripperInterfaceClient():
 
    def __init__(self):
        self.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]

        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action",
                                                   GripperCommandAction)
        rospy.loginfo("Waiting for gripper_controller...")
        self.client.wait_for_server() 
    
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True, plan_only=True)

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

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


    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         wait = False)
        # Try mesh representation
        #idx = -1
        #for obj in find_result.objects:
        #    idx += 1
        #    obj.object.name = "object%d"%idx
        #    self.scene.addMesh(obj.object.name,
        #                       obj.object.mesh_poses[0],
        #                       obj.object.meshes[0],
        #                       wait = False)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0] + 0.1,
                                            2.5,  # wider
					    obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         wait = False)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            # if obj.object.primitives[0].dimensions[0] < 0.03 or \
            #   obj.object.primitives[0].dimensions[0] > 0.04 or \
            #   obj.object.primitives[0].dimensions[0] < 0.07 or \
            #   obj.object.primitives[0].dimensions[0] > 0.09 or \
            #   obj.object.primitives[2].dimensions[2] < 0.19 or \
            #   obj.object.primitives[2].dimensions[2] > 0.20:
            #    continue
            # has to be on table
            #if obj.object.primitive_poses[0].position.z < 0.5:
            #    continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def plan_pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              allow_gripper_support_collision=False,
                                                              planner_id = 'PRMkConfigDefault',
                                                              retries = 2,
                                                              scene=self.scene)
        #self.pick_result = pick_result
        if success:
           return pick_result.trajectory_stages
        else:
           rospy.loginfo("Planning Failed.")
           return None
        
class ArmAndTorsoClient():
  
    def __init__(self):
        pass



class BenchmarkTrajectories():
   # 'Library' of test trajectories
   def __init__(self):
       pass
 
   def wave_hand(self, num_waves = 3):
       # Wave hand benchmark trajectory.
       # Robot waves its hand in a friendly greeting. 
       goal_state = [-1.6, 0.48, 0, -1.73, 0, -.99, -1.54]
       state = JointSpaceGoal(goal_state[:])
       state_sequence = []
       state_sequence.append(state)
       for n in range(0, num_waves):
           goal_state[5] = goal_state[5] + (-1)**(n) * 1.0
           state_sequence.append(JointSpaceGoal(goal_state[:]))
       
       return state_sequence


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
          num_args = len(posture)
          if num_args == 1:
             self.posture = posture
          else:
             rospy.loginfo("GripperPostureGoal requires exactly 1 argument: %d given" %num_args)
           

class FollowTrajectoryClient(object):
    # Executes 'handmade' trajectories
    # Mostly junk, will likely be removed soon
    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
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
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()


class PointHeadClient(object):
    # Use a controller to point the head
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


def ask_for_replan():
    choice = 'not_set'
    options = {'r':True, 'q':False}
    while choice.lower() not in choices.keys():
        choice = raw_input("Replan (r) using current waypoints, or quit (q)?: ")
    return options[choice]


def get_user_approval():
    choice = 'not_set'
    options = {'y':True, 'r':False}
    while choice.lower() not in options.keys():
       choice = raw_input("Execute plan now? Answer 'y' for yes "\
					" or 'r' for replan: ")
    return options[choice]


def subtract_angles(angle_i, angle_j): 

    # Subtraction of two angles using vector method
    # Robust to modality issue occuring for angles having difference
    # greater than +/- pi. 
    pos_vector_i = np.array([np.cos(angle_i), np.sin(angle_i)])
    pos_vector_j = np.array([np.cos(angle_j), np.sin(angle_j)])
    cos_of_angle_i_j = np.dot(pos_vector_i, pos_vector_j)
    angle_i_j = np.arccos(cos_of_angle_i_j)
    
    return angle_i_j


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()



if __name__ == "__main__":
  
   # Create a Node
   rospy.init_node('trajectory_audit')
   
   # Initialization 
   move_base = MoveBaseClient()
   torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
   head_action = PointHeadClient()
   trajectory_generator = TrajectoryGenerator()
   arm_interface = ArmInterfaceClient()
   benchmark_trajectories = BenchmarkTrajectories()
   grasping_client = GripperInterfaceClient()
   executor = Executor()

   joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]

   ## Control Panel ##
   motion_goals = [True]
   #motion_goals = benchmark_trajectories.wave_hand()
   #motion_goals.append(PoseGoal([0.8, 0.0, 0.8, 0.0, 0.75, 1.0]))
    
   x = JointSpaceGoal([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
   #grasping_client.move_to(0.10, 10) 
   #move_base.goto(0.5,0.0,0.0)
   #torso_action.move_to([0.4, ])
   #head_action.look_at(1.2, 0.0, 0.83, "map")


   ###################


   
   goal_achieved = False
   while not goal_achieved:
      # Planning
      trajectories = []
      
      
      
      for motion_goal in motion_goals:
          trajectory_approved = False
	  while not trajectory_approved:
	     #trajectory = trajectory_generator.generate_trajectory(motion_goal)
             # Find Graspable Objects
             while not rospy.is_shutdown():
                 rospy.loginfo("Looking for Object...")
                 grasping_client.updateScene()
                 cube, grasps = grasping_client.getGraspableCube()
                 if cube is not None:
                    rospy.loginfo("Perception successful.")
                    trajectory = grasping_client.plan_pick(cube, grasps)
                    if trajectory is None:
                       continue
                    else:
                       break
           
                 else:
                    rospy.logwarn("Perception failed.")
             trajectory_approved = get_user_approval()
	     if trajectory_approved:
                rospy.loginfo("Trajectory Approved. Adding to Execution Sequence...")
		trajectories.append(trajectory)
		# Update virtual initial state for next segment of trajectory
                trajectory_generator.update_virtual_state()
	     else:
                rospy.loginfo("Replanning Trajectory")

      # Execution
      for trajectory in trajectories[0]:
          executor.execute_trajectories(trajectory)
	  #arm_interface.execute_trajectory(trajectory)
	  goal_achieved = executor.arm_action.verify_goal_achievement()
	  if not goal_achieved:
	     replan = ask_for_replan()
             if replan:
	        # Isolate downstream trajectories
	        current_trajectory_index = trajectories.index(trajectory)
                motion_goals = motion_goals[current_trajectory_index:]
                # Clear virtual initial state so that planner uses actual robot state
                trajectory_generator.clear_virtual_state()
	        rospy.loginfo("Replanning course from current state...")
                break
	     else:
	        rospy.loginfo("Aborting Module...")
	        sys.exit()
