#! /usr/bin/env python
# Written by: Daniel Palmer
import sys
import copy
import rospy
import actionlib
import tf
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

class TrajectorySimulator(): 
       
        def __init__(self): 
	  # Instantiate a RobotCommander object.
          # This object is an interface to the robot as a whole.
          #robot = moveit_commander.RobotCommander()
	  # Instantiate a MoveGroupCommander object.
	  # This object is an interface to one 'group' of joints,
	  # in our case the joints of the arm.
	  self.group = MoveGroupInterface("arm", "base_link")

	  # Create DisplayTrajectory publisher to publish trajectories
	  # for RVIZ to visualize.
	  # self.display_trajectory_publisher = rospy.Publisher(
	  				    #"/move_group/display_planned_path",
	  				    #moveit_msgs.msg.DisplayTrajectory)

	  # Sleep to allow RVIZ time to start up.
	  print "============ Waiting for RVIZ..."
          rospy.sleep(15)
	  print "============ Starting tutorial "

	def get_state_info(self):
		# Let's get some basic information about the robot.
		# This info may be helpful for debugging.

		# Get name of reference frame for this robot
		print "============ Reference frame: %s" % self.group.get_planning_frame()

		# Get name of end-effector link for this group
		print "============ Reference frame: %s" % self.group.get_end_effector_link()

		# List all groups defined on the robot
		print "============ Robot Groups:"
		print self.robot.get_group_names()

		# Print entire state of the robot
		print "============ Printing robot state"
		print self.robot.get_current_state()
		print "============"

	def plan_to_pose_goal(self, goal):
		### Planning to a Pose Goal

		# Let's plan a motion for this group to a desired pose for the
		# end-effector

		print "============ Generating plan 1"
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
	          result = self.group.moveToPose(pose_target, "gripper_link", tolerance = 0.3, plan_only=False)
                  if result.error_code.val == MoveItErrorCodes.SUCCESS:
                     print "================ Pose Achieved"
                     return 

		# Now we call the planner to compute the plan and visualize it if successful.
		# We are just planning here, not asking move_group to actually move the robot.

		print "============ Waiting while RVIZ displays plan1..."
		rospy.sleep(5)
                print "============ Goal achieved"

		# group.plan() automatically asks RVIZ to visualize the plan,
		# so we need not explicitly ask RVIZ to do this.
		# For completeness of understanding, however, we make this
		# explicit request below. The same trajectory should be displayed
		# a second time.

		print "============ Visualizing plan1"
                """
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan1)
		display_trajectory_publisher.publish(display_trajectory);

		print "============ Waiting while plan1 is visualized (again)..."
		rospy.sleep(5)
                """

	def plan_to_jointspace_goal(self, pose):

                # Let's set a joint-space goal and visualize it in RVIZ
                # A joint-space goal differs from a pose goal in that 
                # a goal for the state of each joint is specified, rather
                # than just a goal for the end-effector pose, with no
                # goal specified for the rest of the arm.
                joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                          "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint",
                          "wrist_roll_joint"]
                while not rospy.is_shutdown():
                    result = self.group.moveToJointPosition(joints, pose, 0.1)
                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                       print "============ Pose Achieved"
                       rospy.sleep(5)
                       return "Success"


class FollowTrajectoryClient():
  
    def __init__(self):
        print "================== Constructing Client"
        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        print "================ Client constructed"
        self.client.wait_for_server()
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        rospy.sleep(15)

    def move_to(self, positions, duration=5.0):
        print "============= Moving"
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
      
class StateListener():
      
     def __init__(self):
        self.subs = rospy.Subscriber('joint_states', JointState, self.callback)
        self.last_data = None
        #rospy.spin()
            
     def callback(data):
    	print 'Data: ', data
        self.last_data = data



def wave_arm(joint_state, args):
         motion_count = args[0]
         t_sim = args[1]
         rospy.loginfo("Measured State is %s", joint_state.position)
         shoulder_pan_joint = joint_state.position[0]
         shoulder_lift_joint = joint_state.position[1]
         upperarm_roll_joint = joint_state.position[2]
         elbow_flex_joint = joint_state.position[3]
         forearm_roll_joint = joint_state.position[4]
         wrist_flex_joint = joint_state.position[5]
         wrist_roll_joint = joint_state.position[6]
         result = t_sim.plan_to_jointspace_goal([shoulder_pan_joint, shoulder_lift_joint,
                                        upperarm_roll_joint, elbow_flex_joint,
                                        forearm_roll_joint, wrist_flex_joint,
                                        wrist_roll_joint])
         


if __name__ == "__main__":
  
  # Create a Node
  rospy.init_node('new_trajectory_audit')
  rospy.loginfo('Instantiating RobotCommander object')
  # Instantiate listener object

  #subs = rospy.Subscriber('joint_states', JointState, joint_callback)

  jl = StateListener()

  # Instantiate TrajectorySimulator object
  #pose1 = [1.0, 1.57, 1.57, 1.57, 1.57, 0.5, 0.5]
  pose2 = [0.5,-0.5, 1.0, 0.0, -1.57, 0.0]
  #result = t_sim.plan_to_jointspace_goal(pose1)
  print "============== About to enter loop..."
  #listener = StateListener()
  print '============', listener
  #pose2 = [0, 0, 0, 1.0, 0, 0]
  #

  

  # say the trajectory has now finished
  #finish_time = rospy.time.now()
  #while jl.last_data.header.time < finish_time:
  #	rospy.sleep(0.1)

  # at this point we are confident we have a new joint update since the trajectory came in
  

   
  #pose3 = [0.5, -0.5, 1.0, 0.0, -1.57, -1.1]
  #t_sim.plan_to_jointspace_goal(pose3)
  #client = FollowTrajectoryClient()
  #client.move_to([0.9, 0.1, 0.8, 0.0, 0.0, 0.0, 0.0])



