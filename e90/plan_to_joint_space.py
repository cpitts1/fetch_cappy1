#! /usr/bin/env python
# Author: Cappy Pitts

# This program places an object above the point that is obtained in
# the class ButtonLocationSubscriber and then tucks the arm of the Fetch Robot

import actionlib
import copy
import rospy

from follow_trajectory_client import FollowTrajectoryClient
from trajectory_execution import TrajectoryExecutor
from plan_to_goal import TrajectoryGenerator, PoseGoal, GripperPostureGoal, JointSpaceGoal
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import LinkScale, PlanningScene, PositionIKRequest
from moveit_msgs.srv import GetPositionIK

def main():
    
    ## Initialization ##
    rospy.init_node("joint_space_goal")
    trajectory_generator = TrajectoryGenerator()
    trajectory_executor = TrajectoryExecutor()

    arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint",
                   "upperarm_roll_joint", "elbow_flex_joint", 
                    "forearm_roll_joint", "wrist_flex_joint",
                     "wrist_roll_joint"]
    arm_action = FollowTrajectoryClient('arm_controller', arm_joints)
    torso_action = FollowTrajectoryClient('torso_controller', ["torso_lift_joint"])

    rospy.loginfo("Waiting for get_position_ik...")
    rospy.wait_for_service('compute_ik')
    get_position_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    rospy.loginfo("...connected.")

    joint_start = JointSpaceGoal([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])
    joint_end = JointSpaceGoal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    #tuck_state = JointSpaceGoal([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])
    print joint_start
    print joint_end
    
    joint_start_traj = trajectory_generator.generate_trajectories(joint_start, get_approval=False)
    joint_start_state = trajectory_generator.get_virtual_arm_state()
    trajectory_executor.execute_trajectories(joint_start_traj, error_checking=False)  
  
    #post_push_posture = GripperPostureGoal(1.0)
    #tuck_state = JointSpaceGoal([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])

    trajectory_generator.clear_virtual_arm_state()
    joint_end_traj = trajectory_generator.generate_trajectories(joint_end, get_approval=False)
    trajectory_executor.execute_trajectories(joint_end_traj, error_checking = False)
    

class ButtonLocationSubscriber():
    def __init__(self):
        rospy.Subscriber("button_point", PointStamped, self.get_button_center_CB)

    def get_button_center_CB(self, point_stamped):
        self.point = [point_stamped.point.x, point_stamped.point.y, point_stamped.point.z]
 
    def getButtonLocation(self):
        try:
          point = copy.deepcopy(self.point)
          print "+++++Point: ", point
          return point
        except AttributeError:
          rospy.loginfo("No point available.")


if __name__ == "__main__":
   main()
