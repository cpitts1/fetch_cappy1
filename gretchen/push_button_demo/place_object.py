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
    rospy.init_node("push_button")
    trajectory_generator = TrajectoryGenerator()
    trajectory_executor = TrajectoryExecutor()
    button_finder = ButtonLocationSubscriber()

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

    ## Map button world frame pose to base_link frame pose of gripper ##
    button_center = None
    #approx coords: 0.796538999146 -0.0150021952933 0.264828975569

    while button_center is None:
        rospy.sleep(1.0)
        button_center = button_finder.getButtonLocation()

    desired_gripper_translation = .16645
    pre_push_pose = PoseGoal([button_center[0]-desired_gripper_translation,
                         button_center[1], button_center[2]+.2, 0,0,0])
    
    print pre_push_pose.pose
    
    pre_push_goals = [pre_push_pose]

    pre_push_trajectories = trajectory_generator.generate_trajectories(pre_push_goals, get_approval=True)
    pre_push_state = trajectory_generator.get_virtual_arm_state()
    trajectory_executor.execute_trajectories(pre_push_trajectories, error_checking=False)  
  
    
    post_push_posture = GripperPostureGoal(1.0)
    tuck_state = JointSpaceGoal([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])

    gripper_goal = [post_push_posture]

    trajectory_generator.clear_virtual_arm_state()
    post_push_trajectories = trajectory_generator.generate_trajectories(gripper_goal, get_approval=False)
    trajectory_executor.execute_trajectories(post_push_trajectories, error_checking = False)
    
    torso_action.move_to([.30, ]) 

    post_push_goals = [tuck_state]
    
    trajectory_generator.clear_virtual_arm_state()
    post_push_trajectories = trajectory_generator.generate_trajectories(post_push_goals, get_approval=True)
    trajectory_executor.execute_trajectories(post_push_trajectories, error_checking = False)
    
    torso_action.move_to([0.05, ])

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
