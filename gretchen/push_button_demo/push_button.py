#! /usr/bin/env python

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
    #scene_pub = rospy.Publisher('planning_scene',
    #                             PlanningScene,
    #                             queue_size=10) 


    rospy.loginfo("Waiting for get_position_ik...")
    rospy.wait_for_service('compute_ik')
    get_position_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    rospy.loginfo("...connected.")


    ## Map button world frame pose to base_link frame pose of gripper ##
    button_center = None
    while button_center is None:
        rospy.sleep(1.0)
        button_center = button_finder.getButtonLocation()
    #button_center = [1.08, 0.06, 1.02]
    desired_gripper_translation = 0.08
    push_depth = -.015

    gripper_offset = .16645
    desired_gripper_translation += gripper_offset
    push_depth -= gripper_offset
    pre_push_pose = PoseGoal([button_center[0]-desired_gripper_translation,
                         button_center[1], button_center[2], 0,0,0])
    print pre_push_pose.pose
   

    pre_push_posture = GripperPostureGoal(-0.5)
    push = [button_center[0] + push_depth, button_center[1], button_center[2]]
    post_push_retreat = [button_center[0]-desired_gripper_translation,
                         button_center[1], button_center[2]]
    post_push_posture = GripperPostureGoal(1.0)
    post_push_pose = PoseGoal([post_push_retreat[0], post_push_retreat[1], post_push_retreat[2],
                                                0, 0, 0])
    tuck_state = JointSpaceGoal([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])


    pre_push_goals = [pre_push_pose, pre_push_posture]

    pre_push_trajectories = trajectory_generator.generate_trajectories(pre_push_goals, get_approval=True)
    pre_push_state = trajectory_generator.get_virtual_arm_state()
    print pre_push_state.joint_state.position
    trajectory_executor.execute_trajectories(pre_push_trajectories, error_checking=False)  
      

    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x = push[0]
    ps.pose.position.y = push[1]
    ps.pose.position.z = push[2]
    ps.pose.orientation.w = 1.0

    ik_request = PositionIKRequest()
    ik_request.group_name = 'arm'
    ik_request.pose_stamped = ps

    while True:
          response = get_position_ik(ik_request)
          print response
          user = raw_input("Accept position? Y/N")
          if user == 'Y':
             break
    push_state = response.solution.joint_state.position[6:13]
    #state = get_position_ik(group = 'arm')
    print response.solution.joint_state.name[6:13]
    print push_state
    print pre_push_state

    raw_input('Press ENTER to proceed')	
    arm_action.move_to(push_state) 

    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x = push[0] - .08
    ps.pose.position.y = push[1]
    ps.pose.position.z = push[2]
    ps.pose.orientation.w = 1.0

    ik_request = PositionIKRequest()
    ik_request.group_name = 'arm'
    ik_request.pose_stamped = ps

    while True:
          response = get_position_ik(ik_request)
          print response
          user = raw_input("Accept position? Y/N")
          if user == 'Y':
             break
    inter_state_1 = response.solution.joint_state.position[6:13]
    print inter_state_1
    """
    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x = push[0] - .025 * 2.0/3.0
    ps.pose.position.y = push[1]
    ps.pose.position.z = push[2]
    ps.pose.orientation.w = 1.0

    ik_request = PositionIKRequest()
    ik_request.group_name = 'arm'
    ik_request.pose_stamped = ps

    while True:
          response = get_position_ik(ik_request)
          print response
          user = raw_input("Accept position? Y/N")
          if user == 'Y':
             break
    inter_state_2 = response.solution.joint_state.position[6:13]
    print inter_state_2
    """


    """
    ps = PlanningScene()
    ps.is_diff = True
    print ps
    ls_l = LinkScale()   
    ls_l.link_name = "l_gripper_finger_link"
    print "+++++SCALE: ", ls_l.scale
    ls_l.scale = .01

    ls_r = LinkScale()
    ls_r.link_name = "r_gripper_finger_link"
    ls_r.scale = .01
    
    ps.link_scale.append(ls_l)
    ps.link_scale.append(ls_r)
    scene_pub.publish(ps)
    """

 
    #arm_action.move_to(inter_state_1) 
    #arm_action.move_to(inter_state_2)
    post_push_state = pre_push_state.joint_state.position
    #arm_action.move_to(post_push_state)


    post_push_goals =  [tuck_state]
    """
    ps = PlanningScene()
    ps.is_diff = True
    ls_l = LinkScale()   
    ls_l.link_name = "l_gripper_finger_link"
    ls_l.scale = .06

    ls_r = LinkScale()
    ls_r.link_name = "r_gripper_finger_link"
    ls_r.scale = .06
    
    ps.link_scale.append(ls_l)
    ps.link_scale.append(ls_r)
    scene_pub.publish(ps)
    """
    trajectory_generator.clear_virtual_arm_state()
    post_push_trajectories = trajectory_generator.generate_trajectories(post_push_goals, get_approval=True)

    trajectory_executor.execute_trajectories(post_push_trajectories, error_checking = False)
    
  

 
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
