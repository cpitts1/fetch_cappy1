#!/usr/bin/env python
# Author: Cappy Pitts
import actionlib
import numpy
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, velocities, accelerations, duration=5.0):
    #def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = velocities
        #trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = accelerations
        #trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

    def get_joint_position(self, filename):
	data = numpy.genfromtxt(filename)
	return data

# f'(x) ~ (-3*f(x) + f(x+h) - f(x+2*h)) / (2*h)
def three_point_forward(x0, x1, x2, h):
    return (-3.0*x0 + 4.0*x1 - x2)/(2.0*h)

def difference(a, h):
    result = (a[2:,:] - a[:-2,:]) / (2*h)
    first = three_point_forward(a[0], a[1], a[2], h)
    last = -three_point_forward(a[-1], a[-2], a[-3], h)
    result = numpy.vstack((first, result, last))
    return result


if __name__ == "__main__":

    # Create a node
    rospy.init_node("trajectory_manager")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup FollowTrajectoryClient
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    arm_action = FollowTrajectoryClient("arm_controller", joint_names)
    
    #get the joint positions for the arm
    joint_pos = arm_action.get_joint_position('/home/cpitts1/catkin_ws/src/fetch_cappy/e90/example.txt')
    
    pos_sides = []
    vel_sides = []
    accel_sides = []
    delta_t = 4.0/len(joint_pos)
    ticks = len(joint_pos)/4
    length = len(joint_pos)
    for i in range(4):
        if i != 3:
            pos_sides.append(joint_pos[(i*ticks):((i+1)*ticks)])
        else:
            pos_sides.append(joint_pos[(i*ticks):length])
        vel_sides.append(difference(pos_sides[i],delta_t))
        accel_sides.append(difference(vel_sides[i],delta_t))
    joint_pos = numpy.vstack((pos_sides))
    joint_vel = numpy.vstack((vel_sides))
    joint_accel = numpy.vstack((accel_sides))
   
    #move to random trajectory
    for i in range(len(joint_pos)):
	arm_action.move_to(list(joint_pos[i]), list(joint_vel[i]), list(joint_accel[i]))
	#arm_action.move_to(list(joint_pos[i]))
