#!/usr/bin/env python
# Author: Cappy Pitts
import actionlib
import numpy
import rospy
import sys
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

    def move_to(self, positions, velocities, accelerations, delta_t):
        assert(len(self.joint_names) == positions.shape[1])
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        for i in range(positions.shape[0]): # for each row i
            p = JointTrajectoryPoint()
            p.positions = list(positions[i])
            p.velocities = list(velocities[i])
            p.accelerations = list(accelerations[i])
            p.time_from_start = rospy.Duration(i * delta_t)
            trajectory.points.append(p)


        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

    def get_joint_position(self, filename):
	data = numpy.genfromtxt(filename)
	return data

    def move_to_initial(self, positions, duration):    

        assert(len(self.joint_names) == len(positions))
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = positions

        p.time_from_start = rospy.Duration(duration)
        trajectory.points.append(p)

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

    # Get command line arguments or print usage and exit
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        progname = os.path.basename(sys.argv[0])
        print >> sys.stderr, 'usage: '+progname+' filename'
        sys.exit(1)

    # Create a node
    rospy.init_node("trajectory_manager")

    # Make sure sim time is working
    #while not rospy.Time.now():
    #pass

    # Setup FollowTrajectoryClient
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    rospy.loginfo("Subscribing to FollowTrajectoryClient")
    arm_action = FollowTrajectoryClient("arm_controller", joint_names)
    
    #get the joint positions for the arm
    rospy.loginfo("Getting positions, accelerations, and velocities")
    filename = '/home/cpitts1/catkin_ws/src/fetch_cappy/e90/textfiles/'+filename
    joint_pos = arm_action.get_joint_position(filename)
    #safe is probably 1/10
    delta_t = 1.0/20
    joint_vel = difference(joint_pos,delta_t)
    joint_accel = difference(joint_vel,delta_t)

    #joint_pos_zero = list(joint_pos[0])
    #for i in joint_pos_zero:
    #if abs(joint_pos_zero) > abs(something)+.1 or abs(joint_pos_zero) < abs(something)+.1:
    #sys.exit(1)
    
    rospy.loginfo('Sleeping for 2')
    rospy.sleep(2) 

    rospy.loginfo('Move to start position')
    arm_action.move_to_initial(list(joint_pos[0]), 5.0)

    rospy.loginfo('About to move end effector')
    arm_action.move_to(joint_pos, joint_vel, joint_accel, delta_t)
    rospy.loginfo('Completed motion')
