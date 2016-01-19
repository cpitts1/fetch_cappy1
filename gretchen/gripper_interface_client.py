# Authors: Daniel Palmer and Cappy Pitts
# Program used to control the arm and gripper on Fetch

# Import libraries and ROS packages
import actionlib
import copy
import rospy
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from grasping_msgs.msg import Object
from moveit_python import (PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_msgs.msg import PlaceLocation, CollisionObject, AttachedCollisionObject
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents 
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix, quaternion_multiply,                                 quaternion_inverse, quaternion_matrix, quaternion_from_matrix

class GripperInterfaceClient():
 
    def __init__(self):
        self.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]

        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action",
                                                   GripperCommandAction)
        rospy.loginfo("Waiting for gripper_controller...")
        self.client.wait_for_server() 
        rospy.loginfo("...connected.")
    
        self.scene = PlanningSceneInterface("base_link")
        self.attached_object_publisher = rospy.Publisher('attached_collision_object',
                                                         AttachedCollisionObject,
                                                         queue_size = 10)
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True, plan_only=True)

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()
        self.debug_index = -1

        self.graspables = []

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
            self.scene.removeCollisionObject(name, wait=False)
        
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, wait=False)
        
        rospy.sleep(0.5)   # Gets rid of annoying error messages stemming from race condition.
                            
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

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0] + 0.1,
                                            1.5,  # wider
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
            #if obj.object.primitives[0].dimensions[0] < 0.05 or \
            #   obj.object.primitives[0].dimensions[0] > 0.07 or \
            #   obj.object.primitives[0].dimensions[0] < 0.05 or \
            #   obj.object.primitives[0].dimensions[0] > 0.07 or \
            #   obj.object.primitives[0].dimensions[0] < 0.05 or \
            #   obj.object.primitives[0].dimensions[0] > 0.07:
            #    continue
            # has to be on table
            #if obj.object.primitive_poses[0].position.z < 0.5:
            #    continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def set_target_object(self, primitive_type, target_pose, dimensions):
            primitive_types = {"BOX":1, "SPHERE":2, "CYLINDER":3, "CONE":4}
            self.sought_object = Object()
            primitive = SolidPrimitive()
            primitive.type = primitive_types[primitive_type.upper()]
            primitive.dimensions = dimensions
            self.sought_object.primitives.append(primitive)

            p = Pose()
            p.position.x = target_pose[0]
            p.position.y = target_pose[1]
            p.position.z = target_pose[2]

            quaternion = quaternion_from_euler(target_pose[3], target_pose[4], target_pose[5])
            p.orientation.x = quaternion[0]
            p.orientation.y = quaternion[1]
            p.orientation.z = quaternion[2]
            p.orientation.w = quaternion[3]

            self.sought_object.primitive_poses.append(p)


    def find_graspable_object(self, tolerance=0.01):
        self.updateScene()
        self.current_grasping_target = None
        for obj in self.objects:
            # Must have grasps
            if len(obj.grasps) < 1:
               continue
            # Compare to sought object
            detected_object_dimensions = np.array(obj.object.primitives[0].dimensions)
            sought_object_dimensions = np.array(self.sought_object.primitives[0].dimensions)
            try:
               difference = detected_object_dimensions - sought_object_dimensions
               print "===DEBUG: Difference: " , difference
               mag_diff = np.linalg.norm(difference)
               print "===DEBUG: mag_diff: ", mag_diff
               if mag_diff <= tolerance:
                  self.current_grasping_target = obj
                  tolerance = mag_diff
                  print "===DEBUG: Tolerance: ", tolerance
               else:
                  rospy.loginfo("Object dimensions do not match. Trying another object...")
            except:
               rospy.loginfo("Object geometries do not match. Trying another object...")
        if self.current_grasping_target is None:
           tolerance += .01
           self.find_graspable_object(tolerance = tolerance)
        # Nothing detected
    
    def computeGraspToPickMatrix(self):
        pick_rot = self.current_grasping_target.object.primitive_poses[0].orientation
        grasp_rot = self.pick_result.grasp.grasp_pose.pose.orientation 
        pick_translation_distance = self.pick_result.grasp.pre_grasp_approach.desired_distance
        pick_quaternion = [pick_rot.x, pick_rot.y, pick_rot.z, pick_rot.w]
        grasp_quaternion = [grasp_rot.x, grasp_rot.y, grasp_rot.z, grasp_rot.w]
        pick_to_grasp_quaternion = quaternion_multiply(quaternion_inverse(grasp_quaternion), pick_quaternion)
        rotation_matrix = quaternion_matrix(pick_to_grasp_quaternion)   

        translation = [pick_translation_distance, 0, 0]
        rotation_matrix[[0,1,2],3] = translation
        pick_to_grasp_matrix = np.mat(rotation_matrix)
        grasp_to_pick_matrix = pick_to_grasp_matrix.getI()
        return grasp_to_pick_matrix
       
 
    def computePlaceToBaseMatrix(self, place):
        place_quaternion = place[3:]
        rotation_matrix = quaternion_matrix(place_quaternion)
        translation = place[0:3]
        rotation_matrix[[0,1,2], 3] = translation
        place_to_base_matrix = rotation_matrix
        return place_to_base_matrix


    def broadcastCurrentGraspingTargetTransform(self):
        pose = self.current_grasping_target.object.primitive_poses[0]
        br = tf2_ros.TransformBroadcaster()
 
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "current_grasping_target"
       
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        br.sendTransform(t)
       
 
    def plan_pick(self):
        success, pick_result = self.pickplace.pick_with_retry(self.current_grasping_target.object.name,
                                                              self.current_grasping_target.grasps,
                                           support_name = self.current_grasping_target.object.support_surface,
                                                              scene=self.scene,
                                                              allow_gripper_support_collision=False,
                                                              planner_id='PRMkConfigDefault')
        self.pick_result = pick_result
        print "DEBUG: plan_pick(): ", self.scene.getKnownAttachedObjects()
        if success:
           rospy.loginfo("Planning succeeded.")
           trajectory_approved = get_user_approval()
           if trajectory_approved:
              return pick_result.trajectory_stages 
           else:
              rospy.loginfo("Plan rejected. Getting new grasps for replan...")
        else: 
           rospy.logwarn("Planning failed. Getting new grasps for replan...")
        self.find_graspable_object()
        return self.plan_pick()
        


    def plan_place(self, desired_pose):
            places = list()
            ps = PoseStamped()
            #ps.pose = self.current_grasping_target.object.primitive_poses[0]
            ps.header.frame_id = self.current_grasping_target.object.header.frame_id

            grasp_to_pick_matrix = self.computeGraspToPickMatrix()
            place_to_base_matrix = self.computePlaceToBaseMatrix(desired_pose)
            grasp2_to_place_matrix = place_to_base_matrix * grasp_to_pick_matrix
            position_vector = grasp2_to_place_matrix[0:-1,3]
            quaternion = quaternion_from_matrix(grasp2_to_place_matrix)
            position_array = position_vector.getA1()
          
            l = PlaceLocation()
            direction_components = self.pick_result.grasp.pre_grasp_approach.direction.vector
            l.place_pose.header.frame_id = ps.header.frame_id
            l.place_pose.pose.position.x = position_array[0]
            l.place_pose.pose.position.y = position_array[1]
            l.place_pose.pose.position.z = position_array[2]
            l.place_pose.pose.orientation.x = quaternion[0]
            l.place_pose.pose.orientation.y = quaternion[1]
            l.place_pose.pose.orientation.z = quaternion[2]
            l.place_pose.pose.orientation.w = quaternion[3]

            # copy the posture, approach and retreat from the grasp used
            approach = copy.deepcopy(self.pick_result.grasp.pre_grasp_approach)
            approach.desired_distance /= 2.0

            l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
            l.pre_place_approach = approach
            l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
            places.append(copy.deepcopy(l))
            # create another several places, rotate each by 90 degrees in roll direction
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 1.57, 0, 0)
            places.append(copy.deepcopy(l))
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 1.57, 0, 0)
            places.append(copy.deepcopy(l))
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 1.57, 0, 0)
            places.append(copy.deepcopy(l))
            print "DEBUG: Places: ", places[0]
            success, place_result = self.pickplace.place_with_retry(self.current_grasping_target.object.name,
                                                                places,
                                                                scene=self.scene,
                                                                allow_gripper_support_collision=False,
                                                                planner_id='PRMkConfigDefault')
            if success:
               rospy.loginfo("Planning succeeded.")
               trajectory_approved = get_user_approval()
               if trajectory_approved:
                  return place_result.trajectory_stages
               else:
                  rospy.loginfo("Plan rejected. Replanning...")
            else:
               rospy.logwarn("Planning failed. Replanning...")
            desired_pose[2] += 0.05
            return self.plan_place(desired_pose)


    def recognizeAttachedObject(self):
        print "========= Recognizing attached object"
        name = self.current_grasping_target.object.name
        size_x = self.current_grasping_target.object.primitives[0].dimensions[0]
        size_y = self.current_grasping_target.object.primitives[0].dimensions[1]
        size_z = self.current_grasping_target.object.primitives[0].dimensions[2]
        (x, y, z) = (0.03, 0.0, 0.0)
        link_name  = "gripper_link"
        touch_links = ["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"]
        detach_posture = None
        weight = 0.0
        wait = True 
        """ 
        object_x = self.current_grasping_target.object.primitive_poses[0].position.x
        object_y = self.current_grasping_target.object.primitive_poses[0].position.y
        object_z = self.current_grasping_target.object.primitive_poses[0].position.z


        pick_rot = self.current_grasping_target.object.primitive_poses[0].orientation
        grasp_rot = self.pick_result.grasp.grasp_pose.pose.orientation 
        grasp_position = self.pick_result.grasp.grasp_pose.pose.position
        pick_translation_distance = self.pick_result.grasp.post_grasp_retreat.desired_distance
        pick_quaternion = [pick_rot.x, pick_rot.y, pick_rot.z, pick_rot.w]
        grasp_quaternion = [grasp_rot.x, grasp_rot.y, grasp_rot.z, grasp_rot.w]
        pick_to_grasp_quaternion = quaternion_multiply(quaternion_inverse(grasp_quaternion), pick_quaternion)

        grasp_to_base_matrix_array = quaternion_matrix(grasp_quaternion)
        grasp_to_base_matrix = np.matrix(grasp_to_base_matrix_array)
        displacement = grasp_to_base_matrix.dot(np.matrix([[pick_translation_distance-.03], [0], [0], [0]]))
        print displacement

        attached_location = list()
        attached_location.append(object_x - displacement[0,0])
        attached_location.append(object_y - displacement[1,0])
        attached_location.append(object_z - displacement[2,0])       
        print attached_location
        

        quaternion = Quaternion()
        quaternion.x = pick_to_grasp_quaternion[0]
        quaternion.y = pick_to_grasp_quaternion[1]
        quaternion.z = pick_to_grasp_quaternion[2]
        quaternion.w = pick_to_grasp_quaternion[3]

        pose = Pose()
        pose.position.x = attached_location[0]
        pose.position.y = attached_location[1]
        pose.position.z = attached_location[2]

        pose.orientation = self.current_grasping_target.object.primitive_poses[0].orientation


        collision_object = self.scene.makeSolidPrimitive(self.current_grasping_target.object.name,
                                                     self.current_grasping_target.object.primitives[0],
                                                     pose)
                                                         
                                                          
        attached_collision_object = self.scene.makeAttached("gripper_link",
                                                       collision_object,
                                                       touch_links, detach_posture, 0.0)
        

        self.attached_object_publisher.publish(attached_collision_object)
        """
        self.scene.attachBox(name, size_x, size_y, size_z, x, y, z, link_name,
                              touch_links=touch_links, detach_posture=detach_posture, weight=weight,
                              wait=wait)


    def removeCollisionObjects(self):
        collision_object_names = self.scene.getKnownCollisionObjects()
        print self.current_grasping_target.object.name
        print collision_object_names
        raw_input("press ENTER")
        x = self.scene.getKnownAttachedObjects()
        print x
        raw_input("press Enter")
        for name in collision_object_names:
            if name != self.current_grasping_target.object.name:
               self.scene.removeCollisionObject(name, wait=False)

   
def get_user_approval():
    choice = 'not_set'
    options = {'y':True, 'r':False}
    while choice.lower() not in options.keys():
       choice = raw_input("Execute plan now? Answer 'y' for yes "\
					" or 'r' for replan: ")
    return options[choice]


