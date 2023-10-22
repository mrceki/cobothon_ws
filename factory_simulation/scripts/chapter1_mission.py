#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def get_object_poses(scene, object_prefix, max_objects):
    """
    The function `get_object_poses` retrieves the poses (positions and orientations) of objects in a
    scene based on their names and a maximum number of objects to consider.
    
    :param scene: The `scene` parameter is an object that represents the current scene or environment.
    It likely contains information about the objects in the scene, such as their names and poses
    :param object_prefix: The object_prefix parameter is a string that represents the prefix of the
    object names. For example, if the object_prefix is "obj_", the function will look for objects with
    names like "obj_1", "obj_2", etc
    :param max_objects: The parameter `max_objects` is the maximum number of objects you want to
    retrieve poses for
    :return: a dictionary of object poses.
    """
    object_poses = {}
    for i in range(1, max_objects + 1):
        object_name = f"{object_prefix}{i}"
        if scene.get_known_object_names().count(object_name) > 0:
            object_pose = scene.get_object_poses([object_name])
            object_poses[object_name] = object_pose[object_name]
    return object_poses

def modify_pose_z_orientation(pose, z_offset, orientation):
    """
    The function modifies the z position and orientation of a given pose.
    
    :param pose: The pose parameter is an object that represents the position and orientation of an
    object in 3D space. It typically contains the following attributes:
    :param z_offset: The z_offset parameter is the amount by which the z-coordinate of the pose's
    position should be modified
    :param orientation: The orientation parameter is a quaternion representing the rotation of the pose.
    It consists of four components: x, y, z, and w
    :return: the modified pose object.
    """
    pose.position.z += z_offset
    pose.orientation.x = orientation.x
    pose.orientation.y = orientation.y
    pose.orientation.z = orientation.z
    pose.orientation.w = orientation.w
    return pose

def move_to_pose(group, pose):
    """
    The function `move_to_pose` takes a robot arm group and a desired pose as input, sets the pose
    target for the group, plans and executes the motion, and logs whether the motion was successful or
    not.
    
    :param group: The "group" parameter is an object representing a group of joints or links in a robot.
    It is typically used to control the motion of the robot arm or end effector
    :param pose: The "pose" parameter is the desired pose that the robot arm should move to. It is
    typically represented as a 6-dimensional vector, specifying the position and orientation of the end
    effector of the robot arm. The exact format of the pose depends on the specific robot arm and its
    kinematics
    """
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Successfully moved to the pose.")
    else:
        rospy.loginfo("Failed to plan and execute the motion.")

def approach_to_object(group, pose):
    """
    The function `approach_to_object` sets the pose target for a robot arm group and executes the motion
    plan to move the arm to that pose.
    
    :param group: The "group" parameter is an object representing a group of joints or links in a robot.
    It is typically used to control the motion of the robot's arm or end effector
    :param pose: The "pose" parameter is the desired pose or position that you want the robot's end
    effector to move to. It is typically represented as a 6-dimensional vector, specifying the position
    and orientation of the end effector in 3D space. The specific format of the pose depends on the
    """
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Successfully moved to the pose.")
    else:
        rospy.loginfo("Failed to plan and execute the motion.")

def attach_object(scene, touch_link, object_name):
    """
    The function attaches an object to a specific link in a scene using the ROS attach service.
    
    :param scene: The `scene` parameter is an object representing the current state of the robot's
    environment. It is used to perform operations such as attaching objects to the robot's gripper
    :param touch_link: The `touch_link` parameter is the name of the link on the robot's gripper that
    will come into contact with the object
    :param object_name: The object_name parameter is the name of the object that you want to attach to
    the gripper
    """
    # scene.attach_box("tcp_link", object_name)
    rospy.loginfo("Attaching object to gripper")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "L6"
    req.model_name_2 = f"{object_name}"
    req.link_name_2 = "link"
    attach_srv.call(req)

def detacht_object(scene, touch_link, object_name):
    """
    The function `detacht_object` detaches an object from a gripper in a scene.
    
    :param scene: The scene parameter is an object representing the current state of the environment or
    scene. It is used to interact with the objects in the scene, such as adding or removing objects
    :param touch_link: The `touch_link` parameter is the name of the link on the robot's gripper that is
    currently in contact with the object you want to detach
    :param object_name: The `object_name` parameter is the name of the object that you want to detach
    from the gripper
    """
    # scene.remove_attached_object(touch_link, name=object_name)
    rospy.loginfo("Detaching object from gripper")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "L6"
    req.model_name_2 = f"{object_name}"
    req.link_name_2 = "link"
    detach_srv.call(req)

def remove_object_from_scene(scene, object_name):
    """
    The function removes an object from a scene.
    
    :param scene: The scene parameter is an object that represents the current scene or environment in
    which objects are placed. It could be a 3D scene, a virtual environment, or any other context where
    objects can be placed and manipulated
    :param object_name: The name of the object you want to remove from the scene
    """
    scene.remove_world_object(object_name)

def pick_object(z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link):
    """
    The function `pick_object` is used to pick up an object by modifying the pose, moving the robot arm
    to the modified pose, attaching the object, and then moving the arm back to the original pose.
    
    :param z_offset: The z_offset parameter is the amount by which the z-coordinate of the pose is
    modified. It determines the vertical position of the object to be picked relative to the original
    pose
    :param pick_orientation: The pick_orientation parameter is the desired orientation of the end
    effector when picking up the object. It specifies the rotation of the end effector around the z-axis
    :param approach_retreat_offset: The approach_retreat_offset parameter is the distance by which the
    robot should approach or retreat from the object before and after picking it up. It is used to
    ensure that the robot has a safe distance from the object during the pick and place operation
    :param scene: The "scene" parameter refers to the scene object in the MoveIt! planning scene. It is
    used to add and remove objects from the scene
    :param group: The "group" parameter refers to the move group that is responsible for controlling the
    robot's motion. It is typically an instance of the MoveGroupCommander class in the MoveIt!
    framework. This group is used to plan and execute the robot's movements
    :param touch_link: The touch_link parameter refers to the link on the robot arm that will make
    contact with the object during the pick operation
    """
    modified_pose = modify_pose_z_orientation(pose, z_offset, pick_orientation)
    rospy.loginfo(modified_pose)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(pose, -approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    attach_object(scene, touch_link, object_name)
    modified_pose = modify_pose_z_orientation(pose, approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)

def place_object(conveyor_pose, z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link):
    """
    The function `place_object` is used to move an object to a specified pose on a conveyor, detach it,
    and remove it from the scene.
    
    :param conveyor_pose: The pose of the conveyor where the object is located. It includes the position
    (x, y, z) and orientation (quaternion) of the object
    :param z_offset: The z_offset parameter is the amount by which the object's z-coordinate is
    modified. It is used to adjust the height at which the object is placed or picked up
    :param pick_orientation: The pick_orientation parameter is the desired orientation of the gripper
    when picking up the object. It specifies the rotation of the gripper around the z-axis
    :param approach_retreat_offset: The approach_retreat_offset parameter is the distance by which the
    robot should move towards the object before picking it up and retreat after picking it up. It is
    used to ensure that the robot can properly grasp the object without colliding with it
    :param scene: The "scene" parameter refers to the scene in which the objects are placed or
    manipulated. It could be a virtual environment or a physical workspace
    :param group: The "group" parameter refers to the robot arm or group of joints that will be used to
    perform the object placement task. It could be a specific arm or a combination of joints that are
    responsible for manipulating objects
    :param touch_link: The touch_link parameter refers to the link on the robot arm that will be used to
    touch or interact with the object
    """
    modified_pose = modify_pose_z_orientation(conveyor_pose, z_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(conveyor_pose, -approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    detacht_object(scene, touch_link, object_name)
    modified_pose = modify_pose_z_orientation(conveyor_pose, approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    remove_object_from_scene(scene, object_name)

# The above code is a Python script that uses the MoveIt! package in ROS (Robot Operating System) to
# control a robotic arm.
if __name__ == '__main__':
    rospy.init_node('move_to_object_poses', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    detach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("orion_arm")  # Kontrol grubu adınıza uygun olarak değiştirin
    touch_link = "tcp_link"
    object_prefix = "unit_box"
    max_objects = 8
    z_offset = 0.24  # Eklemek istediğiniz z değeri
    approach_retreat_offset= 0.16
    place_approach_offset= 0.10
    pick_orientation = geometry_msgs.msg.Quaternion(x=1, y=0, z=0, w=0)  # Yeni orientation değeri

    conveyor_pose = geometry_msgs.msg.Pose()
    conveyor_pose.orientation.w = -0.00276280683465302
    conveyor_pose.orientation.x = 0.7515972852706909 
    conveyor_pose.orientation.y = 0.6595208644866943
    conveyor_pose.orientation.z = -0.01123037189245224
    conveyor_pose.position.x = -0.7
    conveyor_pose.position.y = 0.04
    conveyor_pose.position.z = 0.22574710845947266


    scene = moveit_commander.PlanningSceneInterface()
    object_poses = get_object_poses(scene, object_prefix, max_objects)
    
    for object_name, pose in object_poses.items():
        rospy.loginfo(f"Moving to pose of {object_name} with z offset and new orientation:")
        pick_object(z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link)
        place_object(conveyor_pose, place_approach_offset, pick_orientation, place_approach_offset, scene, group, touch_link)
    group.set_named_target("home") 
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Successfully moved to home.")
    else:
        rospy.loginfo("Failed to plan and execute the motion.")

    moveit_commander.roscpp_shutdown()
