#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import sys
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from moveit_commander import MoveGroupCommander
from std_srvs.srv import Empty
from gazebo_conveyor.msg import ConveyorBeltState
from box_location.srv import Boxlocation
global belt1_counter
belt1_counter = 0
global belt2_counter 
belt2_counter = 0

def conveyor_state_callback(msg):
    if msg.power == 0:
        rospy.loginfo("Going to belt1")
        pick_from_belt1(belt1_pose)

def conveyor_state_callback2(msg):
    if msg.power == 0:
        rospy.loginfo("Going to belt2")
        pick_from_belt2(belt2_pose)

def pick_from_belt1(pose):
    """
    The function `pick_from_belt1` picks an object from a conveyor belt and moves it to a specified
    pose.
    
    :param pose: The "pose" parameter represents the desired position and orientation of the end
    effector (robotic arm) in 3D space. It is used to specify where the arm should move to in order to
    pick up an object from a specific location on a conveyor belt
    """
    global belt1_counter
    belt1_counter += 1
    object_name=f"conv1_spawned_box_{belt1_counter}"
    move_to_pose(group, pose)
    attach_object(object_name)
    modified_pose = modify_pose_z_orientation(pose, approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(pose, -approach_retreat_offset, pick_orientation)
    move_to_place_pose(object_name)

def pick_from_belt2(pose):
    """
    The function `pick_from_belt2` picks an object from a conveyor belt and moves it to a specified
    pose.
    
    :param pose: The "pose" parameter is the desired pose (position and orientation) of the object that
    you want to pick from the belt
    """
    global belt2_counter
    belt2_counter += 1
    object_name=f"conv2_spawned_box_{belt2_counter}"
    move_to_pose(group, pose)
    attach_object(object_name)
    modified_pose = modify_pose_z_orientation(pose, approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(pose, -approach_retreat_offset, pick_orientation)
    move_to_place_pose(object_name)

def move_to_place_pose(object_name):
    """
    The function `move_to_place_pose` moves an object to a specified position and performs a series of
    actions.
    
    :param object_name: The parameter "object_name" represents the name of the object that you want to
    move to a specific place pose
    """
    # Hizmet isteği (request) oluşturun
    request = Empty()
    # Servise istek gönderin
    response = place_pose_service(request)
    # Hizmet başarıyla çalıştı ve bir pozisyon (pose) döndü
    place_pose = response.coordinant
    print(place_pose)
    # Hedef pozisyon (place_pose) için hareketi gerçekleştirin
    modified_pose = modify_pose_z_orientation(place_pose, z_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(modified_pose, -approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    detach_object(object_name)
    modified_pose = modify_pose_z_orientation(modified_pose, approach_retreat_offset, pick_orientation)

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
    The function `move_to_pose` sets a target pose for a robot arm group and executes the motion to move
    the arm to that pose.
    
    :param group: The "group" parameter refers to the MoveGroupCommander object that is used to control
    the robot arm. It provides functions for planning and executing motions
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

def attach_object(object_name):#(scene, touch_link, object_name):
    """
    The function `attach_object` attaches an object to a gripper in a scene.
    
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

def detach_object(object_name):
    """
    The function `detach_object` detaches an object from a gripper in a robotic scene.
    
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
    objects can be added or removed
    :param object_name: The name of the object you want to remove from the scene
    """
    scene.remove_world_object(object_name)


# The above code is a Python script that initializes a ROS node, creates service proxies for attaching
# and detaching objects, waits for a service to get the location of a box, defines poses for two
# belts, initializes a robot commander and move group commander, sets parameters for object
# manipulation, creates a planning scene interface, and then enters a loop. Within the loop, it waits
# for a message from a conveyor belt state topic, checks if the power is 0, and then performs actions
# based on the belt state. It picks an object from belt 2, moves to a home position, waits for the
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

    rospy.wait_for_service('/box_location')
    place_pose_service = rospy.ServiceProxy('/box_location', Boxlocation)
    group_name = "orion_arm"  # Grup adını MoveIt konfigürasyonuna göre değiştirin
    belt1_pose = Pose()
    belt1_pose.orientation.x = 1.0
    belt1_pose.position.x = 0.74 
    belt1_pose.position.y = 0.12
    belt1_pose.position.z = 0.26

    belt2_pose = Pose()
    belt2_pose.orientation.x = 1.0
    belt2_pose.position.x = 0.74 
    belt2_pose.position.y = -0.35
    belt2_pose.position.z = 0.26 
    
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(group_name)  # Kontrol grubu adınıza uygun olarak değiştirin
    touch_link = "tcp_link"
    object_prefix = "unit_box"
    max_objects = 8
    z_offset = 0.13  # Eklemek istediğiniz z değeri
    approach_retreat_offset= 0.15
    place_approach_offset= 0.05
    pick_orientation = geometry_msgs.msg.Quaternion(x=1, y=0, z=0, w=0)  # Yeni orientation değeri

    scene = moveit_commander.PlanningSceneInterface()
    msg = rospy.wait_for_message("/belt3/conveyor/state", ConveyorBeltState, timeout=rospy.Duration(20.0))
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/belt3/conveyor/state", ConveyorBeltState, timeout=rospy.Duration(20.0))
        while not msg.power == 0:
                # print(msg.power)
            msg = rospy.wait_for_message("/belt3/conveyor/state", ConveyorBeltState, timeout=rospy.Duration(20.0))
        
        rospy.loginfo("Going to belt2")
        pick_from_belt2(belt2_pose)
        group.set_named_target("home") 
        plan = group.go(wait=True)
    
        msg2 = rospy.wait_for_message("/belt2/conveyor/state", ConveyorBeltState, timeout=rospy.Duration(20.0))
        while not msg2.power == 0:
            msg2 = rospy.wait_for_message("/belt2/conveyor/state", ConveyorBeltState, timeout=rospy.Duration(20.0))
        
        rospy.loginfo("Going to belt1")
        pick_from_belt1(belt1_pose)
        group.set_named_target("home") 
        plan = group.go(wait=True)


    # rospy.Subscriber("/belt2/conveyor/state", ConveyorBeltState, conveyor_state_callback)
    # rospy.Subscriber("/belt3/conveyor/state", ConveyorBeltState, conveyor_state_callback2)




    # group.set_named_target("home") 
    # rospy.spin()
    # plan = group.go(wait=True)
    # if plan:
    #     rospy.loginfo("Successfully moved to home.")
    # else:
    #     rospy.loginfo("Failed to plan and execute the motion.")

    # moveit_commander.roscpp_shutdown()
