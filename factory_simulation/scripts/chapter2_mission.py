#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import sys
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from moveit_commander import MoveGroupCommander
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, TriggerRequest,Empty, EmptyRequest
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
    pose.position.z += z_offset
    pose.orientation.x = orientation.x
    pose.orientation.y = orientation.y
    pose.orientation.z = orientation.z
    pose.orientation.w = orientation.w
    return pose

def move_to_pose(group, pose):
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Successfully moved to the pose.")
    else:
        rospy.loginfo("Failed to plan and execute the motion.")

def approach_to_object(group, pose):
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Successfully moved to the pose.")
    else:
        rospy.loginfo("Failed to plan and execute the motion.")

def attach_object(object_name):#(scene, touch_link, object_name):
    # scene.attach_box("tcp_link", object_name)
    rospy.loginfo("Attaching object to gripper")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "L6"
    req.model_name_2 = f"{object_name}"
    req.link_name_2 = "link"
    attach_srv.call(req)

def detach_object(object_name):
    # scene.remove_attached_object(touch_link, name=object_name)
    rospy.loginfo("Detaching object from gripper")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "L6"
    req.model_name_2 = f"{object_name}"
    req.link_name_2 = "link"
    detach_srv.call(req)

def remove_object_from_scene(scene, object_name):
    scene.remove_world_object(object_name)


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
    rospy.spin()
    plan = group.go(wait=True)
    if plan:
        rospy.loginfo("Successfully moved to home.")
    else:
        rospy.loginfo("Failed to plan and execute the motion.")

    moveit_commander.roscpp_shutdown()
