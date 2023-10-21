import rospy
import moveit_commander
import geometry_msgs.msg
import sys

def get_object_poses(scene, object_prefix, max_objects):
    object_poses = {}
    for i in range(1, max_objects + 1):
        object_name = f"{object_prefix}{i}"
        if scene.get_known_object_names().count(object_name) > 0:
            object_pose = scene.get_object_poses([object_name])
            object_poses[object_name] = object_pose[object_name]
    return object_poses

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

def attach_object(scene, touch_link, object_name):
    scene.attach_box("tcp_link", object_name)

def detacht_object(scene, touch_link, object_name):
    scene.remove_attached_object(touch_link, name=object_name)

def remove_object_from_scene(scene, object_name):
    scene.remove_world_object(object_name)

def pick_object(z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link):
    modified_pose = modify_pose_z_orientation(pose, z_offset, pick_orientation)
    rospy.loginfo(modified_pose)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(pose, -approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    attach_object(scene, touch_link, object_name)
    modified_pose = modify_pose_z_orientation(pose, approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)

def place_object(conveyor_pose, z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link):
    modified_pose = modify_pose_z_orientation(conveyor_pose, z_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    modified_pose = modify_pose_z_orientation(conveyor_pose, -approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    detacht_object(scene, touch_link, object_name)
    modified_pose = modify_pose_z_orientation(conveyor_pose, approach_retreat_offset, pick_orientation)
    move_to_pose(group, modified_pose)
    remove_object_from_scene(scene, object_name)

if __name__ == '__main__':
    rospy.init_node('move_to_object_poses', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("orion_arm")  # Kontrol grubu adınıza uygun olarak değiştirin
    touch_link = "tcp_link"
    object_prefix = "Box_"
    max_objects = 8
    z_offset = 0.15  # Eklemek istediğiniz z değeri
    approach_retreat_offset= 0.05
    pick_orientation = geometry_msgs.msg.Quaternion(x=1, y=0, z=0, w=0)  # Yeni orientation değeri

    conveyor_pose = geometry_msgs.msg.Pose()
    conveyor_pose.orientation.x = 1.0
    conveyor_pose.position.x = -0.5  
    conveyor_pose.position.y = 0
    conveyor_pose.position.z = 0

    scene = moveit_commander.PlanningSceneInterface()
    object_poses = get_object_poses(scene, object_prefix, max_objects)
    
    for object_name, pose in object_poses.items():
        rospy.loginfo(f"Moving to pose of {object_name} with z offset and new orientation:")
        pick_object(z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link)
        place_object(conveyor_pose, z_offset, pick_orientation, approach_retreat_offset, scene, group, touch_link)

    moveit_commander.roscpp_shutdown()