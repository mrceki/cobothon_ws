#!/usr/bin/env python3
from yaml_parse import parser_ch1
import geometry_msgs.msg
import moveit_commander


def ch1_box_spawner(scene):

    config = parser_ch1('/home/cenk/cobot_ws/src/cobothon_ws/factory_simulation/config/box_models_links.yaml')
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    for box in config:

        print(box)

        box_pose.pose.position.x = box[2]+1.5
        box_pose.pose.position.y = box[3]
        box_pose.pose.position.z = box[4]-0.5
        box_name = box[0]
        box_size = (0.15, 0.15, 0.15)
        scene.add_box(box_name, box_pose, size=box_size)


moveit_commander.roscpp_initialize([])
scene = moveit_commander.PlanningSceneInterface()

ch1_box_spawner(scene)