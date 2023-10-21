from yaml_parse import parser_ch1
import geometry_msgs.msg

def ch1_box_spawner(scene):

    config = parser_ch1('../config/box_models_links.yaml')

    for box in config:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = box[0] + '_frame_id'
        box_pose.pose.position.x = box[2]
        box_pose.pose.position.y = box[3]
        box_pose.pose.position.z = box[4]
        box_name = box[0]
        box_size = (0.15, 0.15, 0.15)
        scene.add_box(box_name, box_pose, size=box_size)
