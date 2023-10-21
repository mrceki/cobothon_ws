#!/usr/bin/env python3

import rospy
from BoxSpawnerControl import BoxSpawnerControl

if __name__ == '__main__':
    
    try:
        rospy.init_node('box_spawner_node')
    
        box_spawner = BoxSpawnerControl()
        box_spawner.init_spawning()
    
    except rospy.ROSInterruptException:
        print("ROS stopped.")