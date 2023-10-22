#!/usr/bin/env python3

import rospy
from box_location.srv import Boxlocation
from geometry_msgs.msg import Pose
import yaml
import numpy as np

# YAML dosyasını açma
with open('/home/cenk/cobot_ws/src/cobothon_ws/box_location/config/box_patern.yaml', 'r') as file:
    try:
        data = yaml.safe_load(file)
        structural_matrix = data['chapter_2']['structural_matrix']
        
    except yaml.YAMLError as exc:
        print(exc)
sample_2d_array = structural_matrix

referance  = [-0.8, -0.25, -0.3] 
coordinates_arr = []
coordinates = Pose()

data = [[0, 3, 0], [1, 2, 1], [0, 1, 0]]
for t in range(len(data)):
    for i in range(len(data)):
        for j in range(len(data[i])):
            if data[i][j] != 0:
                coordinates = Pose()
                coordinates.position.x = referance[0]+i*0.16
                coordinates.position.y = referance[1]+j*0.16
                coordinates.position.z = referance[2]+t*0.15
                coordinates.orientation.x = 1
                coordinates_arr.append(coordinates)
                data[i][j] -= 1  

        
i =0


def handle_request(req):

    global i
    i +=1

    print(coordinates_arr[i-1])
    return coordinates_arr[i-1]
    
    

def my_service_server():
    rospy.init_node('box_location_server')
    rospy.Service('box_location', Boxlocation, handle_request)
    rospy.loginfo("box_location is ready.")
    rospy.spin()

if __name__ == '__main__':
    my_service_server()

