#!/usr/bin/env python3

import rospy
from box_location.srv import Boxlocation
from geometry_msgs.msg import Pose
import yaml
import numpy as np

# YAML dosyasını açma
with open('src/box_location/config/box_patern.yaml', 'r') as file:
    try:
        data = yaml.safe_load(file)
        structural_matrix = data['chapter_2']['structural_matrix']
        
    except yaml.YAMLError as exc:
        print(exc)
sample_2d_array = structural_matrix

referance  = [-1.36, 0.5, 0.218] 
coordinant_arr = []
coordinant = Pose()

data = [[0, 3, 0], [1, 2, 1], [0, 1, 0]]
for t in range(len(data)):
    for i in range(len(data)):
        for j in range(len(data[i])):
            if data[i][j] != 0:
                coordinant = Pose()
                coordinant.position.x = referance[0]+i*0.16
                coordinant.position.y = referance[1]+j*0.16
                coordinant.position.z = referance[2]+t*0.15
                coordinant.orientation.x = 1
                coordinant_arr.append(coordinant)
                data[i][j] -= 1  

        
i =0


def handle_request(req):

    global i
    i +=1

    print(coordinant_arr[i-1])
    return coordinant_arr[i-1]
    
    

def my_service_server():
    rospy.init_node('box_location_server')
    rospy.Service('box_location', Boxlocation, handle_request)
    rospy.loginfo("box_location is ready.")
    rospy.spin()

if __name__ == '__main__':
    my_service_server()

