#!/usr/bin/env python3

import rospy
from box_location.srv import Boxlocation  # Servis türünü ve ismini uygun şekilde güncelleyin
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

def call_service():
    # ROS düğümünü başlat
    rospy.init_node('service_client_node')

    # Servis çağırılacak tür ve ismi
    service_name = 'box_location'  # Servis adını uygun şekilde güncelleyin
    service = rospy.ServiceProxy(service_name, Boxlocation)  # Servis türünü uygun şekilde güncelleyin

    # Servis çağırma isteği oluştur
    request = Empty()

    # Servisi çağır ve yanıtı al
    response = service(request)

    # Servisten gelen veriyi yazdır
    print("Received Coordinate:")
    print("Position: x={}, y={}, z={}".format(response.coordinant.position.x,
                                               response.coordinant.position.y,
                                               response.coordinant.position.z))
    print("Orientation: x={}, y={}, z={}, w={}".format(response.coordinant.orientation.x,
                                                       response.coordinant.orientation.y,
                                                       response.coordinant.orientation.z,
                                                       response.coordinant.orientation.w))

if __name__ == '__main__':
    try:
        call_service()
    except rospy.ServiceException as e:
        print("Service call failed:", e)
