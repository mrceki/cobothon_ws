#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from gazebo_conveyor.srv import ConveyorBeltControl

belt_status = {2: False, 3: False}  # Tracks if service has been called for each belt

def laser1(data):
    if data.ranges[0] < 0.3 and not belt_status[2]:
        stopBelt(2)
        belt_status[2] = True
    elif data.ranges[0] > 0.3 and belt_status[2]:
        startBelt(2)
        belt_status[2] = False

def laser2(data):
    if data.ranges[0] < 0.3 and not belt_status[3]:
        stopBelt(3)
        belt_status[3] = True
    elif data.ranges[0] > 0.3 and belt_status[3]:
        startBelt(3)
        belt_status[3] = False

def stopBelt(data):
    try:
        controlS = rospy.ServiceProxy(f'/belt{data}/conveyor/control', ConveyorBeltControl)
        response = controlS(power=0.0)
        print(f"Service call successful for belt {data}: {response.success}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def startBelt(data):
    try:
        controlS = rospy.ServiceProxy(f'/belt{data}/conveyor/control', ConveyorBeltControl)
        response = controlS(power=50.0)
        print(f"Service call successful for belt {data}: {response.success}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
def initializeBelts():
    # Start both belts with default power value
    startBelt(2)
    startBelt(3)


def main():
    rospy.init_node('laserControlConveyor', anonymous=False)
    initializeBelts()  # Initialize the belts
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rospy.Subscriber("/laser1/scan", LaserScan, laser1)
        rospy.Subscriber("/laser2/scan", LaserScan, laser2)
        rate.sleep()



if __name__ == "__main__":
    main()
