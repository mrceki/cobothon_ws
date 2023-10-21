#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

class BoxSpawner:
    
    def __init__(self, count, interval, location):
        self.spawn_count = count
        self.spawn_interval = interval
        self.box_location_x = location[0]
        self.box_location_y = location[1]
        self.box_location_z = location[2]
        self.box_model_name = ""

class BoxSpawnerControl:
    
    def __init__(self):
        rospy.loginfo("BoxSpawner is initializing...")
        
        wait_time = 8
        rospy.loginfo("Spawner will start in %d seconds.", wait_time)
        rospy.sleep(wait_time)
        
        # creating spawners
        spawn_count = rospy.get_param('conveyor_2/spawn_count', default=9)
        spawn_interval = rospy.get_param('conveyor_2/spawn_interval', default=20)
        box_location = rospy.get_param('conveyor_2/spawn_box_location', default=[0, 0.13, 0.9])
        self.conv2_spawner = BoxSpawner(spawn_count, spawn_interval, box_location)
        self.conv2_timer = rospy.Timer(rospy.Duration(spawn_interval), self.conv2_callback)
        
        spawn_count = rospy.get_param('conveyor_3/spawn_count', default=9)
        spawn_interval = rospy.get_param('conveyor_3/spawn_interval', default=20)
        box_location = rospy.get_param('conveyor_3/spawn_box_location', default=[0, -0.36, 0.9])
        self.conv3_spawner = BoxSpawner(spawn_count, spawn_interval, box_location)
        self.conv3_timer = rospy.Timer(rospy.Duration(spawn_interval), self.conv3_callback)
        
        self.total_boxes = rospy.get_param('chapter_2/box_unit_x_columns') * rospy.get_param('chapter_2/box_unit_y_columns') * rospy.get_param('chapter_2/box_unit_height')
        
        # initialize gazebo services
        self.init_gazebo_services()
        
        # initialize spawning process
        self.init_spawning()


    def init_gazebo_services(self):
        rospy.loginfo('gazebo services initializing...')
        self.spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.spawn_model_srv.wait_for_service()
    
    def create_spawn_box_model(self, model_name, mass, size_x, size_y, size_z):
        box_model = """
        <sdf version="1.4">
            <model name="{}">
                <static>false</static>
                <mass>3</mass>
                <link name="link">
                    <inertial>
                        <mass>{}</mass>
                    </inertial>
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>{} {} {}</size>
                            </box>
                        </geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>1000</mu>
                                    <mu2>1000</mu2>
                                </ode>
                                <torsional>
                                    <coefficient>1000</coefficient>
                                </torsional>
                            </friction>
                        </surface>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>{} {} {}</size>
                            </box>
                        </geometry>
                    </visual>
                </link>
            </model>
        </sdf>
        """.format(model_name, mass, size_x, size_y, size_z, size_x, size_y, size_z)
        
        return box_model

    def init_spawning(self):
        rospy.loginfo("Spawn process initializing...")
        
        self.conv2_spawner.box_model_name = "conv2_spawned_box_"
        rospy.loginfo("%d box will be spawned with %s name in every %d seconds", 
                      self.conv2_spawner.spawn_count, self.conv2_spawner.box_model_name, self.conv2_spawner.spawn_interval)
        
        self.conv2_spawn_counter = 1
        
        self.conv3_spawner.box_model_name = "conv3_spawned_box_"
        rospy.loginfo("%d box will be spawned with %s name in every %d seconds", 
                      self.conv3_spawner.spawn_count, self.conv3_spawner.box_model_name, self.conv3_spawner.spawn_interval)
        
        self.conv3_spawn_counter = 1
        
        rospy.loginfo("Total %d boxes needed for palletizing geometry.", self.total_boxes)
        
        self.is_conv2_completed = False
        self.is_conv3_completed = False
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():
            
            if (self.is_conv2_completed == True) and (self.is_conv3_completed == True):
                rospy.loginfo("All spawner processes completed with maximum spawn count.")
                rospy.signal_shutdown("Done.")
            
            c2_count = self.conv2_spawn_counter - 1
            c3_count = self.conv3_spawn_counter - 1
            if self.total_boxes == (c2_count + c3_count):
                rospy.loginfo("Reached %d boxes count needed for palletizing geometry.", self.total_boxes)
                rospy.signal_shutdown("Done.")
            
            rate.sleep()
            
    def conv2_callback(self, event=None):
        
        box_pose = Pose()
        box_pose.position.x = self.conv2_spawner.box_location_x
        box_pose.position.y = self.conv2_spawner.box_location_y
        box_pose.position.z = self.conv2_spawner.box_location_z
        
        model_name = self.conv2_spawner.box_model_name + str(self.conv2_spawn_counter)
            
        mass = 3
        size_x = 0.15
        size_y = 0.15
        size_z = 0.15
        box_model = self.create_spawn_box_model(model_name, mass, size_x, size_y, size_z)
        
        try:
            spawn_resp = self.spawn_model_srv(model_name, box_model, "", box_pose, "world")
            
            if spawn_resp.success == True:
                rospy.loginfo("%s gazebo box model spawned.", model_name)
            else:
                rospy.logwarn("%s gazebo box model could not be spawned.", model_name)
        except:
            rospy.logerr("Gazebo spawn_sdf_model service failed.")
            
        if self.conv2_spawn_counter == self.conv2_spawner.spawn_count:
            rospy.loginfo("Conveyor2 spawner reached %d count.", self.conv2_spawner.spawn_count)
            self.is_conv2_completed = True
            self.conv2_timer.shutdown()
        
        self.conv2_spawn_counter += 1
    
    def conv3_callback(self, event=None):

        box_pose = Pose()
        box_pose.position.x = self.conv3_spawner.box_location_x
        box_pose.position.y = self.conv3_spawner.box_location_y
        box_pose.position.z = self.conv3_spawner.box_location_z
        
        model_name = self.conv3_spawner.box_model_name + str(self.conv3_spawn_counter)
            
        mass = 3
        size_x = 0.15
        size_y = 0.15
        size_z = 0.15
        box_model = self.create_spawn_box_model(model_name, mass, size_x, size_y, size_z)
        
        try:
            spawn_resp = self.spawn_model_srv(model_name, box_model, "", box_pose, "world")
            
            if spawn_resp.success == True:
                rospy.loginfo("%s gazebo box model spawned.", model_name)
            else:
                rospy.logwarn("%s gazebo box model could not be spawned.", model_name)
        except:
            rospy.logerr("Gazebo spawn_sdf_model service failed.")
            
        if self.conv3_spawn_counter == self.conv3_spawner.spawn_count:
            rospy.loginfo("Conveyor3 spawner reached %d count.", self.conv3_spawner.spawn_count)
            self.is_conv3_completed = True
            self.conv3_timer.shutdown()
        
        self.conv3_spawn_counter += 1
