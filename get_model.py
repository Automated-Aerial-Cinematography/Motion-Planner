#!/usr/bin/env python
import rospy
from point_in_polygon import *
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties, GetModelState
import matplotlib as mp
import matplotlib.pyplot as plt
from math import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros.gazebo_interface import spawn_sdf_model_client
#import numpy as np

# Pulled from the test_simple_world.world
# If not in this list, assuming it is 1, 1, 1
# This data is the actual length, width, height
# The position data below is the mid-points.
known_object_scales = {'unit_box_0':(1, 1, 1),
                       'unit_box_1':(2.60838, 2.61964, 2.57757)}

ignore_objects = ("ground_plane")
quadcopter_name = "quadrotor"
target_name = "person_standing"
class gazebo_object:
    def __init__(self, name):
        self.name = name
        self.pos = (0,0,0)
        self.size = (1,1,1)
        self.twist = (0,0,0,1)
        
    def set_pos(self, x, y, z):
        self.pos = (x,y,z)
    def set_size(self, x, y, z):
        self.size = (x,y,z)
    def set_twist(self, _x, _y, _z, _w):
        (x,y,z) = euler_from_quaternion([_x,_y,_z,_w])
        self.twist = (x,y,z)

    def get_pos(self, scale=1):
        pos = [0,0,0]
        pos[0] = self.pos[0]/scale
        pos[1] = self.pos[1]/scale
        pos[2] = self.pos[2]/scale
        return pos
    def get_size(self):
        return self.size
    def get_twist(self):
        twist = [self.twist[0],self.twist[1], self.twist[2]]
        return twist
class ObjectPositionCollector(object):
  
    def __init__(self, rate=0):
        print("Creating object Position Collector")
        if(rate != 0):
            rospy.init_node('WorldDataCollector')

        self.quadcopter = gazebo_object(quadcopter_name)
        self.target = gazebo_object(target_name)
        self.get_world_properties = None
        self.get_model_properties = None
        self.get_model_state = None
        self.objects = {}
        if(rate != 0):
            update_timer = rospy.Timer(rospy.Duration(rate), self.update_world_data)


    def get_proxy_handles(self):
        if self.get_world_properties is None:
            try:
                # Handle for world properties update function
                rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
                self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties',
                                                     GetWorldProperties)
            except rospy.ROSException:
                print('/gazebo/get_world_properties service is unavailable')
        if self.get_model_properties is None:
            try:
                # Handle for retrieving model properties
                rospy.wait_for_service('/gazebo/get_model_properties')
                self.get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties',
                                                     GetModelProperties)
            except rospy.ROSException:
                print('/gazebo/get_model_properties service is unavailable')
        if self.get_model_state is None:
            try:
               # Handle for retrieving model properties
                rospy.wait_for_service('/gazebo/get_joint_properties')
                self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                                     GetModelState)
            except rospy.ROSException:
                print('/gazebo/get_model_properties service is unavailable')

    def update_world_data(self, event=None):
        # TODO this will need a mutex if we use the rospy timer internally.
       #print("Updating World Data")
        # Get the handles to the Service Proxy
        self.get_proxy_handles()
        # Get the Position Data
        try:
            msg_world = self.get_world_properties()
            #print(msg_world)
            if(msg_world.success):
                for model in msg_world.model_names:
                    if(model not in ignore_objects):
                        model_properties = self.get_model_properties(model)
                        # print(model_properties)
                        if(model_properties.success):
                            #for link in model_properties.body_names:
                            # For the time being, I think the first link is the root, unless something indicates otherwise.
                            link = model_properties.body_names[0]
                            model_state = self.get_model_state(model, link)
                            #print(model_state)
                            if(model_state.success):
                               # print "Model Name", model, link, msg_world.sim_time
                               # print "\tX", model_state.pose.position.x
                               # print "\tY", model_state.pose.position.y
                               # print "\tZ", model_state.pose.position.z
                                if(model == quadcopter_name):
                                    self.quadcopter.set_pos(model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z)
                                    self.quadcopter.set_twist(model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w)
                                elif(model == target_name):
                                    self.target.set_pos(model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z)
                                    self.target.set_twist(model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w)
                                else:
                                    if(model not in self.objects):
                                        o = gazebo_object(model)
                                        if(model in known_object_scales):
                                            s = known_object_scales[model]
                                            o.set_size(s[0], s[1], s[2])
                                        self.objects[model] = o
                                    self.objects[model].set_pos(model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z)
                                    self.objects[model].set_twist(model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w)
            #self.draw_objects()
        except rospy.ServiceException, e:
            print("Service Failed", e)

    def draw_objects(self, pause=False):
       # print("Stuff")
        plt.axis([-20,20,-10,20])
        plt.grid(True)
        for o in self.objects:
            obj = self.objects[o]
            pos = obj.get_pos()
            scale = obj.get_size()
           # print(o, pos, scale)
            x = pos[0] - scale[0]/2.0
            y = pos[1] - scale[1]/2.0
          #  print(x,y, (scale[0]/2.0), (scale[1]/2.0))
            twist = obj.get_twist()
            r = plt.Rectangle((x, y),(scale[0]), (scale[1]), twist[2] )
            plt.gca().add_patch(r)
       # for x in range (-5, 5):
        #    for y in range(-5, 5):
         #       print("Checking x,y", x, y, self.get_occupancy_2d(x, y, 0.5))
        if(pause):        
            plt.pause(1)
    def get_occupancy_2d(self, _x, _y, radius):
        for o in self.objects:
            obj = self.objects[o]
            pos = obj.get_pos()
            scale = obj.get_size()
            twist = obj.get_twist() 
           # print(o, pos, scale)
            x = (scale[0]/2.0)+radius
            y = (scale[1]/2.0)+radius
            # TODO, this is just doing a simple check on the rotated square that is the object
            # It just checks that the requested point is within the box or not.
            # If there are an equal number of lines that it is above and below it is within the box
            # Otherwise it is not within the box
            # TODO thoroughly test this code
            x1 = pos[0] - (x*cos(twist[2]) -  y*sin(twist[2]))
            y1 = pos[1] - (x*sin(twist[2]) +  y*cos(twist[2]))
            x2 = pos[0] - (x*cos(twist[2]) -  y*sin(twist[2]))
            y2 = pos[1] + (x*sin(twist[2]) +  y*cos(twist[2]))
            x3 = pos[0] + (x*cos(twist[2]) -  y*sin(twist[2]))
            y3 = pos[1] + (x*sin(twist[2]) +  y*cos(twist[2]))
            x4 = pos[0] + (x*cos(twist[2]) -  y*sin(twist[2]))
            y4 = pos[1] - (x*sin(twist[2]) +  y*cos(twist[2]))
            polygon = [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
            #print(_x,_y, radius, polygon)
            check = isInside(polygon, 4, (_x,_y))
            if(check):
                #print("Occupied", _x,_y)
                return True
       # print("Free", _x,_y)
        return False
    def get_quadcopter(self):
        return self.quadcopter
    def get_target(self):
        return self.target
if __name__ == '__main__':
    try:
        print("Startin Program")
        m = ObjectPositionCollector(1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

