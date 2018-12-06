#!/usr/bin/env python
import rospy
import matplotlib as mp
import matplotlib.pyplot as plt
from MotionPlanner import *
from get_model import *

rate = 20
occupancy_scale = 0.01

mp = MotionPlanner()
ob = ObjectPositionCollector()

def init():
    print("Init")
    mp.setWorldGetter(ob)
    
    
# Sends quadcopter velocity commands via a ROS publisher
def sendCommands(flightPath):
    # Use flightPath[0] as your next point
    if(len(flightPath) > 1):
        x = flightPath[1][0]*occupancy_scale
        y = flightPath[1][1]*occupancy_scale
        #x = flightPath[1][0]*occupancy_scale
        print(flightPath[1],x,y)
    pass
    #velocity_publisher.publish(vel_msg)


# ROS Subscriber for quadcopter position and orientation
# Updates global variables related to quadcopter pose
def updateData():
    #currentPose
    # Update the World Data
    ob.update_world_data()
    #Get quadcopter data
    quadcopter = ob.get_quadcopter()
    qPosition = quadcopter.get_pos(occupancy_scale)
    qOrientation = quadcopter.get_twist()
    # Get Target Data
    target = ob.get_target()
    tPosition = target.get_pos(occupancy_scale)
    tOrientation = target.get_twist()
    
    #qPosition[0] = tPosition[0] - 3/occupancy_scale #(tPosition[0] - qPosition[0])
    #qPosition[1] = tPosition[1] - 3/occupancy_scale #(tPosition[1] - qPosition[1])
    #qPosition[2] = tPosition[2] + 0/occupancy_scale #(tPosition[2] - qPosition[2])
    # TODO FIgure this out
    ePosition = [0,0,0]
    ePosition[0] = tPosition[0] + 3/occupancy_scale #(tPosition[0] - qPosition[0])
    ePosition[1] = tPosition[1] #+ 3/occupancy_scale #(tPosition[1] - qPosition[1])
    ePosition[2] = tPosition[2] + 0 #(tPosition[2] - qPosition[2]) 
    #print('ePosition = ',ePosition)
    mp.set_start(qPosition)
    mp.set_end(ePosition)
    mp.set_currentPose(qOrientation)
    mp.set_target(tPosition)
    pass


# Main loop cycle
def run(rate):
    print("Top of a loop")
    #self.timeReset = True
    start = time.time()
    t1 = time.time()
    updateData()
    t2 = time.time()
    print("Update Time = "+ str(t2-t1))
    t1 = time.time()
    mp.motionControl(False)
    t2 = time.time()
    print("Motion Time = "+ str(t2-t1))
    t1 = time.time()
    sendCommands(mp.finalPath)
    t2 = time.time()
    print("Controls Time = "+ str(t2-t1))
    end = time.time()
    print("Total loop time = "+ str(end-start)+"\n")

if __name__ == '__main__':
    try:
        print("Starting Program")
        rospy.init_node('Motion_Planner')
        # init the program
        init()
        # Set the run rate
        update_timer = rospy.Timer(rospy.Duration(rate), run)
        # Let ROS spin
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

