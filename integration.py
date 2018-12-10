#!/usr/bin/env python
import rospy
import matplotlib as mp
import matplotlib.pyplot as plt
from MotionPlanner import *
from get_model import *
from move_drone import *
from pylab import savefig
from twisted.trial._dist.workercommands import Start
from numpy.distutils import line_endings


rate = 0.05
occupancy_scale = 0.01

mp = MotionPlanner(clockwise=False)
ob = ObjectPositionCollector()
points = []
mv_dr = None
def init():
    global points
    global mv_dr

    print("Init")
    mp.setWorldGetter(ob)
     #self.timeReset = True
    t1 = time.time()
    updateData()

    #Get quadcopter data
    quadcopter = ob.get_quadcopter()
    qPosition = quadcopter.get_pos(occupancy_scale)
    qOrientation = quadcopter.get_twist()
    # Get Target Data
    target = ob.get_target()
    tPosition = target.get_pos(occupancy_scale)
    tOrientation = target.get_twist()

    (l1, l2, a1, a2) = generate_params(qPosition, tPosition, 3.5/occupancy_scale, 270)

    line_params = Disc(l1, l2)
    mp.set_start(a1)
    mp.set_end(a2)
    mp.set_currentPose(qOrientation)
    mp.set_target(tPosition)
    
    print("Start line ",l1)
    print("End line ",l2)
    print("Start Arc", a1)
    print("End Arc ",a2)
    print("Quad", qPosition)
    print("Target",tPosition)

    t2 = time.time()
    print("Update Time = "+ str(t2-t1))
    t1 = time.time()
    plt.clf()
    line = mp.RRTstar(line_params, 0)
    for pt in reversed(line): 
        points.append(pt) 
    mp.motionControl(False)
    points += mp.finalPath
    
    ob.draw_objects()
    for a in range (0, (len(points)-1)):
        #print([points[a][0]*occupancy_scale, points[a+1][1]*occupancy_scale], [points[a][0]*occupancy_scale, points[a+1][1]*occupancy_scale])
        plt.plot([points[a][0]*occupancy_scale, points[a+1][0]*occupancy_scale], [points[a][1]*occupancy_scale, points[a+1][1]*occupancy_scale], "r")

    savefig("integration.png")
    
    for i in range(len(points)):
        points[i][0] = points[i][0]*occupancy_scale
        points[i][1] = points[i][1]*occupancy_scale
        tPosition = target.get_pos(occupancy_scale)
    tPosition = target.get_pos(1)
    mv_dr = Move_Drone(points,tPosition)
    t2 = time.time()
    print("Motion Planning Time = "+ str(t2-t1))

def generate_params(start_position, target_position, radius, full_angle):
    # Define the Start point of the RRT to get to Arc
    line_start = start_position
    # Determine the End point of the RRT to get to the arc (also the start of the arc)
    x1 = start_position[0]
    y1 = start_position[1]
    x2 = target_position[0]
    y2 = target_position[1]
    d = math.sqrt(math.pow(y2-y1, 2) + math.pow(x2-x1, 2))
    adj = (x2-x1)
    hyp = d
    xe = 0
    ye = 0
    theta = 0
    if (hyp != 0):
        theta = math.acos((x2-x1)/hyp)
        ye = (d-radius) * math.sin(theta) + y1
        xe = (d-radius) * math.cos(theta) + x1
    line_end = [xe, ye, start_position[2]]
    arc_start = line_end
    #print(arc_start)
    #print(theta, full_angle)
    theta = theta + math.radians(full_angle)
    #print("New Theta", theta)
    #print(radius, x2, y2)
   # print(math.sin(theta), math.cos(theta))
    ye = y2 - (radius) * math.sin(theta)
    xe = x2 - (radius) * math.cos(theta)
    #print(xe, ye)
    arc_end = [xe, ye, start_position[2]] 
    
    return (line_start, line_end, arc_start, arc_end)
    
    
# Sends quadcopter velocity commands via a ROS publisher
def sendCommands(quad_position=None, quad_orientation=None):
    #print("Starting to send commands")
    global mv_dr
    # new_path = []
    # Use flightPath[0] as your next point

        # if i%2 == 0:
        #     new_path.append(flightPath[i])
    #print (flightPath)
    if(mv_dr != None):
        mv_dr.main(quad_position, quad_orientation)
    else:
        print("Move Drone is None")
        # for j in range
    # if(len(flightPath) > 1):
    #     x = flightPath[1][0]*occupancy_scale
    #     y = flightPath[1][1]*occupancy_scale
    #     #x = flightPath[1][0]*occupancy_scale
    #     print(flightPath[1],x,y)
    # pass
    #velocity_publisher.publish(vel_msg)


# ROS Subscriber for quadcopter position and orientation
# Updates global variables related to quadcopter pose
def updateData():
    #currentPose
    # Update the World Data
    ob.update_world_data()
    pass


# Main loop cycle
def run(rate):
    #print("Top of a loop")
    #self.timeReset = True
    #start = time.time()
    t1 = time.time()
    #updateData()
    #quadcopter = ob.get_quadcopter(True)
    #qPosition = quadcopter.get_pos(1)
    #qOrientation = quadcopter.get_twist()
    t2 = time.time()
    print("Update Time = "+ str(t2-t1))
    t1 = time.time()
    #sendCommands(qPosition, qOrientation)
    sendCommands()
    t2 = time.time()
    print("Controls Time = "+ str(t2-t1))
    #end = time.time()
    #print("Total loop time = "+ str(end-start)+"\n")

if __name__ == '__main__':
    try:
        print("Starting Program")
        rospy.init_node('Motion_Planner')
        # init the program
        init()
        # Set the run rate
        run(rate)
        #update_timer = rospy.Timer(rospy.Duration(rate), run)
        # Let ROS spin
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

