import sys
import time
import math
#import rospy
import numpy as np
import random as rand
import matplotlib.cm as cm
import matplotlib.pyplot as plt
#from std_msgs.msg import String
#from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped

class MotionPlanner():
    
    def __init__(self):
        # Global variables
        self.timeReset = False
        self.end = [40, 45, 1]                                    # x, y, z
        self.start = [90, 60, 1]                                      # x, y, z
        self.target = [55, 55, 1]                                   # x, y, z
        self.currentPose = [0, 0, 0]                                # x, y, z
        self.mapSize = [100, 100]                                   # Size of the world
        self.world = np.zeros((self.mapSize[0],self.mapSize[0]))    # Create an empty world 
        self.startTime = 0.0                                        # Start time of motion
        self.cycleTime = 0.1                                        # Allows the main loop to run once every X seconds
        self.motionTime = 10.0                                      # Total time to complete motion in seconds
        self.angularChange = 0.0                                    # Angle between start and end with centerpoint at target
        self.clockwise = True
        self.printouts = True
        
        #rospy.init_node('quad_motion', anonymous=True)
        #velocity_publisher = rospy.Publisher('/command/twist', Twist, queue_size=10)
        #telemetry_listener = rospy.Subscriber("????????", String, updateData)
        #vel_msg = Twist()
    
    
    '''
    ####################################################################    
    ######################## Primary Functions #########################
    ####################################################################
    '''
    
    # General setup to occur once
    def setup(self):    
        #vel_msg.linear.x = 0
        #vel_msg.linear.y = 0
        #vel_msg.linear.z = 0
        #vel_msg.angular.x = 0
        #vel_msg.angular.y = 0
        #vel_msg.angular.z = 0
        
        #velocity_publisher.publish(vel_msg)
        #rospy.Timer(rospy.Duration(cycleTime), timerEvent)
        #angularChange = planarAngle()    
        self.startTime = rospy.get_time()
        
        
    # Performs the motion control calculations    
    def motionControl(self):
        arc = self.createPlanarArc()
        #self.elapsedTime = rospy.get_time() - startTime
        self.plotMotion(arc)
        #vel_msg.angular.z = angularChange / motionTime  #Rotation velocity is constant for now
    
    
    # Creates a 2-D top-down view of the motion path
    def plotMotion(self, path):
        plt.clf()
        plt.imshow(self.world, cmap=cm.magma)
        plt.gca().invert_yaxis()
        plt.plot(self.start[0], self.start[1], 'yo')
        plt.plot(self.target[0], self.target[1], 'go')
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        for a in path:
            plt.plot(a[0], a[1], 'r.')
            plt.pause(0.01 * self.cycleTime)        
        plt.plot(self.end[0], self.end[1], 'yo')
        plt.show()
        #plt.plot()
        #plt.imshow()
    
    
    def createPlanarArc(self):
        path = [] #[self.start[0], self.start[1], 0.0]
        radStart = self.planarDist(self.start, self.target)     #Radius about the target at the start of the arc
        radEnd = self.planarDist(self.end, self.target)         #Radius about the target at the end of the arc
        radDelt = radEnd - radStart                             #Difference between radii
        print("Radius: " + str(radStart) + ", " + str(radEnd)) if self.printouts else None
        
        radDeltPerSec = radDelt / self.motionTime
        thetaStart = self.worldPlanarAngle(self.start, self.target)
        thetaEnd = self.worldPlanarAngle(self.end, self.target)
        
        if self.clockwise:
            #neg rotation
            if thetaStart > thetaEnd:                
                thetaDelt = -1 * (thetaStart - thetaEnd)                  
            else:
                thetaDelt = -1 * (((2 * math.pi) - thetaEnd) + thetaStart)
        else:
            #pos rotation
            if thetaStart > thetaEnd: 
                thetaDelt  = (2 * math.pi) - thetaStart + thetaEnd
            else:
                thetaDelt  = thetaEnd - thetaStart 

        thetaDeltPerSec = thetaDelt / self.motionTime
        print("Theta: " + str(thetaStart) + ", " + str(thetaEnd) + ", "  + str(thetaDelt)) if self.printouts else None
        
        for a in range(0, int(self.motionTime / self.cycleTime)):
            t = a * self.cycleTime
            tempRad = (radDeltPerSec * t) + radStart
            tempTheta = (thetaDeltPerSec * t) + thetaStart
            
            while tempTheta > math.pi:
                tempTheta -= (2 * math.pi)
            while tempTheta <= -math.pi:
                tempTheta += (2 * math.pi)
            tempTheta = round(tempTheta, 3)
                
            x = round(((math.cos (tempTheta) * tempRad) + self.target[0]), 3)
            y = round(((math.sin(tempTheta) * tempRad) + self.target[1]), 3)
            path.append([x, y, t])
            if a%10 == 0:
                print("[X, Y, theta, T]: " + str(x) + ", " + str(y) + ", " + str(tempTheta) + ", " + str(t))  if self.printouts else None
                
        path.append([self.end[0], self.end[1], self.motionTime])    
        return path
        
        
    # Sends quadcopter velocity commands via a ROS publisher
    def sendCommands(self):
        pass
        #velocity_publisher.publish(vel_msg)
    
    
    # ROS Subscriber for quadcopter position and orientation
    # Updates global variables related to quadcopter pose
    def updateData(self):
        #currentPose
        pass
    
    
    # Forces a tick rate for the motion planner
    # Main loop cycle time is controlled by changing timeReset
    def timerEvent(self, event):
        self.timeReset = True    
        
        
    '''
    ####################################################################    
    ######################### Helper Functions #########################
    ####################################################################
    '''
        
    # Calculates the angle between two points about a  third target point projected into the x-y plane
    # Inputs:
    #   a: [x, y(, z)] position for first point
    #   b: [x, y(, z)] position of second point
    #   c: [x, y(, z)] position of target or center of rotation
    # Return:
    #   Angle between a and b in x-y plane
    def planarAngle(self, a, b, c):
        aToC = self.planarDist(a, c)    # First to Target distance
        bToC = self.planarDist(b, c)    # Second to Target distance
        aToB = self.planarDist(a, b)    # First to Second distance
        
        # Law of cosines: cos(A) = (b^2 + c^2 - a^2)/(2*b*c)
        return arccos(((aToC*aToC) + (bToC*bToC) - (aToB*aToB))/ (2*aToC*bToC)) #NEED TO ACCOUNT FOR ROLLOVER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    
    # Calculates the angle between two points relative to the X-axis projected into the x-y plane
    # Inputs:
    #   a: [x, y(, z)] position for first point
    #   b: [x, y(, z)] position of target or center of rotation
    # Return:
    #   Angle between a and x-axis about b in x-y plane
    def worldPlanarAngle(self, a, b):
        delta = [(a[0] - b[0]), (a[1] - b[1])]
        angle = math.atan2(delta[1], delta[0])
        if angle < 0.0:
            angle += (2 * math.pi)
        return angle
    
    
        
    # Calculates the distance between two points projected onto the x-y plane
    # Inputs:
    #   a: [x, y(, z)] position of first point
    #   b: [x, y(, z)] position of second point
    # Return:
    #   Linear distance between a and b in x-y plane
    def planarDist(self, a, b):
        xSq = (b[0] - a[0])**2
        ySq = (b[1] - a[1])**2
        return math.sqrt((xSq + ySq))
    
        
    # Calculates the distance between two points in 3D space
    # Inputs:
    #   a: [x, y, z] position of first point
    #   b: [x, y, z] position of second point
    # Return:
    #   Linear distance between a and b
    def trueDist(self, a, b):
        xSq = (b[0] - a[0])**2
        ySq = (b[1] - a[1])**2
        zSq = (b[2] - a[2])**2
        return math.sqrt((xSq + ySq + zSq))
    
    
    # Linearly interpolates between the start and end positions
    # Inputs:
    #   a: [x, y, z] start position
    #   b: [x, y, z] end position
    #   t: Desired return time
    #   tF: Total duration of interpolation
    # Return:
    #   [x, y, z] position of quadcopter at time t    
    def linInter(self, a, b, t, tF):
        delta = b - a
        returnPos = ((delta / tF) * t) + a
        pass
    
        
    # Circularly interpolates between the start and end positions
    # Inputs:
    #   a: [x, y, z] start position
    #   b: [x, y, z] end position
    #   t: Desired return time
    #   tF: Total duration of interpolation
    # Return:
    #   [x, y, z] position of quadcopter at time t    
    def cirInter(self, a, b, t, tF):
        pass


'''
########################
###### Node Class ######
########################
'''    

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.z = 1.0
        self.cost = 0.0
        self.parent = None        
    
'''    
####################################################################  
########################### Main Function ##########################
####################################################################
'''
    
if __name__ == '__main__':
    mp = MotionPlanner()
    #mp.setup()
    mp.motionControl()
    #try:
    #    setup()
    #    #updateData()
    #    createPath()
    #    
    #    while True:        
    #        if timeReset:
    #            #updateData()
    #            motionControl()
    #            timeReset = False
    #    
    #except rospy.ROSInterruptException: pass
