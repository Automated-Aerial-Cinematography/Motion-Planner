import sys
import time
import math
import copy
#import rospy
import heapq
import heapq_max
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
        self.end = [180, 600, 1]                                    # x, y, z
        self.start = [780, 740, 1]                                      # x, y, z
        self.target = [430, 610, 1]                                   # x, y, z
        self.currentPose = [0, 0, 0]                                # x, y, z
        self.mapSize = [1000, 1000]                                   # Size of the world
        self.world = np.zeros((self.mapSize[0],self.mapSize[0]))    # Create an empty world 
        self.startTime = 0.0                                        # Start time of motion
        self.cycleTime = 0.1                                        # Allows the main loop to run once every X seconds
        self.motionTime = 10.0                                      # Total time to complete motion in seconds
        self.angularChange = 0.0                                    # Angle between start and end with centerpoint at target
        self.clockwise = True
        self.printouts = False
        self.arc = []
        
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
        #self.startTime = rospy.get_time()
        #self.addSquare([770,325], 50)
        self.addCircle([350,350], 60)
        
        
    def addSquare(self, pos, size):
        halfSize = int(size/2)
        for x in range((-halfSize), halfSize):
            for y in range((-halfSize), halfSize):
                self.world[(pos[0]+x),(pos[1]+y)] = 1
                
    def addCircle(self, pos, rad):
        for x in range((-rad), rad):
            for y in range((-rad), rad):
                point  = [(pos[0] + x), (pos[1] + y)]
                if self.planarDist(pos, point) <= rad:
                    self.world[(pos[0]+x),(pos[1]+y)] = 1
        
        
    # Performs the motion control calculations    
    def motionControl(self):
        self.arc = self.createPlanarArc()
        disc = self.processArc(self.arc)
        failures = 0
        #self.elapsedTime = rospy.get_time() - startTime
        
        solutions = []
        print("Number of discontinuities: " + str(len(disc)))
        for item in disc:
            #print("Start: " + str(a.start))
            #print("End: " + str(a.end))
            while failures < 5:
                sol = self.RRT(item, failures)
                if sol is not None:
                    solutions.append(sol)
                    #print("Solution Length: " + str(len(sol)))
                    break
                else:
                    failures += 1
        
        #solution = self.aStar()
        
        self.plotMotion(solutions)
        #vel_msg.angular.z = angularChange / motionTime  #Rotation velocity is constant for now
    

    def processArc(self, arc):
        freeSpace = []
        discontinuities = []
        gap = False
        start = arc[0]
        end = arc[(len(arc)-1)]
        self.arc = []
        
        for a in range(0,len(arc)):       
            rounded = [int(round(arc[a][0])), int(round(arc[a][1]))]
            if self.world[rounded[0], rounded[1]] != 1:
                if not gap:
                    start = rounded
                    self.arc.append(arc[a])
                else:
                    end = rounded
                    self.arc.append(arc[a])
                    disc = Disc(start, end)
                    discontinuities.append(disc)
                    gap = False
                
            else:
                gap = True

        return discontinuities
                
            
    def RRT(self, gap, attempt):
        maxLoops = 2000
        loopNum = 0
        self.expandDist = 15
        self.expandFactor = 3
        
        bestSolution = None
        bestSolutionCost = np.inf
        
        startNode = Node(int(round(gap.start[0])), int(round(gap.start[1])))
        endNode = Node(int(round(gap.end[0])), int(round(gap.end[1])))
                
        self.T = [startNode]
        path = []#self.generatePath() #process path
        plt.clf()
        plt.imshow(self.world.T, cmap=cm.magma)
        plt.gca().invert_yaxis()
        #plt.plot(gap.start[0], gap.start[1], 'yo')
        
        while loopNum < maxLoops:
            loopNum += 1
            
            xRand = self.sample(gap, attempt) #returns node with x, y position
            xNearest, xNearestIndex = self.nearestVertex(xRand) #returns node and index in T
            xNew = self.extend(xNearest, xRand, xNearestIndex)
            if xNew == None:
                continue
            xNew = self.nearVert(xNew)
            if not self.edgeCollision(xNew.pos, xNew.parent):
                #plt.plot([xNew.x(), xNew.parentX()], [xNew.y(), xNew.parentY()], "r")
                #plt.pause(0.001)
                self.T.append(xNew) #Change this for RRT*
                if self.planarDist(xNew.pos, endNode.pos) <= self.expandFactor * self.expandDist:
                    if xNew.cost < bestSolutionCost:
                        bestSolutionCost = xNew.cost
                        bestSolution = xNew
                    
                    #print("Path found!")
                    #path = self.generatePath()
                    #plt.show()
                    #return path
        path = self.generatePath(bestSolution)
        if path != []:
            print("RRT SOLUTION FOUND")
            #plt.plot(gap.end[0], gap.end[1], 'yo')
            #plt.show()
            return path            
        else:
            print("RRT FAILED!")
            return None
        
        
    def nearVert(self, node):
        cost = np.inf
        for index, item in enumerate(self.T):
            dist = self.planarDist(node.pos, item.pos)
            if  dist <= self.expandFactor * self.expandDist:
                newCost = dist + item.cost
                if newCost < cost:
                    cost = newCost
                    best = item
                    ind = index
        if cost != np.inf:
            node.parent = best.pos
            node.parentInd = ind
        return node
        
    def generatePath(self, node):
        # T = list of Nodes

        #ind = len(self.T) - 1
        if node == None:
            return []
        currentNode = node
        #print(str(currentNode.parent))
        path = []
        a = len(self.T)
        b = 0
        while currentNode.parentInd is not [None, None] and b <= a:
            b += 1
            path.append(currentNode.pos)
            currentNode = self.T[currentNode.parentInd]
        return path

    # Checks for collision in linear line between node and parent                
    def edgeCollision(self, nodePos, parentPos):
        if nodePos == parentPos:
            return True
        delta = [None, None]
        dir = [1, 1]
        delta[0] = int(parentPos[0] - nodePos[0])
        delta[1] = int(parentPos[1] - nodePos[1])
        if delta[0] < 0:
            dir[0] = -1
        if delta[1] < 0:
            dir[1] = -1

        temp = [None, None]
        theta = math.atan2(delta[1], delta[0])
        #print(" ")
        #print("Node: " + str(nodePos))
        #print("Parent: " + str(parentPos))
        #print("Delta: " + str(delta))
        if abs(delta[0]) > abs(delta[1]):
            for a in range(dir[0], (delta[0]+dir[0]), dir[0]):# + 1)):
                #print("a: " + str(a))
                temp[0] = int(nodePos[0] + a)
                temp[1] = int(nodePos[1] + round(a*math.tan(theta)))
                #print("Temp: " + str(temp))
                if self.world[temp[0], temp[1]] == 1:
                    #print("Collision")
                    return True
        else:
            for a in range(dir[0], (delta[1]+dir[0]), dir[1]):#  + 1)):
                #print("a: " + str(a))
                temp[0] = int(nodePos[0] + round(a/math.tan(theta)))
                temp[1] = int(nodePos[1] + a)
                #print("Temp: " + str(temp))
                if self.world[temp[0], temp[1]] == 1:
                    #print("Collision")
                    return True
        return False
        
        
    def extend(self, nearest, rand, index):
        theta = math.atan2((rand.y() - nearest.y()), (rand.x() - nearest.x()))
        newNode = copy.deepcopy(nearest)
        newNode.pos[0] += int(round(self.expandDist * math.cos(theta)))
        
        newNode.pos[1] += int(round(self.expandDist * math.sin(theta)))
        newNode.parent = nearest.pos
        newNode.parentInd = index
        
        #if newNode.x() < 0:
        #    newNode.pos[0] = 0
        #if newNode.y() < 0:
        #    newNode.pos[1] = 0
        #if newNode.x() >= self.mapSize[0]:
        #    newNode.pos[0] = self.mapSize[0] - 1
        #if newNode.y() >= self.mapSize[1]:
        #    newNode.pos[1] = self.mapSize[1] - 1
        
        if newNode.x() < 0 or newNode.y() < 0:
            return None
        if newNode.x() >= self.mapSize[0] or  newNode.y() >= self.mapSize[1]:
            return None
            
        newNode.cost += self.planarDist(newNode.pos, newNode.parent)
        return newNode
        
                    
    def nearestVertex(self, x):
        cost = np.inf
        #ind = 0
        for index, item in enumerate(self.T):
            newCost = self.planarDist(x.pos, item.pos)
            if newCost < cost:
                cost = newCost
                nearest = item
                ind = index
        #print("Len T: " + str(len(self.T)))
        #print("Parent Ind: " + str(ind))
        #print("Cost: " + str(cost))
        return nearest, ind
        
                    
    def sample(self, gap, attempt):
        a = rand.random()
        if a >= 0.95:
            return Node(gap.end[0], gap.end[1])
        center = [None, None]
        delta = [None, None]
        center[0] = int(round(((gap.end[0] - gap.start[0]) * 0.5) + gap.start[0]))
        center[1] = int(round(((gap.end[1] - gap.start[1]) * 0.5) + gap.start[1]))
        
        delta[0] = abs(gap.start[0] - gap.end[0])
        delta[1] = abs(gap.start[1] - gap.end[1])
        
        if delta[0] > delta[1]:
            area = delta[0] + (attempt * delta[0] * 0.25)
        else:
            area = delta[1] + (attempt * delta[1] * 0.25)
            
        #print("Center: " + str(center))
        #print("Delta: " + str(delta))
        #print("Area: " + str(area))
        
        #x = rand.randint((-self.mapSize[0]), self.mapSize[0])
        #y = rand.randint((-self.mapSize[1]), self.mapSize[1])
        x = rand.randint((-area), area)
        y = rand.randint((-area), area)
        x = int(round(x + center[0]))
        y = int(round(y + center[1]))
        #if x < 0:
        #    x = 0
        #if y < 0:
        #    y = 0
        #if x >= self.mapSize[0]:
        #    x = self.mapSize[0] - 1
        #if y >= self.mapSize[1]:
        #    y = self.mapSize[1] - 1
        newSamp = Node(x, y)
        return newSamp

    
    # Creates a 2-D top-down view of the motion path
    # https://matplotlib.org/gallery/color/colormap_reference.html
    def plotMotion(self, solutions):
        plt.clf()
        plt.imshow(self.world.T, cmap=cm.magma)
        plt.gca().invert_yaxis()
        plt.plot(self.start[0], self.start[1], 'yo')
        plt.plot(self.target[0], self.target[1], 'go')
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        for a in self.arc:
            plt.plot(a[0], a[1], 'r.')
            #plt.pause(0.01 * self.cycleTime)
        for c in solutions:
            for b in c:
                plt.plot(b[0], b[1], 'm.')
            
        plt.plot(self.end[0], self.end[1], 'yo')
        plt.show()
        #plt.plot()
        #plt.imshow()
    
    
    def createPlanarArc(self):
        path = [] #[self.start[0], self.start[1], 0.0]
        radStart = self.planarDist(self.start, self.target)     #Radius about the target at the start of the arc
        radEnd = self.planarDist(self.end, self.target)         #Radius about the target at the end of the arc
        radDelt = radEnd - radStart                             #Difference between radii
        if self.printouts:
            print("Radius: " + str(radStart) + ", " + str(radEnd))
        
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
        
        if self.printouts:
            print("Theta: " + str(thetaStart) + ", " + str(thetaEnd) + ", "  + str(thetaDelt))
        
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
            path.append([x, y, tempTheta, t])
            if a%10 == 0 and self.printouts:
                print("[X, Y, theta, T]: " + str(x) + ", " + str(y) + ", " + str(tempTheta) + ", " + str(t))
                
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
        self.pos = [x, y]
        self.z = 1.0
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.cost = 0.0
        self.parent = [None, None]
        self.parentInd = 0
        
    def x(self):
        return self.pos[0]
 
    def y(self):
        return self.pos[1]
        
    def parentX(self):
        return self.parent[0]
        
    def parentY(self):
        return self.parent[1]
            
 
'''
#################################
###### Discontinuity Class ######
#################################
'''    

class Disc():
    def __init__(self, a, b):
        self.start = [a[0], a[1]]
        self.end = [b[0], b[1]]

          
'''
###############################
##   RBE 550 Final Project   ##
## Quadcopter Motion Planner ##
###############################
'''
    
if __name__ == '__main__':
    mp = MotionPlanner()
    mp.setup()
    mp.motionControl()
    
