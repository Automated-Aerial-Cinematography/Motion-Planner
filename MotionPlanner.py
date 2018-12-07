import sys
import time
import math
import copy
import heapq
import heapq_max
import numpy as np
import random as rand
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from get_model import *

occupancy_scale = 0.01


class MotionPlanner():
    
    def __init__(self, isStandAlone=False, rate=0):
        # Global variables
        self.timeReset = False
        self.end = [180, 600, 1]                                    # x, y, z
        self.start = [780, 740, 1]                                  # x, y, z
        self.target = [430, 610, 1]                                 # x, y, z
        self.currentPose = [0, 0, 0]                                # x, y, z

        self.startTime = 0.0                                        # Start time of motion
        self.cycleTime = 0.1                                        # Allows the main loop to run once every X seconds
        self.motionTime = 10.0                                      # Total time to complete motion in seconds
        self.angularChange = 0.0                                    # Angle between start and end with centerpoint at target
        self.clockwise = True
        self.printouts = False
        self.arc = []
        self.expandDist = 15
        self.expandFactor = 3
        self.smoothFactor = 4

        self.finalPath = []
        self.isStandAlone = isStandAlone
        self.rate = rate
        self.worldData = None
        self.mapSize = [2000, 2000]                                 # Size of the world
        self.worldData = None       
        if(not isStandAlone):
            self.minMapSize = [-2000, -2000] # Minimum Map Size
            self.world = None
        else:
            self.minMapSize = [0, 0] # Minimum Map Size
            self.world = np.zeros((self.mapSize[0],self.mapSize[0]))    # Create an empty world 
            
    def set_start(self, start):
        self.start = start
        
    def set_end(self, end):
        self.end = end
        
    def set_target(self, target):
        self.target = target
        
    def set_currentPose(self, currentPose):
        self.currentPose = currentPose
    
    def reset(self):
        self.arc = []
        self.finalPath = []
        
    def setWorldGetter(self, objPositionCollector):
        self.worldData = objPositionCollector   

    '''
    ####################################################################    
    ######################## Primary Functions #########################
    ####################################################################
    '''
    
    # Runs any necessary setup.
    # Inputs:
    #   None
    # Return:
    #   None
    def setup(self):    
        print("Setup")
        if(self.isStandAlone):
            self.addSquare([760,500], 70)
            self.addCircle([350,350], 60)
        
       
    # Acts as a top level method for calling motion control. 
    # Inputs:
    #   printData:
    # Return:
    #   None
    def motionControl(self, printData=True):
        self.arc = self.createPlanarArc()
        disc = self.processArc(self.arc)
        failures = 0
        
        solutions = []
        print("Number of discontinuities: " + str(len(disc)))
        for item in disc:
            while failures < 5:
                sol = self.RRTstar(item, failures, 3000)
                if sol is not None:
                    solutions.append(sol)
                    break
                else:
                    failures += 1
        if failures < 5:
            self.finalPath = self.generateFinalPath(solutions)
            temp = self.isStandAlone
            if(printData):
                self.isStandAlone = True
            self.plotMotion(solutions, disc)
            self.isStandAlone = temp
        else:
            self.finalPath = []
            

    # Performs the RRT* motion control calculations.
    # Inputs:
    #   gap: A discontinuity over which to perform the RRT* motion planning.
    #   attempt: The current attempt number. Used to expand the seach area after failed attempts.
    #   maxLoops: Maximum number of loops before the current RRT* search terminates.
    # Return:
    #   path: The final path as a sequance of node locations.
    #   None: Returns 'None' if the RRT* search fails to find a valid path.
    def RRTstar(self, gap, attempt, maxLoops=3000):    
        startTime = time.time()
        #maxLoops = 3000
        loopNum = 0
                
        bestSolution = None
        bestSolutionInd = None
        bestSolutionCost = np.inf
        
        startNode = Node(gap.start[0], gap.start[1])
        endNode = Node(gap.end[0], gap.end[1])
                
        self.T = [startNode]
        path = []
        if(self.isStandAlone):
            plt.clf()
            if(self.world != None):
                plt.imshow(self.world.T, cmap=cm.magma)
            plt.gca().invert_yaxis()
        
        while loopNum < maxLoops:
            loopNum += 1
            
            xRand = self.sample(gap, attempt)
            xNearest, xNearestIndex = self.nearestVertex(xRand)
            xNew = self.extend(xNearest, xRand, xNearestIndex)
            if xNew == None:
                continue
            xNew = self.nearVert(xNew)
            if not self.edgeCollision(xNew.pos, xNew.parent):
                self.T.append(xNew)
                if self.planarDist(xNew.pos, endNode.pos) <= self.expandFactor * self.expandDist:
                    if xNew.cost < bestSolutionCost:
                        bestSolutionCost = xNew.cost
                        bestSolutionInd = len(self.T) - 1
                        bestSolution = xNew
        if(bestSolution is None):
            print("Best Solution is None!")
        else:                        
            endNode.parent = bestSolution.pos
            endNode.parentInd = bestSolutionInd
            endNode.cost = bestSolution.cost + self.planarDist(endNode.pos, bestSolution.pos)
            path = self.generatePath(endNode)
        endTime = time.time()
        print("RRT* Time: " + str(endTime - startTime))
        if path != []:
            print("RRT* SOLUTION FOUND")
            return path            
        else:
            print("RRT* FAILED!")
            return None
  

    # Processes the pre-calculated arc. Will simplify it into a series of points at a set distance and notes any discontinuities
    # Inputs:
    #   arc: Pre-computed ideal path arc
    # Return:
    #   discontinuities: a list of discontinuities caused by obstacle impeding the arc.
    def processArc(self, arc):
        freeSpace = []
        discontinuities = []
        gap = False
        start = [int(arc[0][0]), int(arc[0][1])]
        self.arc = []
        self.arc.append(start)

        for a in range(1,len(arc)):       
            rounded = [int(round(arc[a][0])), int(round(arc[a][1]))]
            #print(rounded)
            if self.get_occupancy_data(rounded[0], rounded[1]) != 1:
                if not gap:
                    if self.planarDist(self.arc[-1], arc[a]) >= self.expandFactor  * self.expandDist:
                        self.arc.append(rounded)
                        start = rounded
                else:
                    end = [int(round(arc[a+self.expandFactor][0])), int(round(arc[a+self.expandFactor][1]))]
                    self.arc.append(None)
                    self.arc.append(end)
                    disc = Disc(start, end)
                    discontinuities.append(disc)
                    gap = False
                
            else:
                if self.planarDist(rounded, start) < self.expandFactor * self.expandDist and gap == False:
                    #print('Arc',self.arc, a)

                    start = self.arc[-2]

                    self.arc.pop()
                gap = True
        if self.arc[-1] != self.end:
            self.arc.append([int(round(self.end[0])), int(round(self.end[1]))])
        return discontinuities
    
    
    # Sets a node's parent to be the closest esisting node in terms of linear distance. 
    # Inputs:
    #   node: New node to be connected to nearest existing node.
    # Return:
    #   node: Modified node.  
    def nearVert(self, node):
        cost = np.inf
        for index, item in enumerate(self.T):
            if node.pos == item.pos:
                continue
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
        
    # Follows the RRT* tree backwards from a node to the start node.
    # Inputs:
    #   node: The node from which to trace backwards.
    # Return:
    #   path: A list of points representing the nodes which connect the start node to the desired node.
    def generatePath(self, node):
        if node == None:
            return []
        currentNode = node
        path = []
        a = len(self.T)
        b = 0
        while currentNode.parentInd is not None and b <= a:
            b += 1
            path.append(currentNode.pos)
            currentNode = self.T[currentNode.parentInd]
        return path


    # Checks for collision in linear line between a node and its parent or any two points.
    # Inputs:
    #   nodePos: The x-y position of the node.
    #   parentPos: The x-y position of the parent.
    # Return:
    #   True if there is a obstacle blocking the straight line path between the two points
    #   False if there is NOT a obstacle blocking the straight line path between the two points 
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
        if abs(delta[0]) > abs(delta[1]):
            for a in range(dir[0], (delta[0]+dir[0]), dir[0]):
                temp[0] = int(nodePos[0] + a)
                temp[1] = int(nodePos[1] + round(a*math.tan(theta)))
                if self.get_occupancy_data(temp[0], temp[1]) == 1:
                    return True
        else:
            for a in range(dir[0], (delta[1]+dir[0]), dir[1]):#  + 1)):
                temp[0] = int(nodePos[0] + round(a/math.tan(theta)))
                temp[1] = int(nodePos[1] + a)
                if self.get_occupancy_data(temp[0], temp[1]) == 1:
                    return True
        return False
        
    # Creates a new node by extending from an existing node in the direction of a given point.
    # Inputs:
    #   nearest: The closest existing node to the random, given point.
    #   rand: The random x-y position towards which to extend the tree.
    #   index: The index of the closest existing node in the tree list.
    # Outputs:
    #   newNode: Returns the new node if it exists within the bounds of the world.
    #   'None: Returns 'None' if the new node exists outside the world bounds.
    def extend(self, nearest, rand, index):
        theta = math.atan2((rand.y() - nearest.y()), (rand.x() - nearest.x()))
        newNode = copy.deepcopy(nearest)
        newNode.pos[0] += int(round(self.expandDist * math.cos(theta)))
        
        newNode.pos[1] += int(round(self.expandDist * math.sin(theta)))
        newNode.parent = nearest.pos
        newNode.parentInd = index
        
        if newNode.x() < self.minMapSize[0] or newNode.y() < self.minMapSize[1]:
            return None
        if newNode.x() >= self.mapSize[0] or  newNode.y() >= self.mapSize[1]:
            return None
            
        newNode.cost += self.planarDist(newNode.pos, newNode.parent)
        return newNode
        
    # Returns the closest existing node to a point.
    # Input:
    #   x: The point from which to find the closest existing node.
    # Return:
    #   nearest: The closest node to the point x
    #   ind: The index of the closets node in the RRT* tree list.
    def nearestVertex(self, x):
        cost = np.inf
        for index, item in enumerate(self.T):
            newCost = self.planarDist(x.pos, item.pos)
            if newCost < cost:
                cost = newCost
                nearest = item
                ind = index
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
            area = int(round((0.65 * delta[0]) + (attempt * delta[0] * 0.25)))
        else:
            area = int(round((0.65 * delta[1]) + (attempt * delta[1] * 0.25)))

        x = rand.randint((-area), area)
        y = rand.randint((-area), area)
        x = int(round(x + center[0]))
        y = int(round(y + center[1]))

        newSamp = Node(x, y)
        #print(x,y)
        return newSamp

    
    # Creates a 2-D top-down view of the motion path
    def plotMotion(self, solutions, disc):
        finalPath = []
        if(self.isStandAlone):
            plt.clf()
            if(self.world != None):
                plt.imshow(self.world.T, cmap=cm.magma)
            plt.gca().invert_yaxis()
            plt.plot(self.start[0], self.start[1], 'yo')
            plt.plot(self.target[0], self.target[1], 'go')
            plt.xlabel("x-axis")
            plt.ylabel("y-axis")
        discNum = 0

        for a in self.arc:
            if a != None:
                finalPath.append([a[0], a[1]])
                if(self.isStandAlone):
                    plt.plot(a[0], a[1], 'r.')
            else:
                #print("Here")
                for b in solutions[discNum][::-1]:
                    finalPath.append([b[0], b[1]])
                    if(self.isStandAlone):
                        plt.plot(b[0], b[1], 'm.')
                    
                discNum += 1
            if(self.isStandAlone):
                plt.pause(0.11 * self.cycleTime)

        
        print("Final Path Nodes: " + str(len(finalPath))) 
        if(self.isStandAlone):
            plt.plot(self.end[0], self.end[1], 'yo')
            plt.show()
        
            plt.clf()
            if(self.world != None):
                plt.imshow(self.world.T, cmap=cm.magma)
            plt.gca().invert_yaxis()
            plt.plot(self.start[0], self.start[1], 'yo')
            plt.plot(self.target[0], self.target[1], 'go')
            plt.xlabel("x-axis")
            plt.ylabel("y-axis")
        if(self.isStandAlone):
            for a in range (0, (len(finalPath)-1)):
                plt.plot([finalPath[a][0], finalPath[a+1][0]], [finalPath[a][1], finalPath[a+1][1]], "r")
            plt.plot(self.end[0], self.end[1], 'yo')
            plt.show()


    def generateFinalPath(self, solutions):
        finalPath = []
        discNum = 0
        for a in self.arc:
            if a != None:
                finalPath.append([a[0], a[1]])
            else:
                #print("Here")
                for b in solutions[discNum][::-1]:
                    finalPath.append([b[0], b[1]])  
                discNum+=1
        return finalPath


    def createPlanarArc(self):
        path = []
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
        
        

    '''
    ####################################################################    
    ######################### Helper Functions #########################
    ####################################################################
    '''
    
    # Adds a square shape to the internal world map used while debugging
    # Inputs:
    #   pos: Postition of the center of the square
    #   size: Width and height of the square
    # Return:
    #   None   
    def addSquare(self, pos, size):
        halfSize = int(size/2)
        for x in range((-halfSize), halfSize):
            for y in range((-halfSize), halfSize):
                self.world[(pos[0]+x),(pos[1]+y)] = 1
    
    
    # Adds a circle shape to the internal world map used while debugging
    # Inputs:
    #   pos: Postition of the center of the circle
    #   size: Radius of the circle
    # Return:
    #   None               
    def addCircle(self, pos, rad):
        for x in range((-rad), rad):
            for y in range((-rad), rad):
                point  = [(pos[0] + x), (pos[1] + y)]
                if self.planarDist(pos, point) <= rad:
                    self.world[(pos[0]+x),(pos[1]+y)] = 1


    # Calculates the angle between two points about a  third target point projected into the x-y plane
    # Inputs:
    #   a: [x, y(, z)] position for first point
    #   b: [x, y(, z)] position of second point
    #   c: [x, y(, z)] position of target or center of rotation
    # Return:
    #   Angle between a and b about c in x-y plane
    def planarAngle(self, a, b, c):
        deltaA = [(a[0] - c[0]), (a[1] - c[1])]
        angleA = math.atan2(deltaA[1], deltaA[0])
        if angleA < 0.0:
            angleA += (2 * math.pi) 
        deltaB = [(b[0] - c[0]), (b[1] - c[1])]
        angleB = math.atan2(deltaB[1], deltaB[0])
        if angleB < 0.0:
            angleB += (2 * math.pi)
        angle = angleB - angleA
        if angle < 0.0:
            angle += (2 * math.pi)
        if angle >= 2 * math.pi:
            angle -= (2 * math.pi)
        return angle
    
    
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
        
        
    # Querys the world map to check is a point exists in C-free.
    # Inputs:
    #   x: x coordinate of the point
    #   y: y coordinate of the point
    #   z: z coordinate of the point
    # Return:
    #   True if the point exists in C-free.
    #   False if the point exists in C-obs.
    def get_occupancy_data(self, x, y, z=1):
        if(self.isStandAlone):
            return self.world[x, y]
        else:
            #t1 = time.time()
            occ = self.worldData.get_occupancy_2d(float(x)*occupancy_scale,float(y)*occupancy_scale,1)
            #t2 = time.time()
            #print("Occ Check time "+ str(t2-t1))
            #print(occ, x,y, float(x)*occupancy_scale,float(y)*occupancy_scale)
            return occ



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
        self.parentInd = None
        
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
    if(len(sys.argv) > 1):
        if(sys.argv[1]=="ROS"):
            mp = MotionPlanner(False, 20)# Starting out slow so that we can ease into this
            mp.run()
        else:
            print("Err: Unknown Params", sys.argv)
    else:
        mp = MotionPlanner(True)
        mp.setup()
        mp.motionControl()
