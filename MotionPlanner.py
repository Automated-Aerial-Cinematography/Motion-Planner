import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped

# Global variables
timeReset = False
start = [0,0,1]         # x, y, z
end = [10, 0, 1]        # x, y, z
target = [5, 10, 1]     # x, y, z
currentPose = [0, 0, 0] # x, y, z
startTime = 0.0         # Start time of motion
cycleTime = 0.1         # Allows the main loop to run once every X seconds
motionTime = 10.0       # Total time to complete motion in seconds
angularChange = 0.0     # Angle between start and end with centerpoint at target

rospy.init_node('quad_motion', anonymous=True)
velocity_publisher = rospy.Publisher('/command/twist', Twist, queue_size=10)
#telemetry_listener = rospy.Subscriber("????????", String, updateData)
vel_msg = Twist()


'''
####################################################################    
######################## Primary Functions #########################
####################################################################
'''

# General setup to occur once
def setup():    
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    velocity_publisher.publish(vel_msg)
    rospy.Timer(rospy.Duration(cycleTime), timerEvent)
    angularChange = planarAngle()    
    startTime = rospy.get_time()
    
    
# Performs the motion control calculations    
def motionControl():
    elapsedTime = rospy.get_time() - startTime
    vel_msg.angular.z = angularChange / motionTime  #Rotation velocity is constant for now

    
# Sends quadcopter velocity commands via a ROS publisher
def sendCommands():
    velocity_publisher.publish(vel_msg)


# ROS Subscriber for quadcopter position and orientation
# Updates global variables related to quadcopter pose
def updateData():
    #currentPose
    pass


# Forces a tick rate for the motion planner
# Main loop cycle time is controlled by changing timeReset
def timerEvent(event):
    timeReset = True    
    
    
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
def planarAngle(a, b, c):
    aToC = planarDist(a, c)    # First to Target distance
    bToC = planarDist(b, c)    # Second to Target distance
    aToB = planarDist(a, b)    # First to Second distance
    
    # Law of cosines: cos(A) = (b^2 + c^2 - a^2)/(2*b*c)
    return arccos(((aToC*aToC) + (bToC*bToC) - (aToB*aToB))/ (2*aToC*bToC))

    
# Calculates the distance between two points projected onto the x-y plane
# Inputs:
#   a: [x, y(, z)] position of first point
#   b: [x, y(, z)] position of second point
# Return:
#   Linear distance between a and b in x-y plane
def planarDist(a, b)
    xSq = (b[0] - a[0])^2
    ySq = (b[1] - a[1])^2
    return sqrt(xSq + ySq)

    
# Calculates the distance between two points in 3D space
# Inputs:
#   a: [x, y, z] position of first point
#   b: [x, y, z] position of second point
# Return:
#   Linear distance between a and b
def trueDist(a, b)
    xSq = (b[0] - a[0])^2
    ySq = (b[1] - a[1])^2
    zSq = (b[2] - a[2])^2
    return sqrt(xSq + ySq + zSq)


# Linearly interpolates between the start and end positions
# Inputs:
#   a: [x, y, z] start position
#   b: [x, y, z] end position
#   t: Desired return time
#   tF: Total duration of interpolation
# Return:
#   [x, y, z] position of quadcopter at time t    
def linInter(a, b, t, tF):
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
def cirInter(a, b, t, tF):
    pass

    
'''    
####################################################################  
########################### Main Function ##########################
####################################################################
'''
    
if __name__ == '__main__':
    try:
        setup()
        #updateData()
        createPath()
        
        while True:        
            if timeReset:
                #updateData()
                motionControl()
                timeReset = False
        
    except rospy.ROSInterruptException: pass