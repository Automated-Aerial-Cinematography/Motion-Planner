import rospy
from geometry_msgs.msg import PoseStamped, Point, Twist
from tf.transformations import euler_from_quaternion
import math

drone_x = 0.0
drone_y = 0.0
drone_z = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

def pose_callback(data):
    global drone_x,drone_y,drone_z,roll,pitch,yaw

    #Drone position in space(x,y,z)
    drone_x = data.pose.position.x
    drone_y = data.pose.position.y
    drone_z = data.pose.position.z

    # Orientation of drone
    rot = data.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

rospy.init_node("move_drone")

sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

speed = Twist()
r = rospy.Rate(4)


# To access co-ordinates
location = Point()

def reach_z(delta):
    if delta > 0:
        speed.angular.x, speed.angular.y, speed.angular.z = 0.0, 0.0, 0.0
        speed.linear.x , speed.linear.y = 0.0, 0.0
        speed.linear.z = 0.3

    elif delta < 0:
        speed.angular.x, speed.angular.y, speed.angular.z = 0.0, 0.0, 0.0
        speed.linear.x , speed.linear.y = 0.0, 0.0
        speed.linear.z = -0.3

i = 0
Goal = [2,2,3]
Path = [[3,3,3], [5,4,1], [5,6,3]]
while not rospy.is_shutdown():
    if i< len(Path):
    # Distance and angle of robot from goal
        delta_x = Path[i][0] - drone_x
        delta_y = Path[i][1] - drone_y
        delta_z = Path[i][2] - drone_z
        goal_delta_x = Goal[0] - drone_x
        goal_delta_y = Goal[1] - drone_y
        goal_delta_z = Goal[2] - drone_z
        angle_with_goal = math.atan2(goal_delta_y, goal_delta_x)
        angle_with_target = math.atan2(delta_y,delta_x)
        dist_from_target = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        #print('Path_x:{},Path_y{},Path_z{}' .format(Path[i][0],Path[i][1],Path[i][2]))
        #print('drone_x:{},drone_y{},drone_z{}' .format(drone_x,drone_y,drone_z))

        if abs(delta_z) >= 0.1:
            drone_height = reach_z(delta_z)

        else:
            speed.linear.z = 0.0
            if abs(angle_with_goal - yaw) > 0.1 and angle_with_goal > yaw:
                speed.angular.z = 0.3
                speed.angular.x , speed.angular.y = 0.0, 0.0

                if delta_x <0.1 and delta_y <0.1:
                    speed.linear.x , speed.linear.y = 0.0, 0.0
                    speed.angular.x, speed.angular.y, speed.angular.z = 0.0, 0.0, 0.0

            elif abs(angle_with_goal - yaw) > -0.1 and angle_with_goal < yaw:
                speed.angular.z = -0.3
                speed.angular.x , speed.angular.y = 0.0, 0.0

                if delta_x <0.1 and delta_y <0.1:
                    speed.linear.x , speed.linear.y = 0.0, 0.0
                    speed.angular.x, speed.angular.y, speed.angular.z = 0.0, 0.0, 0.0

            else:
                speed.angular.x, speed.angular.y, speed.angular.z = 0.0, 0.0, 0.0
                speed.linear.x = dist_from_target*math.sin(angle_with_target)
                speed.linear.y = dist_from_target*math.cos(angle_with_target)

                if delta_x <0.1 and delta_y <0.1:
                    speed.linear.x , speed.linear.y = 0.0, 0.0
                    speed.angular.x, speed.angular.y, speed.angular.z = 0.0, 0.0, 0.0
                    i += 1

    pub.publish(speed)
    # r.sleep()
