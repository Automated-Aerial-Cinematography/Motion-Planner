import rospy
from geometry_msgs.msg import PoseStamped, Point, Twist
from tf.transformations import euler_from_quaternion
import math
# Path = [[3,3,3], [5,4,1], [5,6,3]]
class Move_Drone():

    def __init__(self,Path):
        # rospy.init_node("move_drone")
        self.Path = Path
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.pose_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.speed = Twist()
        self.r = rospy.Rate(4)
        # To access co-ordinates
        self.location = Point()


    def pose_callback(self,data):
        # global drone_x,drone_y,drone_z,roll,pitch,yaw

        #Drone position in space(x,y,z)
        self.drone_x = data.pose.position.x
        self.drone_y = data.pose.position.y
        self.drone_z = data.pose.position.z

        # Orientation of drone
        self.rot = data.pose.orientation
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.rot.x, self.rot.y, self.rot.z, self.rot.w])






    # Goal location(x,y.z)
    #location.x = 3
    #location.y = 3
    #location.z = 3

    def reach_z(self,delta,linear_speed_z):
        if delta > 0:
            self.speed.angular.x, self.speed.angular.y, self.speed.angular.z = 0.0, 0.0, 0.0
            self.speed.linear.x , self.speed.linear.y = 0.0, 0.0
            self.speed.linear.z = linear_speed_z

        elif delta < 0:
            self.speed.angular.x, self.speed.angular.y, self.speed.angular.z = 0.0, 0.0, 0.0
            self.speed.linear.x , self.speed.linear.y = 0.0, 0.0
            self.speed.linear.z = -linear_speed_z

    def turn_left(self,turning_speed_z):
        self.speed.angular.z = turning_speed_z
        self.speed.angular.x ,self.speed.angular.y = 0.0, 0.0

    def turn_right(self,turning_speed_z):
        self.speed.angular.z = -turning_speed_z
        self.speed.angular.x , self.speed.angular.y = 0.0, 0.0

    def stop(self):
        self.speed.linear.x , self.speed.linear.y = 0.0, 0.0
        self.speed.angular.x, self.speed.angular.y, self.speed.angular.z = 0.0, 0.0, 0.0

    def move_towards_target(self,linear_speed_x):
        self.speed.angular.x, self.speed.angular.y, self.speed.angular.z = 0.0, 0.0, 0.0
        self.speed.linear.x = linear_speed_x


    def main(self):
        i = 0
        # Path = [[3,3,3], [5,4,1], [5,6,3]]
        while not rospy.is_shutdown():
            if i< len(self.Path):
            # Distance and angle of robot from goal
                self.delta_x = self.Path[i][0] - self.drone_x
                self.delta_y = self.Path[i][1] - self.drone_y
                self.delta_z = 3 - self.drone_z
                self.angle_with_goal = math.atan2(self.delta_y,self.delta_x)
                #print('Path_x:{},Path_y{},Path_z{}' .format(Path[i][0],Path[i][1],Path[i][2]))
                #print('drone_x:{},drone_y{},drone_z{}' .format(drone_x,drone_y,drone_z))

                if abs(self.delta_z) >= 0.1:
                    self.drone_height = self.reach_z(self.delta_z,0.3)

                else:
                    self.speed.linear.z = 0.0
                    if abs(self.angle_with_goal - self.yaw) > 0.1 and self.angle_with_goal > self.yaw:
                        self.turn = self.turn_left(0.3)

                        if self.delta_x <0.1 and self.delta_y <0.1:
                            Stop = self.stop()

                    elif abs(self.angle_with_goal - self.yaw) > -0.1 and self.angle_with_goal < self.yaw:
                        self.turn = self.turn_right(0.3)

                        if self.delta_x <0.1 and self.delta_y <0.1:
                            Stop = self.stop()

                    else:
                        move = self.move_towards_target(0.3)

                        if self.delta_x <0.1 and self.delta_y <0.1:
                            Stop = self.stop()
                            i += 1

            self.pub.publish(self.speed)
#     # r.sleep()
# Path = [[3,3,3], [5,4,1], [5,6,3]]
# mv_dr = Move_Drone(Path)
# mv_dr.main()