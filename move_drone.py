import rospy
from geometry_msgs.msg import PoseStamped, Point, Twist
from tf.transformations import euler_from_quaternion
import math

i = 0

# Path = [[3,3,3], [5,4,1], [5,6,3]]
class Move_Drone():

    def __init__(self,Path, target=[0,0,0]):
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
        
        self.target = target
        self.speed = Twist()
        self.r = rospy.Rate(0.002)
        # To access co-ordinates
        self.location = Point()


    def pose_callback(self,data):
        # global drone_x,drone_y,drone_z,roll,pitch,yaw
        print("Pose callback called")
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

    def move_in_direction(self,linear_speed, angle_off_x):
        #self.speed.angular.x, self.speed.angular.y, self.speed.angular.z = 0.0, 0.0, 0.0
        self.speed.linear.x = linear_speed * math.cos(angle_off_x)
        self.speed.linear.y = linear_speed * math.sin(angle_off_x)
        #print(self.speed.linear.x, self.speed.linear.y)
    def main(self, position = None, orientation = None):
        global i
        if(position != None):
            self.drone_x = position[0]
            self.drone_y = position[1]
            self.drone_z = position[2]
        if(orientation != None):
            self.roll = orientation[0]
            self.pitch = orientation[1]
            self.yaw = orientation[2]
        self.threshold = math.pi / 60        # Path = [[3,3,3], [5,4,1], [5,6,3]]
        while not rospy.is_shutdown():
            if i < len(self.Path):
            # Distance and angle of robot from goal
                self.delta_x = self.Path[i][0] - self.drone_x
                self.delta_y = self.Path[i][1] - self.drone_y
                self.delta_z = 2 - self.drone_z
                self.angle_with_goal = math.atan2(self.delta_y,self.delta_x) - math.pi/2
                self.angle_with_target = math.atan2((self.target[1]-self.drone_y),(self.target[0] - self.drone_x)) #- math.pi/2
    
                self.dist_from_goal = math.sqrt(self.delta_x*self.delta_x + self.delta_y*self.delta_y) 
                #print('Path_x:{},Path_y:{},Path_z:{}' .format(self.Path[i][0],self.Path[i][1],3))
                #print('drone_x:{},drone_y{},drone_z{}' .format(drone_x,drone_y,drone_z))
               # print("Angle with target: ", self.angle_with_target)
               # print("Yaw: ", self.yaw)
               # print("Position: ", self.drone_x, self.drone_y, self.drone_z)
               # print("Target: ", self.target[0], self.target[1])
    
                if abs(self.delta_z) >= self.threshold:
                    self.drone_height = self.reach_z(self.delta_z,0.3)
    
                else:
                    self.speed.linear.z = 0.0
                    # Handling what the turn rate should be. Default to this value then check if it needs to be lower
                    turn_rate = 0.3
                   # print("Angle with target ", self.angle_with_target, "Yaw", self.yaw, "Check ", (abs(self.angle_with_target-self.yaw) > self.threshold))
                    if(abs(self.angle_with_target-self.yaw) < 1.5*turn_rate):
                        # Since the threshold is not guaranteed to be less than turn_rate we don't want to accidentally bump up the turn rate.
                        turn_rate = min(self.threshold - 0.01, turn_rate - 0.01)
                        
                    if abs(self.angle_with_target-self.yaw) > self.threshold and self.angle_with_target > self.yaw:
                        self.turn = self.turn_left(turn_rate)
                        #print("Turning Left ", turn_rate, self.angle_with_target, self.yaw)
                       # if self.dist_from_goal <= 0.1:
                       #     Stop = self.stop()
                       #     print("Delta", self.dist_from_goal)
                            #i += 1
    
                    elif abs(self.angle_with_target-self.yaw) > self.threshold and self.angle_with_target < self.yaw:
                        self.turn = self.turn_right(turn_rate)
                        #print("Turning Right ", turn_rate, self.angle_with_target, self.yaw)
                        #if self.dist_from_goal <= 0.1:
                        #    Stop = self.stop()
                        #    print("Delta", self.dist_from_goal)
                            #i += 1
                    else:
                        #print("Not Turning...")
                        self.turn = self.turn_left(0.0)
                        
                    #else:
                    vel_to_move = 0.3
                    if(abs(self.dist_from_goal) < 3.0*vel_to_move):
                        vel_to_move = 0.15
                    move = self.move_in_direction(vel_to_move, self.angle_with_goal)
                    #print("Dist to ", i, " = ", self.dist_from_goal, "Movement", vel_to_move, "With angle ", self.angle_with_goal)
                    if self.dist_from_goal <= 0.1:
                        Stop = self.stop()
                        i += 1
    
            self.pub.publish(self.speed)
            #self.r.sleep()
# Path = [[-3.000171556929056, -0.0021987329929744285], [-6.000171556929056, -0.6021987329929744], [-10.300171556929056, 0.19780126700702566], [-13.000171556929057, 1.5978012670070258], [-16.600171556929055, 4.1978012670070255], [-18.500171556929054, 8.197801267007026], [-18.900171556929056, 12.597801267007025], [-19.400171556929056, 16.697801267007026], [-19.400171556929056, 20.997801267007027], [-18.800171556929055, 24.797801267007028], [-17.300171556929055, 27.397801267007026], [-15.500171556929057, 29.797801267007028], [-15.400171556929056, 33.397801267007026], [-16.400171556929056, 36.897801267007026], [-15.900171556929054, 40.69780126700703], [-13.100171556929057, 44.19780126700702], [-10.900171556929054, 46.397801267007026], [-8.000171556929056, 49.49780126700703], [-3.6001715569290553, 50.397801267007026], [-1.6001715569290553, 53.29780126700703], [-0.03235535490787814, 57.385999980841234], [0.0, 57.300000000000004], [4.9, 57.7], [9.7, 58.8], [14.399999999999999, 60.5], [18.700000000000003, 62.800000000000004], [22.7, 65.8], [26.200000000000003, 69.3], [29.2, 73.2], [27.0, 74.4], [24.0, 74.3], [20.300000000000004, 73.8], [17.9, 77.2], [17.0, 81.5], [19.1, 85.4], [19.3, 89.3], [19.1, 93.4], [19.6, 97.6], [19.8, 101.4], [19.5, 104.9], [18.6, 108.6], [21.099999999999998, 111.8], [25.300000000000004, 112.9], [28.900000000000002, 112.10000000000001], [28.900000000000002, 112.10000000000001], [25.8, 116.0], [22.200000000000003, 119.39999999999999], [18.2, 122.2], [18.2, 119.2], [17.4, 114.9], [16.6, 110.8], [12.3, 110.0], [8.3, 110.1], [4.2, 109.39999999999999], [0.5, 111.6], [-2.7, 113.2], [-6.7, 113.70000000000002], [-10.9, 112.8], [-14.2, 115.60000000000001], [-18.0, 117.8], [-22.0, 119.7], [-22.0, 119.7], [-25.6, 116.30000000000001], [-28.700000000000003, 112.5], [-31.200000000000003, 108.3], [-33.2, 103.70000000000002], [-34.4, 98.9], [-35.0, 94.0], [-35.1, 92.4]]


# mv_dr = Move_Drone(Path)
# mv_dr.main()