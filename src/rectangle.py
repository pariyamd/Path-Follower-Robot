#!/usr/bin/python3

from dis import dis
import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose

from math import radians

import numpy as np
import math
import time
class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        # self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        # self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(90) # rad
        # self.stop_distance = rospy.get_param("/rectangle/stop_distance") # m
        # self.epsilon = rospy.get_param("/rectangle/epsilon")
        self.width = 2
        self.length = 3
        self.pose_x = 0
        self.pose_y = 0
        self.vel= Twist()
        self.vel.linear.x = 0.2  # m/s
        self.vel.angular.z = 0.1  # rad/s
        # defining the states of our robot
        self.GOw, self.GOl, self.ROTATEl, self.ROTATEw = 0, 1, 2, 3
        self.state = self.GOw
        self.p=3
        self.i_error=0
        self.counter = 0
        self.ds=0.1
        self.k_p=0.2 #0.2
        self.k_theta=1.3 #1.4
        self.k_i=0.0001
        self.k_d=0.005
        self.k_i_max=50
        self.k_i_min=-50
        self.last_time= None
        self.sigma_error=0
        self.distance_error_last = 0.0
        self.path=self.generate_path()
    
    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        self.pose_x= msg.pose.pose.position.x
        self.pose_y= msg.pose.pose.position.y
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw
    


    def generate_path(self):
        if self.p==1:
            X1 = np.linspace(-3, 3 , 100)
            Y1 = np.array([2]*100)

            Y2 = np.linspace(2, -2 , 100)
            X2 = np.array([3]*100)

            X3 = np.linspace(3, -3 , 100)
            Y3 = np.array([-2]*100)

            Y4 = np.linspace(-2, 2 , 100)
            X4 = np.array([-3]*100)

            return (np.concatenate([X1,X2, X3 , X4]), np.concatenate([Y1,Y2,Y3,Y4]))
        if self.p==2:
            a = 0.17
            k = math.tan(a)
            X , Y = [] , []

            for i in range(150):
                t = i / 20 * math.pi
                dx = a * math.exp(k * t) * math.cos(t)
                dy = a * math.exp(k * t) * math.sin(t)
                X.append(dx)
                Y.append(dy) 
            return (X,Y)
        if self.p==3:
            X1 = np.linspace(-3., -1 , 50)
            Y1 = np.zeros((50,))

            x_dim, y_dim = 1,1
            t = np.linspace(np.pi, 0, 100)
            X2 = x_dim * np.cos(t) 
            Y2 = y_dim * np.sin(t)

            X3 = np.linspace(1, 3 , 50)
            Y3 = np.zeros((50,))

            x_dim, y_dim = 3,3
            t = np.linspace(np.pi*2, np.pi, 200)
            X4 = x_dim * np.cos(t) 
            Y4 = y_dim * np.sin(t)
            return (np.concatenate([X1,X2, X3 , X4]), np.concatenate([Y1,Y2,Y3,Y4]))
            
        if self.p==4:
            X1 = np.linspace(-1, 1 , 10)
            Y1 = np.array([3]*10)

            X2 = np.linspace(1, 1 + 2**(1/2) , 10)
            Y2 = - (2**(1/2)) * (X2 - 1) + 3

            Y3 = np.linspace(1, -1 , 10)
            X3 = np.array([1 + 2**(1/2)]*10)

            X4 = np.linspace(1 + 2**(1/2), 1, 10)
            Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 

            X5 = np.linspace(1, -1 , 10)
            Y5 = np.array([-3]*10)

            X6 = np.linspace(-1, -1 - 2**(1/2) , 10)
            Y6 = - (2**(1/2)) * (X6 + 1) - 3 


            Y7 = np.linspace(-1, 1 , 10)
            X7 = np.array([- 1 - 2**(1/2)]*10)


            X8 = np.linspace(-1 - 2**(1/2), -1, 10)
            Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1

            return(np.concatenate([X1,X2,X3,X4,X5,X6,X7,X8]), np.concatenate([Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8]))
           
        if self.p==5:
            growth_factor = 0.1
            X , Y = [] , []

            for i in range(400):
                t = i / 20 * math.pi
                dx = (1 + growth_factor * t) * math.cos(t)
                dy = (1 + growth_factor * t) * math.sin(t)
                X.append(dx)
                Y.append(dy) 
            return(X,Y)

    def angular_error(self, goal_x, goal_y):
        theta_star = math.atan2(goal_y - self.pose_y, goal_x - self.pose_x)
        if theta_star - self.yaw < -math.pi:
            return theta_star- self.yaw + 2 * math.pi
        if theta_star - self.yaw > math.pi:
            return theta_star- self.yaw - 2 * math.pi
        rospy.loginfo(f"theta_star:{theta_star}, yaw: {self.yaw}")
        return theta_star - self.yaw

    def dist(self, goal_x, goal_y):
        return math.sqrt(((goal_x - self.pose_x) ** 2) + ((goal_y - self.pose_y) ** 2))- self.ds

    def update_PID(self,distance_error, dt=None):

        if dt == None:
            cur_time = time.time()
            if self.last_time is None:
                self.last_time = cur_time 
            dt = cur_time - self.last_time
            self.last_time = cur_time
            
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0.0

        p_term = self.k_p * distance_error

        self.sigma_error += dt * distance_error

        i_term = self.k_i * self.sigma_error
        
        if i_term > self.k_i_max and self.k_i != 0:
            i_term = self.k_i_max
            self.sigma_error = i_term / self.k_i
        elif i_term < self.k_i_min and self.k_i != 0:
            i_term = self.k_i_min
            self.sigma_error = i_term / self.k_i

        self.d_error = (distance_error - self.distance_error_last) / dt
        self.distance_error_last = distance_error
        
        # Calculate derivative contribution to command 
        d_term = self.k_d * self.d_error
        rospy.loginfo(f"p: {p_term}, i: {i_term}, d:{d_term}")
        return p_term + i_term + d_term

    def run(self):
        tmp=0
        sigma_error=0
        while not rospy.is_shutdown():
            rospy.loginfo("\n\n\n")
            path=self.generate_path()   
            self.get_heading()

            goal_x, goal_y = path[0][tmp],path[1][tmp]

            distance_error = self.dist(goal_x, goal_y)
            linear_speed= self.update_PID(distance_error)
            
            # last_sigma_error = sigma_error
            # sigma_errpor= distance_error

            if distance_error < self.ds:
                rospy.loginfo(f"step {tmp}")
                tmp += 1
                self.i_error = 0


            if tmp == len(path)-1:
                tmp = 1
                self.counter += 1

            angle_to_goal = self.angular_error(goal_x, goal_y)
            
          
 
            # angle_to_goal = angle_to_goal - self.yaw    
            
            # if abs(angle_to_goal) > 0.05:
            z_counterclock = self.k_theta * angle_to_goal
            
            

            rospy.loginfo(f"Angular_error = {angle_to_goal}")
            rospy.loginfo(f"GOAL X, Y {goal_x}, {goal_y}")
            self.vel.linear.x = linear_speed
            self.vel.angular.z = z_counterclock
            
            rospy.loginfo(f"SELF = {self.pose_x},{self.pose_y}")
            rospy.loginfo(f"LOCATION Follower {tmp}")
            rospy.loginfo(f"DISTANCE = {distance_error}" )
      

            self.cmd_publisher.publish(self.vel)
            now = rospy.get_rostime()
            # rospy.loginfo("Time now: ", now.secs)
            next = 0.3
            rospy.loginfo(f"Twist: {self.vel.linear.x}, {self.vel.angular.z}")
            
            rospy.sleep(next)
            

if __name__ == "__main__":
    controller = Controller()
    controller.run()