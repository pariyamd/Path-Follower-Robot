#!/usr/bin/python3

import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians, pi,dist
import numpy as np
import time
import matplotlib.pyplot as plt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.width=2
        self.length=3
        self.path=self.generate_path()
        self.errors=[]
        # defining the states of our robot
        self.GOw, self.GOl, self.ROTATEl, self.ROTATEw = 0, 1, 2, 3
        self.state = self.GOw
         
    def generate_path(self):
        X1 = np.linspace(-3, 3 , 10)
        Y1 = np.array([2]*10)

        Y2 = np.linspace(2, -2 , 10)
        X2 = np.array([3]*10)

        X3 = np.linspace(3, -3 , 10)
        Y3 = np.array([-2]*10)

        Y4 = np.linspace(-2, 2 , 10)
        X4 = np.array([-3]*10)

        return (np.concatenate([X1,X2, X3 , X4]), np.concatenate([Y1,Y2,Y3,Y4]))

    # heading of the robot 
    def get_heading(self):
        
        msg = rospy.wait_for_message("/odom" , Odometry, timeout=1)
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw

    def go(self,remaining):
        msg = rospy.wait_for_message("/odom" , Odometry)
        prev_x,prev_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.errors.append(self.get_error(prev_x,prev_y))

        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)
        rospy.sleep(1)
        msg = rospy.wait_for_message("/odom" , Odometry)

        
        while remaining >= self.epsilon:

            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)

            msg = rospy.wait_for_message("/odom" , Odometry)
            current_x, current_y = msg.pose.pose.position.x, msg.pose.pose.position.y
            
            delta = dist([current_x,current_y],[prev_x,prev_y])
            remaining -= delta
            prev_x,prev_y = msg.pose.pose.position.x, msg.pose.pose.position.y
            self.errors.append(self.get_error(prev_x,prev_y))

            rospy.loginfo(f"remaining {remaining}")
        
    def get_error(self,x,y):
        dists=[]
        for i in range(len(self.path)):
            dists.append(dist([self.path[0][i],self.path[1][i]],[x,y]))
        return min(dists)

    def plot_error(self):
        xbar = list(range(len(self.errors)))
        plt.plot(xbar, self.errors)
        plt.xlabel('Step')
        plt.ylabel('Error')
        plt.legend()
        plt.title('Movement Error')
        plt.show()
              
    def rotate(self):
        rospy.loginfo("Rotating")
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)
        remaining_angle = self.goal_angle
        prev_angle = self.get_heading()
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        
        # rotation loop
        while remaining_angle >= 0.1:
            current_angle = self.get_heading()  
            delta = abs(prev_angle - current_angle)
            if (current_angle<0 and prev_angle>0):
                # delta= abs(prev_angle - (current_angle + radians(2*pi)))
                delta = abs(abs(pi -prev_angle) + abs(current_angle +pi))
                rospy.loginfo(f"aaaaaaaaaaaaa : prev:{abs(pi -prev_angle)} current : {abs(current_angle + pi)}")   
            # if (current_angle>0 and prev_angle<0):
            #     delta= abs((prev_angle+ radians(2*pi)) - current_angle )
                rospy.loginfo(f"prev:{prev_angle} current : {current_angle } pi: {radians(pi)}")        
            rospy.loginfo(f"Delta : {delta},prev:{prev_angle} current : {current_angle}, remaining : {remaining_angle}")
            remaining_angle -= delta
            prev_angle = current_angle

        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

    def run(self):
        init_time=time.time()
        self.go(self.width/2)
        self.rotate()
        self.go(self.length/2)
        self.rotate()
        while not rospy.is_shutdown():
            
            # check whether state is changed or not
            if init_time + 100 < time.time():
                self.plot_error()
            if self.state == self.GOw:
                remaining = self.width
                self.go(remaining)
                self.state= self.ROTATEl
                rospy.loginfo("ROTATEW")
                continue
            
            if self.state == self.GOl:
          
                remaining = self.length
                self.go(remaining)
                self.state= self.ROTATEw
                rospy.loginfo("ROTATEW")
                continue
            
            self.rotate()
            
            if self.state == self.ROTATEl:
                rospy.loginfo("GOL")
                self.state = self.GOl
            else:
                rospy.loginfo("GOW")
                self.state = self.GOw


if __name__ == "__main__":
    controller = Controller()
    controller.run()