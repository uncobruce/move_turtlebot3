#!/usr/bin/env python

import rospy
import sys
import actionlib
import csv
import math
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import time
import datetime
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge
import cv2


def write2csv():
    # open the file in the write mode
    f = open('./turttime1.csv', 'w')

    # create the csv writer
    writer = csv.writer(f)

    # write a row to the csv file
    writer.writerow(["Time (s)", "x", "y ", "theta","scan"])

def callback(msg):
    ## open the file in the write mode
    #f = open('./turttime1.csv', 'a')
    ## create the csv writer
    #writer = csv.writer(f)

    #print(msg.pose.pose)

    (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    
    #writer.writerow([time.time(), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    ## close the file
    #f.close()

def start_at(hour, min, sec):

    t = datetime.datetime.now()
   
    while t.hour != hour or t.minute != min or t.second != sec:  
        t = datetime.datetime.now()
        #print t.hour, t.minute, t.second


class Move(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "square_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.scan_sub_name = "/scan"
        self.img_sub_name = "/camera/rgb/image_raw"
        self.vel_pub = None
        self.odometry_sub = None
        self.br = CvBridge()

        self.total_distance = 0
        self.previous_x = 0
        self.previous_y = 0

        # ROS params
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None
        self.scan_range = None
        self.img = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        rospy.init_node(self.node_name, log_level=rospy.INFO, anonymous=True)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        #rospy.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = rospy.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = rospy.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)
        self.scan_sub = rospy.Subscriber(self.scan_sub_name, LaserScan, callback=self.__scan_ros_sub, queue_size=self.queue_size)
        self.img_sub = rospy.Subscriber(self.img_sub_name, Image, callback=self.__img_ros_sub, queue_size=self.queue_size)
        
        #rospy.spin()

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not rospy.is_shutdown():
            
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        #sys.exit("The process has been interrupted by the user!")


    def __odom_ros_sub(self, msg):

        self.odom_pose = msg.pose.pose

        #print (self.odom_pose.position.x, self.odom_pose.position.y)

        ## tracking distance traveled 
        x = self.odom_pose.position.x
        y = self.odom_pose.position.y
        d_increment = math.sqrt((x - self.previous_x) * (x - self.previous_x) +
                    (y - self.previous_y)*(y - self.previous_y))
        self.total_distance = self.total_distance + d_increment
        #print("Total distance traveled is {:.2f}m".format(self.total_distance))
        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y
        
        ## open the file in the write mode
        #f = open('./turttime1.csv', 'a')
        ## create the csv writer
        #writer = csv.writer(f)

        #print(msg.pose.pose)

        #(roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        #writer.writerow([time.time(), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, self.scan_range])


    def __scan_ros_sub(self, msg):

        #pass
        self.scan_range = msg.ranges #360 values in set
        # print msg.ranges
        # print len(msg.ranges)  #360
        # print msg.angle_min  #0.11999
        # print msg.angle_max  #3.5

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)

    def __img_ros_sub(self, msg):

        self.img_ros_sub = self.br.imgmsg_to_cv2(msg)
        
        ## show camera img
        # (rows,cols,channels) = self.img_ros_sub.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(self.img_ros_sub, (50,50), 10, 255)
   
        # cv2.imshow("Image window", self.img_ros_sub)
        # cv2.waitKey(3)
        

class MoveOdom(Move):
    """
    This class implements a semi closed-loop square trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the roscore (on the computer or the robot, depending on your configuration)
            $ roscore
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
     - Start this node on your computer:
            $ python move_square odom
    """

    def __init__(self):

        super(MoveOdom, self).__init__()

        self.pub_rate = 0.1

    def get_z_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        #print roll, pitch, yaw
        return yaw
        
    def move_of(self, d, speed=0.35):

        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not rospy.is_shutdown():
            
            #print ('-------', self.odom_pose.position.x, self.odom_pose.position.y)

            sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
            (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
            sys.stdout.flush()

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def turn_of(self, a, ang_speed=0.35):

        # Convert the orientation quaternion message to Euler angles
        a_init = self.get_z_rotation(self.odom_pose.orientation)
        #print ("original theta",a_init)

        # Set the angular velocity forward until angle is reached
        while (abs(self.get_z_rotation(self.odom_pose.orientation) - a_init)) < a and not rospy.is_shutdown():
            
            #print ('------', self.get_z_rotation(self.odom_pose.orientation), a_init)

            sys.stdout.write("\r [TURN] The robot has turned of {:.2f}".format(self.get_z_rotation(self.odom_pose.orientation) - \
                a_init) + "rad over {:.2f}".format(a) + "rad")
            sys.stdout.flush()

            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def debug_move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not rospy.is_shutdown():
            time.sleep(0.1)

        # Implement main instructions
        self.move_of(1)
        self.turn_of(math.pi/4)
        self.move_of(0.5)
        self.turn_of(-(math.pi/2))
        self.stop_robot()

    def move_square(self, distance):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not rospy.is_shutdown():
            time.sleep(0.1)

        # Implement main instructions
        self.move_of(distance)
        self.turn_of(math.pi/2)
        self.move_of(distance)
        self.turn_of(math.pi/2)
        self.move_of(distance)
        self.turn_of(math.pi/2)
        self.move_of(distance)
        self.turn_of(math.pi/2)
        self.stop_robot()
    
    def move_circle(self, radius):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not rospy.is_shutdown():
            time.sleep(0.1)

        distance = (math.pi)*radius*2
        self.total_distance = 0

        while self.total_distance < distance and not rospy.is_shutdown():

            sys.stdout.write("\r [CIRCLE] The robot has moved of {:.2f}".format(self.total_distance) +  "m over {:.2f}".format(distance) + "m for circle with radius of " + str(radius) + "m")
            sys.stdout.flush()

            msg = Twist()
            msg.linear.x = 0.3
            angular_speed = msg.linear.x / radius
            
            ## angular.z unit is rad/s
            msg.angular.z = angular_speed  
            ## radius = msg.linear.x/msg.angular.z
            robotturt.vel_ros_pub(msg)
            
            time.sleep(robotturt.pub_rate)

        self.stop_robot()
        self.total_distance = 0


if __name__ == '__main__':

    robotturt = MoveOdom()
    robotturt.start_ros()
    
    ## start robot path at specific time
    #start_at(16, 54, 12)

    robotturt.debug_move()
    #robotturt.move_square(1)
    robotturt.move_circle(0.5)




