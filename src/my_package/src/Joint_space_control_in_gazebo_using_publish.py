#!/usr/bin/python3
import rospy
import time
from math import radians
from std_msgs.msg import Float64
from open_manipulator_msgs.msg import KinematicsPose
from gazebo_msgs.msg import LinkStates

rospy.init_node('My_node',anonymous = False) # Creating a node 'Forward'

joint1_pub = rospy.Publisher('/joint1_position/command',Float64,queue_size=1)
joint2_pub = rospy.Publisher('/joint2_position/command',Float64,queue_size=1)
joint3_pub = rospy.Publisher('/joint3_position/command',Float64,queue_size=1)
joint4_pub = rospy.Publisher('/joint4_position/command',Float64,queue_size=1)

theta_1 = Float64()
theta_2 = Float64()
theta_3 = Float64()
theta_4 = Float64()

# theta_1 = float(input('Enter Joint 1 angle in degrees: '))
# theta_2 = float(input('Enter Joint 2 angle in degrees: '))
# theta_3 = float(input('Enter Joint 3 angle in degrees: '))
# theta_4 = float(input('Enter Joint 4 angle in degrees: '))

# If i want to give input directly from this file I have to give some delay --> 1second to gazebo to respond
theta_1 = float(90)
theta_2 = float(-30)
theta_3 = float(30)
theta_4 = float(20)

time.sleep(2)

joint1_pub.publish(radians(theta_1))
joint2_pub.publish(radians(theta_2))
joint3_pub.publish(radians(theta_3))
joint4_pub.publish(radians(theta_4))
