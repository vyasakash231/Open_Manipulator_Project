#! /usr/bin/python3
import rospy
import sys
from math import degrees, radians
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

rospy.init_node('My_service_node')

rospy.wait_for_service('/goal_joint_space_path') #wait here till the service is Up
service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition) #create service client
request_object = SetJointPositionRequest()

theta_1 = float(input('Enter Joint 1 angle in degrees: '))
theta_2 = float(input('Enter Joint 2 angle in degrees: '))
theta_3 = float(input('Enter Joint 3 angle in degrees: '))
theta_4 = float(input('Enter Joint 4 angle in degrees: '))

# 1st Movement
request_object.planning_group = 'arm'
request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
request_object.joint_position.position = [radians(theta_1), radians(theta_2), radians(theta_3), radians(theta_4)]
request_object.joint_position.max_accelerations_scaling_factor = 1.0
request_object.joint_position.max_velocity_scaling_factor = 1.0
request_object.path_time = 2.0

rospy.loginfo("Doing Service Call 1...")
response = service_client(request_object)
print(response)

""" rospy.sleep(5)

# 2nd Movement
request_object.planning_group = 'arm'
request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
request_object.joint_position.position = [0.0, 0.0, 0.0, 0.0]
request_object.joint_position.max_accelerations_scaling_factor = 1.0
request_object.joint_position.max_velocity_scaling_factor = 1.0
request_object.path_time = 2.0

rospy.loginfo("Doing Service Call 2...")
result = service_client(request_object)
print(result)
rospy.sleep(5) """