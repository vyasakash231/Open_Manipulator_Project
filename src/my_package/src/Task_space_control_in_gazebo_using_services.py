#! /usr/bin/python3
import rospy
from math import radians
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest 

rospy.wait_for_service('/goal_task_space_path') #wait here till the service is Up
service_client = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose) #create service client
request_object = SetKinematicsPoseRequest()

Px = float(input('Enter End-effector Px: '))
Py = float(input('Enter End-effector Py: '))
Pz = float(input('Enter End-effector Pz: '))

# 1st Movement
request_object.planning_group = 'arm'
request_object.end_effector_name = 'gripper'
request_object.kinematics_pose.pose.position.x = Px
request_object.kinematics_pose.pose.position.y = Py
request_object.kinematics_pose.pose.position.z = Pz
request_object.kinematics_pose.max_accelerations_scaling_factor = 1
request_object.kinematics_pose.max_velocity_scaling_factor = 1
request_object.path_time = 2.0

rospy.loginfo("Doing Service Call 1...")
response = service_client(request_object)
print(response)

