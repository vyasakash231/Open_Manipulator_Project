#! /usr/bin/python3
import rospy
from math import radians
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest 
import numpy as np
import matplotlib.pyplot as Plot

rospy.wait_for_service('/goal_task_space_path') #wait here till the service is Up
service_client = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose) #create service client
request_object = SetKinematicsPoseRequest()

lam = np.linspace(0,1,5)

Coordinate_pt = np.array([[-0.207,0.013],[-0.277,0.013],[-0.277,0.087],[-0.207,0.013]])
(r,c) = Coordinate_pt.shape

X = np.array([])
Y = np.array([])

for i in range(0,r-1):
    for j in range(0,len(lam)):
        X = np.concatenate((X,np.array([lam[j] * Coordinate_pt[i+1,0] + (1 - lam[j]) * Coordinate_pt[i,0]])))
        Y = np.concatenate((Y,np.array([lam[j] * Coordinate_pt[i+1,1]  + (1 - lam[j]) * Coordinate_pt[i,1]])))

request_object.planning_group = 'arm'
request_object.end_effector_name = 'gripper'
request_object.kinematics_pose.pose.position.x = Coordinate_pt[0,0]
request_object.kinematics_pose.pose.position.y = Coordinate_pt[0,1]
request_object.kinematics_pose.pose.position.z = 0.083
request_object.kinematics_pose.max_accelerations_scaling_factor = 0.5
request_object.kinematics_pose.max_velocity_scaling_factor = 1
request_object.path_time = 0.5

response = service_client(request_object)

rospy.sleep(1)

request_object.planning_group = 'arm'
request_object.end_effector_name = 'gripper'
request_object.kinematics_pose.pose.position.x = Coordinate_pt[1,0]
request_object.kinematics_pose.pose.position.y = Coordinate_pt[1,1]
request_object.kinematics_pose.pose.position.z = 0.083
request_object.kinematics_pose.max_accelerations_scaling_factor = 0.5
request_object.kinematics_pose.max_velocity_scaling_factor = 1
request_object.path_time = 0.5

response = service_client(request_object)

rospy.sleep(1)

request_object.planning_group = 'arm'
request_object.end_effector_name = 'gripper'
request_object.kinematics_pose.pose.position.x = Coordinate_pt[2,0]
request_object.kinematics_pose.pose.position.y = Coordinate_pt[2,1]
request_object.kinematics_pose.pose.position.z = 0.083
request_object.kinematics_pose.max_accelerations_scaling_factor = 0.5
request_object.kinematics_pose.max_velocity_scaling_factor = 1
request_object.path_time = 0.5

response = service_client(request_object)

rospy.sleep(1)

request_object.planning_group = 'arm'
request_object.end_effector_name = 'gripper'
request_object.kinematics_pose.pose.position.x = Coordinate_pt[3,0]
request_object.kinematics_pose.pose.position.y = Coordinate_pt[3,1]
request_object.kinematics_pose.pose.position.z = 0.083
request_object.kinematics_pose.max_accelerations_scaling_factor = 0.5
request_object.kinematics_pose.max_velocity_scaling_factor = 1
request_object.path_time = 0.5

response = service_client(request_object)

rospy.sleep(1)

fig = Plot.figure()
Plot.xlabel('X')
Plot.plot(X,Y, 'r-')
Plot.ylabel('Y')
Plot.legend(['$End Effector Point$'])
Plot.xlim([-0.55,0.25])
Plot.ylim([-0.25,0.25])
Plot.grid()
Plot.show()