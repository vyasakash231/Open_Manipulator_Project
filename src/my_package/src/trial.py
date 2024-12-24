#! /usr/bin/python3
"""
The line at the top of this Python script is called a "shebang" line. It specifies the path to the Python interpreter that should be used to execute the script.
When you run a script in the terminal using ./script.py, the system looks for the shebang line at the top of the script to determine which interpreter to use. 
In this case, #!/usr/bin/python3 specifies that the system should use the python interpreter that is found in the user's PATH environment variable.
"""
import rospy
from math import pi, atan2
import numpy as np
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from Jacobian import jacobian_matrix
from Transformation import transformation_matrix
from open_manipulator_msgs.msg import KinematicsPose
from Joint_Limit_Avoidance import weight_Func
from matplotlib import pyplot as Plot

"""
To run this program open 3 terminals,
In first run: roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
In second run: roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false          --> close and repeat this command to reset gazebo
In third run: rosrun my_package Inverse_Jacobian_based_Control_with_JLA.py
"""

# Using class keyword we created our sample class called Task_space_Control
class Task_Space_Control():
    # CLASS ATTRIBUTE
    n = 4  # No of joints
    m = 3  # mth norm of a vector
    K = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) 

    # DH Parameters
    alpha = np.array([0,pi/2,0,0])   
    a = np.array([0,0,0.13,0.124])
    d = np.array([0.077,0,0,0])
    #le = 0.126
    le = 0.138

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('My_service_node')

        # Subscribe to the Kinematic pose of the manipulator
        rospy.Subscriber('/gripper/kinematics_pose',KinematicsPose,self.Update_pose)

        # wait here till the service is Up
        rospy.wait_for_service('/goal_joint_space_path')

        # Initialize the service client to set the pose of the manipulator in Gazebo
        self.service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)  # create service client
        self.request_object = SetJointPositionRequest()

        # Initialize the joint angles
        self.current_Px = None
        self.current_Py = None
        self.current_Pz = None

        self.k_p = 0.5

    def Update_pose(self,data):
        self.current_Px = data.pose.position.x
        self.current_Py = data.pose.position.y
        self.current_Pz = data.pose.position.z

#################################################################### METHOD - 1 #########################################################################    
    def JLA_1(self,del_X):  # Inequality Constraint Method
        i = 0
        q = np.array([0,0,0,0])  # Hardware & Gazebo Joint Angle at Home Position

        joint_offset = np.radians(np.array([0,79.380345,-79.380345,0]))  # Difference btw Hardware/Gazebo & DH convention
        _,R_joint_offset,_ = transformation_matrix(Task_Space_Control.n,Task_Space_Control.alpha,Task_Space_Control.a,Task_Space_Control.d,joint_offset) # (4,3,3)

        theta = q + joint_offset  # Initial Joint position as per DH convention
        _,R_theta,_ = transformation_matrix(Task_Space_Control.n,Task_Space_Control.alpha,Task_Space_Control.a,Task_Space_Control.d,theta) # (4,3,3)

        Jc = np.eye((Task_Space_Control.n))
        epsilon = 0.5*np.ones(q.shape) # Activation buffer
        
        q_range = np.radians(np.array([[-180,180],[-117,90],[-90,87.5],[-103,114.5]]))

        theta_new = np.zeros(theta.shape)

        #while np.max(np.abs(del_X[:,i])) > 0.003:
        while np.linalg.norm(del_X[:,i]) > 0.003:
            if i == 200: # break if the error is not reducing 
                break

            # If any Joint breach the joint limit, the loop will end 
            if q_range[0,0] > q[0] or q[0] > q_range[0,1]:
                rospy.signal_shutdown('Joint 1 Limit breached')

            if q_range[1,0] > q[1] or q[1] > q_range[1,1]:
                rospy.signal_shutdown('Joint 2 Limit breached')
              
            if q_range[2,0] > q[2] or q[2] > q_range[2,1]:
                rospy.signal_shutdown('Joint 3 Limit breached')

            if q_range[3,0] > q[3] or q[3] > q_range[3,1]:
                rospy.signal_shutdown('Joint 4 Limit breached')
 
            # Calculating Weight Matrix
            We, Wc, Wv = weight_Func(Task_Space_Control.m,Task_Space_Control.n,q_range,q,epsilon)

            Px,Py,Pz = self.current_Px,self.current_Py,self.current_Pz  # Forward Kinematics
            #print('Coord: ',Px,',',Py,',',Pz,'\n')

            # Newton Raphson Mathod
            del_X = np.hstack((del_X, np.array([[goal_Px - Px],[goal_Py - Py],[goal_Pz - Pz]])))
            print('del_X norm: ',np.linalg.norm(del_X[:,i]))

            # Calculate Jacobain
            J = jacobian_matrix(Task_Space_Control.n,Task_Space_Control.alpha,Task_Space_Control.a,Task_Space_Control.d,theta,Task_Space_Control.le)  # Calculate J
            Je = J[0:3,:]

            Jn = np.linalg.inv(np.transpose(Je) @ We @ Je + np.transpose(Jc) @ Wc @ Jc + Wv) @ np.transpose(Je) @ We
            
            # Calculating Next joint Position
            d_theta = np.reshape(np.radians(Jn @ del_X[:,i+1]),(4,))
            _,R_d_theta,_ = transformation_matrix(Task_Space_Control.n,Task_Space_Control.alpha,Task_Space_Control.a,Task_Space_Control.d,d_theta)

            R_theta_new = np.matmul(R_theta, R_d_theta) # theta_new = theta + d_theta
            R_q = np.matmul(np.transpose(R_joint_offset,(0,2,1)), R_theta_new)
            
            for h in range(Task_Space_Control.n):
                theta_new[h] = atan2(R_theta_new[h,1,0],R_theta_new[h,0,0]) # for DH-parameters
                q[h] = atan2(R_q[h,1,0],R_q[h,0,0]) # for Hardware and Gazebo

            theta = theta_new  # for DH-convention

            # Sending New Joint Coordinates to the Robotic ARM using service argumment -> planning_group, joint_position, path_time
            self.request_object.planning_group = 'arm'
            self.request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            self.request_object.joint_position.position = [q[0],q[1],q[2],q[3]]
            self.request_object.joint_position.max_accelerations_scaling_factor = 0.5
            self.request_object.joint_position.max_velocity_scaling_factor = 0.5
            self.request_object.path_time = 0.1

            self.response = self.service_client(self.request_object)

            i = i + 1

            rospy.sleep(0.1)  # small delay before calling another service
        
        fig = Plot.figure()
        Plot.xlabel('No of Iteration')
        Plot.ylabel('$\Delta$X, $\Delta$Y, $\Delta$Z')
        Plot.legend(['$e_{X}$','$e_{Y}$','$e_{Z}$'])
        Plot.grid()
        
        Plot.plot(range(1,i),del_X[0,1:i], 'r-')
        Plot.plot(range(1,i),del_X[1,1:i], 'b-')
        Plot.plot(range(1,i),del_X[2,1:i], 'g-')
        Plot.show()

        rospy.spin()  # To stop the loop and program by pressing ctr + C 

""" 
when this file is being executted no class or function run directly the only condition which run first is the 
line which has 0 indentation (except function and class) so only {if __name__ == '__main__':} is left which 
will be executted first and that will call turtle_move() class and move2goal() method. 
"""

if __name__ == "__main__":
    try:
        goal_Px = 0.17
        goal_Py = -0.13
        goal_Pz = 0.0

        # fig = []

        del_X = np.ones((3,1))  # defining an array to start the loop

        task = Task_Space_Control()  # task = Instance / object of Task_space_Control class 
        rospy.sleep(0.5)  # Give buffer time for Service to activate 
        task.JLA_1(del_X)  # Publish new joint angle using JLA_1

    except rospy.ROSInterruptException:
        pass 