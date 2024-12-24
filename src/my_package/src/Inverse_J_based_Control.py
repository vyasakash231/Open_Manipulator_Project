#! /usr/bin/python3
import rospy
from math import pi
import numpy as np
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from Jacobian import jacobian_matrix
from std_msgs.msg import Float64
from open_manipulator_msgs.msg import KinematicsPose

# Using class keyword we created our sample class called Task_space_Control
class Task_space_Control():
    # CLASS ATTRIBUTE
    n = 4 # No of joints

    # DH Parameters
    alpha = np.array([0,pi/2,0,0])   
    a = np.array([0,0,0.13,0.124])
    d = np.array([0.077,0,0,0])
    le = 0.126
    
    # Defining a special Method called __init__ which is going to be called upon whenever We create an Instance of the class
    def __init__(self):  # First special Method of class also called class constructor/Initilization method
        rospy.init_node('My_node',anonymous = False) # Creating a node 'My_node'

        # INSTANCE ATTRIBUTE 
        rospy.wait_for_service('/goal_joint_space_path') #wait here till the service is Up
        self.service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition) #create service client
        self.request_object = SetJointPositionRequest()

        self.current_Px = None
        self.current_Py = None
        self.current_Pz = None

        self.Pose_sub = rospy.Subscriber('/gripper/kinematics_pose',KinematicsPose,self.Update_pose)

    def Update_pose(self,data):
        self.current_Px = round(data.pose.position.x, 5)
        self.current_Py = round(data.pose.position.y, 5)
        self.current_Pz = round(data.pose.position.z, 5)
    
    def move_func(self):

        i = 0

        theta_1 = Float64()
        theta_2 = Float64()
        theta_3 = Float64()
        theta_4 = Float64()

        del_X = np.ones((3,1)) # defining an array to start the loop
        theta = np.array([[0,1.38544838,-1.38544838,0]]) # Initial Joint position as per DH convention

        while np.max(np.abs(del_X[:,0])) > 0.05:
            #[Px,Py,Pz] = task.current_pose()
            Px, Py, Pz = self.current_Px, self.current_Py, self.current_Pz
            
            # Newton Raphson Mathod
            del_X = np.array([[goal_Px - Px],[goal_Py - Py],[goal_Pz - Pz]])
            print(np.reshape(del_X,(3,)))
            
            Jv = jacobian_matrix(Task_space_Control.n,Task_space_Control.alpha,Task_space_Control.a,Task_space_Control.d,theta[i,:],Task_space_Control.le)  # Calculate j
            Jv_pseudo = np.round(np.linalg.pinv(Jv), 5)
            theta_new = np.degrees(theta[i,:]) + np.reshape(np.dot(Jv_pseudo,del_X),(4,))
            theta = np.vstack((theta,np.radians(theta_new)))
            #theta = np.radians(theta_new)
            
            # Subtracting Home Position Joint Angle as per DH-Convention and Publishing it
            theta_1 = theta[i,0] - theta[0,0] 
            theta_2 = theta[i,1] - theta[0,1] 
            theta_3 = theta[i,2] - theta[0,2] 
            theta_4 = theta[i,3] - theta[0,3] 

            # 1st Movement
            self.request_object.planning_group = 'arm'
            self.request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            self.request_object.joint_position.position = [theta_1, theta_2, theta_3, theta_4]
            self.request_object.joint_position.max_accelerations_scaling_factor = 1
            self.request_object.joint_position.max_velocity_scaling_factor = 1
            self.request_object.path_time = 0.1

            self.response = self.service_client(self.request_object)
            
            i = i+1

            rospy.sleep(0.1) 

        rospy.signal_shutdown("Times's Up!")

        #rospy.spin() # To stop the loop and program by pressing ctr + C 

""" when this file is being executted no class or function run directly the only condition which run first is the line which has 0 indentation except function 
and class so only if __name__ == '__main__' is left which will be executted first and that will call turtle_move() class and move2goal() method. """
            
if __name__ == "__main__":
    try:
        goal_Px = 0.17
        goal_Py = 0.13
        goal_Pz = 0.1

        task = Task_space_Control()  # task = Instance / object of Task_space_Control class
        rospy.sleep(2)
        task.move_func()  # Publish new joint angle
    
    except rospy.ROSInterruptException:
        pass 