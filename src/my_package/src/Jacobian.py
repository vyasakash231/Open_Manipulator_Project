from math import radians, pi
import numpy as np
from Transformation import transformation_matrix

def jacobian_matrix(n,alpha,a,d,theta,le):
    (P_00, R, O) = transformation_matrix(n,alpha,a,d,theta)

    R_n_0 = R[n-1,:,:]
    O_n_0 = np.transpose(np.array([O[:,n-1]]))
    O_E_n = np.array([[le],[0],[0]])
    O_E = O_n_0 + np.dot(R_n_0,O_E_n)

    Jz = np.zeros((3,n))
    Jw = np.zeros((3,n))

    for i in range(0,n):
        Z_i_0 = np.transpose(np.array([R[i,:,2]]))
        O_i_0 = np.transpose(np.array([O[:,i]]))
        O_E_i_0 = O_E - O_i_0

        cross_prod = np.cross(Z_i_0,O_E_i_0,axis=0)
        Jz[:,i] = np.reshape(cross_prod,(3,)) # conver 2D of shape (3,1) to 1D of shape (3,)
        Jw[:,i] = np.reshape(Z_i_0,(3,)) # conver 2D of shape (3,1) to 1D of shape (3,)

    J = np.concatenate((Jz,Jw),axis=0)
    
    return Jz


