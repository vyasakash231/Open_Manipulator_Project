from math import cos, sin, pi
import numpy as np

def transformation_matrix(n,alpha,a,d,theta):
    I = np.eye(4)
    R = np.zeros((n,3,3))
    O = np.zeros((3,n))

    # Transformation Matrix
    for i in range(0,n):
        T = np.array([[      cos(theta[i])        ,      -sin(theta[i])        ,        0      ,        a[i]        ],
                      [sin(theta[i])*cos(alpha[i]), cos(theta[i])*cos(alpha[i]), -sin(alpha[i]), -d[i]*sin(alpha[i])],                                               
                      [sin(theta[i])*sin(alpha[i]), cos(theta[i])*sin(alpha[i]),  cos(alpha[i]),  d[i]*cos(alpha[i])],     
                      [             0             ,             0              ,        0      ,          1         ]])

        T_new = np.dot(I,T)
        R[i,:,:] = T_new[0:3,0:3]
        O[0:3,i] = T_new[0:3,3]
        I = T_new
        i= i + 1

    T_final = I
    d_nn = np.array([[0.138],[0],[0],[1]])
    P_00_home = np.dot(T_final,d_nn)
    P_00 = P_00_home[0:3]
    
    return(P_00, R,O)