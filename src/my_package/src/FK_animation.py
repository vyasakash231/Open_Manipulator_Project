from vpython import *  # VPython package in python makes it easy to construct passable 3D displays and animations
from time import *
import numpy as np

file = open("FK_Data", "rb")  # Open Coordinates file
FK_Data = np.load(file)  # Load Coordinates file
X,Y,Z = FK_Data[:,0:6],FK_Data[:,6:12],FK_Data[:,12:18] # Seperate X,Y,Z Coordinates

# Link Lengths
l1 = 0.077
l2 = 0.13
l3 = 0.124
le = 0.126

# Set Axis
x = arrow(axis=vector(1,0,0),color=color.green,length=0.05)
x_label = text(text='X',color=color.green,height=0.015,pos=vector(0.05,0,0))
y = arrow(axis=vector(0,1,0),color=color.blue,length=0.05)
y_label = text(text='Y',color=color.blue,height=0.015,pos=vector(0.005,0.05,0))
z = arrow(axis=vector(0,0,1),color=color.magenta,length=0.05)
z_label = text(text='Z',color=color.magenta,height=0.015,pos=vector(0.003,0,0.05))

# Create Joints
ball_1 = sphere(pos=vector(X[0,1],Z[0,1],Y[0,1]),color = color.red,radius=0.01)
ball_2 = sphere(pos=vector(X[0,2],Z[0,2],Y[0,2]),color = color.red,radius=0.01)
ball_3 = sphere(pos=vector(X[0,3],Z[0,3],Y[0,3]),color = color.red,radius=0.01)
ball_4 = sphere(pos=vector(X[0,4],Z[0,4],Y[0,4]),color = color.red,radius=0.01)

# Create Base Platform
base = box(pos=vector(0,-0.005/2,0),color=color.yellow,length=0.1,width=0.1,height=0.005)

# Create Links
link_1 = cylinder(color=color.white,length=l1,radius=0.005)
link_1.pos = vector(X[0,0],Z[0,0],Y[0,0])
link_1.axis = vector(X[0,1],Z[0,1],Y[0,1]) - link_1.pos

link_2 = cylinder(color=color.white,length=0,radius=0.0045)
link_2.pos = vector(X[0,1],Z[0,1],Y[0,1])
link_2.axis = vector(X[0,2],Z[0,2],Y[0,2]) - link_2.pos

link_3 = cylinder(color=color.white,length=l2,radius=0.004)
link_3.pos = vector(X[0,2],Z[0,2],Y[0,2])
link_3.axis = vector(X[0,3],Z[0,3],Y[0,3]) - link_3.pos

link_4 = cylinder(color=color.white,length=l3,radius=0.0035)
link_4.pos = vector(X[0,3],Z[0,3],Y[0,3])
link_4.axis = vector(X[0,4],Z[0,4],Y[0,4]) - link_4.pos

link_5 = cylinder(color=color.white,length=le,radius=0.002)
link_5.pos = vector(X[0,4],Z[0,4],Y[0,4])
link_5.axis = vector(X[0,5],Z[0,5],Y[0,5]) - link_5.pos

i = 1
while True:
    rate(1) 

    ball_1.pos = vector(X[i,1],Z[i,1],Y[i,1])
    ball_2.pos = vector(X[i,2],Z[i,2],Y[i,2])
    ball_3.pos = vector(X[i,3],Z[i,3],Y[i,3])
    ball_4.pos = vector(X[i,4],Z[i,4],Y[i,4])

    link_1.pos = vector(X[0,0],Z[0,0],Y[0,0])
    link_1.axis = vector(X[0,1],Z[0,1],Y[0,1]) - link_1.pos

    link_2.pos = vector(X[i,1],Z[i,1],Y[i,1])
    link_2.axis = vector(X[i,2],Z[i,2],Y[i,2]) - link_2.pos

    link_3.pos = vector(X[i,2],Z[i,2],Y[i,2])
    link_3.axis = vector(X[i,3],Z[i,3],Y[i,3]) - link_3.pos

    link_4.pos = vector(X[i,3],Z[i,3],Y[i,3])
    link_4.axis = vector(X[i,4],Z[i,4],Y[i,4]) - link_4.pos

    link_5.pos = vector(X[i,4],Z[i,4],Y[i,4])
    link_5.axis = vector(X[i,5],Z[i,5],Y[i,5]) - link_5.pos

    i = i + 1
    if i == 50:
        print('Done!')
        break
    else:
        pass
