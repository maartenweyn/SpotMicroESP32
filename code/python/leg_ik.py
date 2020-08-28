#https://gitlab.com/custom_robots/spotmicroai/simulation/-/blob/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics/Kinematic.ipynb

from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt

def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax

setupView(110).view_init(elev=20., azim=135)

l1=60.5
l2=10
l3=100.7
l4=118.5



x=-55
y=-100
z=20

def legIK(x,y,z):
    
    """
    x/y/z=Position of the Foot in Leg-Space
    
    F=Length of shoulder-point to target-point on x/y only
    G=length we need to reach to the point on x/y
    H=3-Dimensional length we need to reach
    """

    F=sqrt(x**2+y**2-l1**2)
    G=F-l2  
    H=sqrt(G**2+z**2)

    theta1=-atan2(y,x)-atan2(F,-l1)
    
    D=(H**2-l3**2-l4**2)/(2*l3*l4)
    theta3=acos(D) 
    
    theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))

    return(theta1,theta2,theta3)

def calcLegPoints(angles):
    
    (theta1,theta2,theta3)=angles
    theta23=theta2+theta3

    T0=np.array([0,0,0,1])
    T1=T0+np.array([-l1*cos(theta1),l1*sin(theta1),0,0])
    T2=T1+np.array([-l2*sin(theta1),-l2*cos(theta1),0,0])
    T3=T2+np.array([-l3*sin(theta1)*cos(theta2),-l3*cos(theta1)*cos(theta2),l3*sin(theta2),0])
    T4=T3+np.array([-l4*sin(theta1)*cos(theta23),-l4*cos(theta1)*cos(theta23),l4*sin(theta23),0])
        
    return np.array([T0,T1,T2,T3,T4])

def drawLegPoints(p):

    plt.plot([p[0][0],p[1][0],p[2][0],p[3][0],p[4][0]], 
             [p[0][2],p[1][2],p[2][2],p[3][2],p[4][2]],
             [p[0][1],p[1][1],p[2][1],p[3][1],p[4][1]], 'k-', lw=3)
    plt.plot([p[0][0]],[p[0][2]],[p[0][1]],'bo',lw=2)
    plt.plot([p[4][0]],[p[4][2]],[p[4][1]],'ro',lw=2)

#drawLegPoints(calcLegPoints(legIK(x,y,z)))

for z in range(-100,150,50):
    drawLegPoints(calcLegPoints(legIK(x+25,y,z)))
    drawLegPoints(calcLegPoints(legIK(x-50,y,z)))
    
print("OK")

plt.show()

print("OK\n")