rom __future__ import division
import numpy as np
from math import *
import math
from numpy.linalg import inv
from math import sin
from math import cos
from numpy import float64, deg2rad, dtype
import __main__
import unittest
from mpmath import jtheta, phi
from _socket import TCP_CORK
from PIL.ImageTransform import Transform
class vectorspace(object):
    def __init__(self,theta_angles=[0,0,0,0,0,0]):
        self.theta_angles=theta_angles
        self.DH_d=[-815,0,0,-1545,0,0,-158]
        self.DH_a=[350,1200,145,0,0,0,0]
        self.DH_alpha=[math.radians(-90),0,math.radians(90),math.radians(90),math.radians(-90),0,0]
      
    def A01(self):
        return transformation(d=self.DH_d[0],a=self.DH_a[0],alpha=self.DH_alpha[0],theta=self.theta_angles[0])
 
    def A12(self):
        return transformation(d=self.DH_d[1],a=self.DH_a[1],alpha=self.DH_alpha[1],theta=self.theta_angles[1]+math.radians(-90))
 
    def A23(self):
        return transformation(d=self.DH_d[2],a=self.DH_a[2],alpha=self.DH_alpha[2],theta=self.theta_angles[2])
 
    def A34(self):
        return transformation(d=self.DH_d[3],a=self.DH_a[3],alpha=self.DH_alpha[3],theta=self.theta_angles[3])
 
    def A45(self):
        return transformation(d=self.DH_d[4],a=self.DH_a[4],alpha=self.DH_alpha[4],theta=self.theta_angles[4])
 
    def A56(self):
        return transformation(d=self.DH_d[5],a=self.DH_a[5],alpha=self.DH_alpha[5],theta=self.theta_angles[5])
   
    def A6tcp(self):
        return transformation(d=self.DH_d[6],a=self.DH_a[6],alpha=self.DH_alpha[6],theta=0)
  
    #Data sheet values
    def A3WRIST(self):
        return transformation(alpha=0, a=0, theta=0, d=self.DH_d[3])

    def base2tcp(self):
        return self.A12()*self.A23()*self.A34()*self.A45()*self.A56()*self.A6tcp()
  
    def base26(self):
        return self.A12()*self.A23()*self.A34()*self.A45()*self.A56()
  
    def baseToWrist(self):
        return self.A01() * self.A12() * self.A23() * self.A3WRIST()
    def __str__(self):
        return"("+",".join(["%.2f"%np.rad2deg(a) for a in self.theta_angles]  )+")"
  
    def __repr__(self):
        return'vectorspace("%s")' % (self.theta_angles)
      
 
class transformation(object):
    #inintialization
    D=0
    d34=0
    gamma=0
    beta=0
  
    DH_d = vectorspace().DH_d
    DH_a = vectorspace().DH_a
    DH_alpha = vectorspace().DH_alpha
  
    def __init__(self,matrix=np.eye(4,4, 0, dtype=float64),d=0,a=0,alpha=0,theta=0):
        self.matrix=matrix
        self.rotate_z(theta)
        self.translate([0,0,d])
        self.rotate_x(alpha)
        self.translate([a,0,0])
        self.min_angles = [math.radians(-185.0),math.radians(-135.0),math.radians(-120.0),math.radians(-350.0),math.radians(-130.0),math.radians(-350.0)]
        self.max_angles = [math.radians(185.0),math.radians(35.0),math.radians(158.0),math.radians(350.0),math.radians(130.0),math.radians(350.0)]
   
    def transform(self,matrix):
        self.matrix=np.dot(matrix,self.matrix)
        return self
 
    def translate(self, vector3):
 
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][3] = vector3[0]
        transform[1][3] = vector3[1]
        transform[2][3] = vector3[2]
        return self.transform(transform)


    def rotate_x(self, angle):

        transform = np.eye(4, 4,dtype=np.float64)
        transform[1][1] = cos(angle)
        transform[1][2] = -1 * sin(angle)
        transform[2][1] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_y(self, angle):
  
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cos(angle)
        transform[0][2] = -1 * sin(angle)
        transform[2][0] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_z(self, angle):

        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cos(angle)
        transform[0][1] = -1 * sin(angle)
        transform[1][0] = sin(angle)
        transform[1][1] = cos(angle)
        return self.transform(transform)
        
        
        if __name__ == '__main__':
    vector=vectorspace()
    #basetotool=vector.A6tcp().matrix
    #print basetotool
    transformat=transformation()
    ik=transformat.IKSolution()
    print ik
    #rotatex=transformat.Ca()
    #print rotatex
    #rotate_x=rotatex.rotate_x(4).matrix
    #print rota
